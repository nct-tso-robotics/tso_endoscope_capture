#include "endoscope_capture/stereo_splitter.hpp"

#include <image_transport/image_transport.h>
#include <ros/console.h>
#include <sys/resource.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <deque>
#include <exception>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <utility>
#include <vector>

namespace endoscope_capture {
namespace {
using Clock = std::chrono::steady_clock;

double elapsed_seconds(Clock::time_point start) {
  return std::chrono::duration<double>(Clock::now() - start).count();
}

double percentile(const std::deque<double>& samples, double fraction) {
  std::vector<double> ordered(samples.begin(), samples.end());
  std::sort(ordered.begin(), ordered.end());
  const double index = fraction * (ordered.size() - 1);
  const auto lower = static_cast<std::size_t>(std::floor(index));
  const auto upper = static_cast<std::size_t>(std::ceil(index));
  // Interpolate adjacent samples at the fractional percentile index.
  return ordered[lower] + (ordered[upper] - ordered[lower]) * (index - lower);
}

}  // namespace

struct StereoSplitter::Impl {
  Impl(ros::NodeHandle output_namespace, const SplitterConfig& config)
      : processor(config), transport(output_namespace),
        left(transport.advertiseCamera("left/image_raw", 1)),
        right(transport.advertiseCamera("right/image_raw", 1)) {}

  struct GapEvent {
    double seconds;
    std::uint64_t missing;
  };
  mutable std::mutex mutex;
  StereoProcessor processor;
  image_transport::ImageTransport transport;
  image_transport::CameraPublisher left;
  image_transport::CameraPublisher right;
  std::uint64_t received = 0;
  std::uint64_t processed = 0;
  std::uint64_t errors = 0;
  std::uint64_t pool_exhausted = 0;
  std::uint64_t sequence_gaps = 0;
  std::uint64_t duplicates = 0;
  std::uint64_t backwards = 0;
  std::uint32_t last_sequence = 0;
  std::uint64_t last_stamp = 0;
  bool have_previous = false;
  Clock::time_point started = Clock::now();
  std::clock_t cpu_started = std::clock();
  Clock::time_point first_processed{};
  Clock::time_point last_processed{};
  std::deque<double> durations_ms;
  std::deque<GapEvent> gaps;
};

StereoSplitter::StereoSplitter(ros::NodeHandle output_namespace,
                               const SplitterConfig& config)
    : impl_(new Impl(std::move(output_namespace), config)) {}
StereoSplitter::~StereoSplitter() = default;

void StereoSplitter::validate_input_size(std::uint32_t width, std::uint32_t height) const {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->processor.validate_input_size(width, height);
}

bool StereoSplitter::process(const sensor_msgs::ImageConstPtr& source) {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  const auto started = Clock::now();
  ++impl_->received;
  if (source) {
    const auto stamp = source->header.stamp.toNSec();
    if (impl_->have_previous) {
      const std::uint32_t delta = source->header.seq - impl_->last_sequence;
      if (delta > 1 && delta < (std::uint32_t{1} << 31)) {
        impl_->sequence_gaps += delta - 1;
        if (impl_->gaps.size() == 256) impl_->gaps.pop_front();
        impl_->gaps.push_back({elapsed_seconds(impl_->started), delta - 1});
      }
      if (stamp == impl_->last_stamp) ++impl_->duplicates;
      if (stamp < impl_->last_stamp) ++impl_->backwards;
    }
    impl_->last_stamp = stamp;
    impl_->last_sequence = source->header.seq;
    impl_->have_previous = true;
  }
  try {
    StereoPair pair;
    if (!impl_->processor.process(source, pair)) {
      ++impl_->pool_exhausted;
      ROS_WARN_THROTTLE(5, "Stereo output buffers retained; frame dropped with bounded memory");
      return false;
    }
    impl_->left.publish(pair.left, pair.left_info);
    impl_->right.publish(pair.right, pair.right_info);
    const auto finished = Clock::now();
    if (impl_->processed == 0) impl_->first_processed = finished;
    ++impl_->processed;
    impl_->last_processed = finished;
    if (impl_->durations_ms.size() == 10000) impl_->durations_ms.pop_front();
    impl_->durations_ms.push_back(
        std::chrono::duration<double, std::milli>(finished - started).count());
    return true;
  } catch (const std::exception& error) {
    ++impl_->errors;
    ROS_ERROR_STREAM_THROTTLE(5, "Rejected stereo input frame: " << error.what());
    return false;
  }
}

std::string StereoSplitter::stats_json() const {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  const double elapsed = elapsed_seconds(impl_->started);
  const double span = impl_->processed > 1
                          ? std::chrono::duration<double>(impl_->last_processed -
                                                          impl_->first_processed).count()
                          : 0;
  rusage usage{};
  getrusage(RUSAGE_SELF, &usage);
  std::ostringstream out;
  out << std::fixed << std::setprecision(6)
      << "{\"received\":" << impl_->received
      << ",\"processed\":" << impl_->processed
      << ",\"errors\":" << impl_->errors
      << ",\"pool_exhausted\":" << impl_->pool_exhausted
      << ",\"source_sequence_gaps\":" << impl_->sequence_gaps
      << ",\"duplicate_source_stamps\":" << impl_->duplicates
      << ",\"backwards_source_stamps\":" << impl_->backwards
      << ",\"processed_hz\":" << (span > 0 ? (impl_->processed - 1) / span : 0)
      << ",\"elapsed_seconds\":" << elapsed
      << ",\"cpu_percent_one_core\":"
      << (elapsed > 0 ? 100.0 * (std::clock() - impl_->cpu_started) / CLOCKS_PER_SEC / elapsed : 0)
      << ",\"peak_rss_mib\":" << usage.ru_maxrss / 1024.0;
  if (impl_->durations_ms.empty()) {
    out << ",\"callback_ms_p50\":null,\"callback_ms_p95\":null,\"callback_ms_max\":null";
  } else {
    out << ",\"callback_ms_p50\":" << percentile(impl_->durations_ms, 0.50)
        << ",\"callback_ms_p95\":" << percentile(impl_->durations_ms, 0.95)
        << ",\"callback_ms_max\":" << percentile(impl_->durations_ms, 1.0);
  }
  out << ",\"callback_sample_limit\":10000,\"source_gap_events\":[";
  bool first = true;
  for (const auto& gap : impl_->gaps) {
    if (!first) out << ',';
    first = false;
    out << "{\"seconds_since_start\":" << gap.seconds
        << ",\"missing\":" << gap.missing << '}';
  }
  out << "]}";
  return out.str();
}

}  // namespace endoscope_capture
