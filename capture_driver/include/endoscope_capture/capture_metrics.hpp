#pragma once

#include "endoscope_capture/decklink_capture.hpp"

#include <sys/resource.h>
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

namespace endoscope_capture {
namespace capture_metrics_detail {
using Clock = std::chrono::steady_clock;
inline double seconds_since(Clock::time_point start) {
  return std::chrono::duration<double>(Clock::now() - start).count();
}
inline double percentile(std::vector<double> values, double fraction) {
  if (values.empty()) return 0;
  std::sort(values.begin(), values.end());
  return values[static_cast<std::size_t>(fraction * (values.size() - 1))];
}

}  // namespace capture_metrics_detail

struct CaptureMetrics {
  using Clock = std::chrono::steady_clock;
  std::uint64_t published = 0, pool_exhausted = 0, frame_processing_errors = 0;
  std::uint64_t nonmonotonic_stamps = 0, last_source_sequence = 0;
  std::uint64_t copied_bytes = 0;
  std::vector<double> copy_ms;
  std::vector<double> publish_ms;
  Clock::time_point started = Clock::now();
  std::clock_t cpu_started = std::clock();

  void sample(std::vector<double>& values, double value) {
    if (values.size() < 10000) values.push_back(value);
    else values[published % values.size()] = value;
  }

  std::string json(const endoscope_capture::CaptureStats& stats) const {
    const double elapsed = capture_metrics_detail::seconds_since(started);
    rusage usage{};
    getrusage(RUSAGE_SELF, &usage);
    std::ostringstream out;
    out << std::fixed << std::setprecision(6)
        << "{\"timestamp_semantics\":\"SDK_callback_arrival_system_clock\","
        << "\"elapsed_seconds\":" << elapsed
        << ",\"published\":" << published
        << ",\"published_hz\":" << (elapsed > 0 ? published / elapsed : 0)
        << ",\"callbacks\":" << stats.callbacks
        << ",\"frames_received\":" << stats.frames_received
        << ",\"frames_delivered\":" << stats.frames_delivered
        << ",\"overwritten_before_delivery\":" << stats.overwritten
        << ",\"discarded_before_delivery\":" << stats.discarded
        << ",\"pending_frames\":" << stats.pending_frames
        << ",\"no_signal_callbacks\":" << stats.no_signal
        << ",\"format_changes\":" << stats.format_changes
        << ",\"invalid_frames\":" << stats.invalid_frames
        << ",\"wait_timeouts\":" << stats.timeouts
        << ",\"sdk_errors\":" << stats.sdk_errors
        << ",\"signal_present\":" << (stats.signal_present ? "true" : "false")
        << ",\"running\":" << (stats.running ? "true" : "false")
        << ",\"pool_exhausted\":" << pool_exhausted
        << ",\"frame_processing_errors\":" << frame_processing_errors
        << ",\"nonmonotonic_stamps\":" << nonmonotonic_stamps
        << ",\"last_source_sequence\":" << last_source_sequence
        << ",\"copied_payload_bytes\":" << copied_bytes
        << ",\"publish_timing_scope\":\"stereo_processing_and_all_outputs\""
        << ",\"copy_ms_p50\":" << capture_metrics_detail::percentile(copy_ms, 0.50)
        << ",\"copy_ms_p95\":" << capture_metrics_detail::percentile(copy_ms, 0.95)
        << ",\"copy_ms_max\":" << capture_metrics_detail::percentile(copy_ms, 1.0)
        << ",\"publish_ms_p50\":" << capture_metrics_detail::percentile(publish_ms, 0.50)
        << ",\"publish_ms_p95\":" << capture_metrics_detail::percentile(publish_ms, 0.95)
        << ",\"publish_ms_max\":" << capture_metrics_detail::percentile(publish_ms, 1.0)
        << ",\"timing_sample_limit\":10000"
        << ",\"cpu_percent_one_core\":"
        << (elapsed > 0 ? 100.0 * (std::clock() - cpu_started) / CLOCKS_PER_SEC / elapsed : 0)
        << ",\"peak_rss_mib\":" << usage.ru_maxrss / 1024.0 << "}";
    return out.str();
  }
};

}  // namespace endoscope_capture
