#include "endoscope_capture/ros2_publisher.hpp"

#include <endoscope_capture/msg/capture_status.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace endoscope_capture {
namespace {
using Clock = std::chrono::steady_clock;

rclcpp::QoS publication_qos(const std::string& reliability, std::size_t depth) {
  if (depth < 1 || depth > 32)
    throw std::invalid_argument("qos_depth must be between 1 and 32");
  auto qos = rclcpp::QoS(rclcpp::KeepLast(depth)).durability_volatile();
  if (reliability == "reliable") return qos.reliable();
  if (reliability == "best_effort") return qos.best_effort();
  throw std::invalid_argument("qos_reliability must be reliable or best_effort");
}
}  // namespace

struct Ros2Publisher::Impl {
  Impl(rclcpp::Node& node, const SplitterConfig& config,
       const CameraInfo& calibration, const std::string& reliability, std::size_t depth)
      : processor(config), combined_calibration(calibration), logger(node.get_logger()) {
    const auto qos = publication_qos(reliability, depth);
    combined = node.create_publisher<Image>("~/image_raw", qos);
    actual_qos_depth = combined->get_actual_qos().depth();
    combined_info = node.create_publisher<CameraInfo>("~/camera_info", qos);
    left = node.create_publisher<Image>("left/image_raw", qos);
    right = node.create_publisher<Image>("right/image_raw", qos);
    left_info = node.create_publisher<CameraInfo>("left/camera_info", qos);
    right_info = node.create_publisher<CameraInfo>("right/camera_info", qos);
    status = node.create_publisher<msg::CaptureStatus>("~/status", qos);
    diagnostics = node.create_publisher<std_msgs::msg::String>("~/diagnostics", qos);
  }

  StereoProcessor processor;
  CameraInfo combined_calibration;
  rclcpp::Logger logger;
  rclcpp::Publisher<Image>::SharedPtr combined, left, right;
  rclcpp::Publisher<CameraInfo>::SharedPtr combined_info, left_info, right_info;
  rclcpp::Publisher<msg::CaptureStatus>::SharedPtr status;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diagnostics;
  std::size_t actual_qos_depth = 0;
  std::uint64_t received = 0, processed = 0, errors = 0, pool_exhausted = 0;
  Clock::time_point started = Clock::now(), first{}, last{};
};

Ros2Publisher::Ros2Publisher(rclcpp::Node& node, const SplitterConfig& config,
                             const CameraInfo& calibration,
                             const std::string& reliability, std::size_t depth)
    : impl_(new Impl(node, config, calibration, reliability, depth)) {}
Ros2Publisher::~Ros2Publisher() = default;

void Ros2Publisher::validate_input_size(std::uint32_t width, std::uint32_t height) const {
  impl_->processor.validate_input_size(width, height);
  const auto& calibration = impl_->combined_calibration;
  if ((calibration.width || calibration.height) &&
      (calibration.width != width || calibration.height != height))
    throw std::invalid_argument("Combined calibration dimensions differ from capture input");
}

bool Ros2Publisher::publish(const ImageConstPtr& source, double configured_hz,
                            const PublishRateEstimator::Sample& measured,
                            std::uint64_t source_sequence) {
  ++impl_->received;
  try {
    if (!source) throw std::invalid_argument("Null combined frame");
    validate_input_size(source->width, source->height);
    StereoPair pair;
    if (!impl_->processor.process(source, pair)) {
      ++impl_->pool_exhausted;
      return false;
    }
    auto info = impl_->combined_calibration;
    info.width = source->width;
    info.height = source->height;
    info.header = source->header;
    msg::CaptureStatus status;
    status.header = source->header;
    status.configured_publish_hz = configured_hz;
    status.measured_publish_hz = measured.hz;
    status.measured_rate_valid = measured.valid;
    status.rate_window_samples = measured.samples;
    status.rate_window_seconds = measured.window_seconds;
    status.source_sequence = source_sequence;
    impl_->combined->publish(*source);
    impl_->combined_info->publish(info);
    impl_->left->publish(*pair.left);
    impl_->left_info->publish(*pair.left_info);
    impl_->right->publish(*pair.right);
    impl_->right_info->publish(*pair.right_info);
    impl_->status->publish(status);
    const auto now = Clock::now();
    if (impl_->processed == 0) impl_->first = now;
    impl_->last = now;
    ++impl_->processed;
    return true;
  } catch (const std::exception& error) {
    ++impl_->errors;
    RCLCPP_ERROR(impl_->logger, "Frame publication failed: %s", error.what());
    return false;
  }
}

void Ros2Publisher::diagnostics(const std::string& report) {
  std_msgs::msg::String message;
  message.data = report;
  impl_->diagnostics->publish(message);
}

std::string Ros2Publisher::stats_json() const {
  const double span = std::chrono::duration<double>(impl_->last - impl_->first).count();
  const double elapsed = std::chrono::duration<double>(Clock::now() - impl_->started).count();
  std::ostringstream report;
  report << std::fixed << std::setprecision(6)
         << "{\"received\":" << impl_->received
         << ",\"processed\":" << impl_->processed
         << ",\"errors\":" << impl_->errors
         << ",\"pool_exhausted\":" << impl_->pool_exhausted
         << ",\"qos_depth\":" << impl_->actual_qos_depth
         << ",\"processed_hz\":" << (impl_->processed > 1 && span > 0
                                        ? (impl_->processed - 1) / span : 0)
         << ",\"elapsed_seconds\":" << elapsed << '}';
  return report.str();
}

}  // namespace endoscope_capture
