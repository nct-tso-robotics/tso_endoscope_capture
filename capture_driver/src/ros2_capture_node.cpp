#include "endoscope_capture/capture_metrics.hpp"
#include "endoscope_capture/decklink_capture.hpp"
#include "endoscope_capture/message_pool.hpp"
#include "endoscope_capture/ros2_publisher.hpp"

#include <camera_info_manager/camera_info_manager.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {
using Clock = std::chrono::steady_clock;
using namespace endoscope_capture;

double seconds_since(Clock::time_point start) {
  return std::chrono::duration<double>(Clock::now() - start).count();
}

template <class T>
T startup_parameter(rclcpp::Node& node, const std::string& name, const T& value) {
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = true;
  return node.declare_parameter<T>(name, value, descriptor);
}

std::ofstream report_file(const std::string& path) {
  std::ofstream stream;
  if (!path.empty()) {
    stream.open(path, std::ios::out | std::ios::trunc);
    if (!stream) throw std::runtime_error("Cannot open report file: " + path);
  }
  return stream;
}

void write_report(std::ofstream& stream, const std::string& report) {
  if (stream.is_open()) {
    stream << report << '\n';
    stream.flush();
    if (!stream) throw std::runtime_error("Failed writing capture report");
  }
}

int run(const rclcpp::Node::SharedPtr& node) {
  const int device = startup_parameter(*node, "device_index", 0);
  const int buffers = startup_parameter(*node, "buffer_count", 4);
  const double rate = startup_parameter(*node, "publish_rate", 30.0);
  const double duration = startup_parameter(*node, "duration_seconds", 0.0);
  const auto mode = startup_parameter(*node, "mode", std::string("1080p5994"));
  const auto frame_id = startup_parameter(*node, "frame_id", std::string("camera"));
  const auto calibration_url = startup_parameter(*node, "camera_info_url", std::string());
  const auto reliability = startup_parameter(*node, "qos_reliability", std::string("reliable"));
  const int qos_depth = startup_parameter(*node, "qos_depth", 4);
  const auto metrics_path = startup_parameter(*node, "metrics_file", std::string());
  const auto split_metrics_path = startup_parameter(*node, "split_metrics_file", std::string());
  if (node->get_parameter("use_sim_time").as_bool())
    throw std::invalid_argument("Hardware capture requires system ROS time");
  const auto time_guard = node->add_on_set_parameters_callback(
      [](const std::vector<rclcpp::Parameter>& parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto& parameter : parameters) {
          if (parameter.get_name() == "use_sim_time" && parameter.as_bool()) {
            result.successful = false;
            result.reason = "Hardware capture requires system ROS time";
          }
        }
        return result;
      });
  if (device < 0 || buffers < 2 || buffers > 32 || !std::isfinite(rate) ||
      rate < 0.1 || rate > 240 || !std::isfinite(duration) || duration < 0 || frame_id.empty())
    throw std::invalid_argument("Invalid device, buffer count (2..32), publish_rate [0.1,240], duration or frame_id");
  if (!metrics_path.empty() && metrics_path == split_metrics_path)
    throw std::invalid_argument("Capture and splitter metrics require different files");
  auto metrics_output = report_file(metrics_path);
  auto split_metrics_output = report_file(split_metrics_path);

  SplitterConfig config;
  config.left_calibration = startup_parameter(*node, "left_calibration", std::string());
  config.right_calibration = startup_parameter(*node, "right_calibration", std::string());
  config.scale_x = startup_parameter(*node, "scale_x", 0.5);
  config.scale_y = startup_parameter(*node, "scale_y", 1.0);
  config.opencv_threads = startup_parameter(*node, "opencv_threads", 1);
  config.buffer_count = static_cast<std::size_t>(buffers);
  CameraInfo calibration;
  if (!calibration_url.empty()) {
    camera_info_manager::CameraInfoManager manager(node.get(), "capture");
    if (!manager.loadCameraInfo(calibration_url))
      throw std::runtime_error("Cannot load combined-image calibration");
    calibration = manager.getCameraInfo();
  }
  if (qos_depth < 1 || qos_depth > 32)
    throw std::invalid_argument("qos_depth must be between 1 and 32");
  Ros2Publisher publisher(*node, config, calibration, reliability,
                          static_cast<std::size_t>(qos_depth));
  DeckLinkCapture capture;
  capture.start(static_cast<unsigned>(device), mode);
  const auto width = static_cast<std::uint32_t>(capture.width());
  const auto height = static_cast<std::uint32_t>(capture.height());
  publisher.validate_input_size(width, height);
  if (!calibration_url.empty() && (calibration.width != width || calibration.height != height))
    throw std::invalid_argument("Combined calibration dimensions differ from the selected capture mode");
  if (width > std::numeric_limits<std::uint32_t>::max() / 4 ||
      static_cast<std::size_t>(width) * 4 > std::numeric_limits<std::size_t>::max() / height)
    throw std::runtime_error("Capture dimensions overflow ROS image storage");
  MessagePool<ImagePtr> pool(config.buffer_count, [&] {
    auto image = std::make_shared<Image>();
    image->width = width;
    image->height = height;
    image->encoding = "bgra8";
    image->is_bigendian = false;
    image->step = width * 4;
    image->header.frame_id = frame_id;
    image->data.resize(static_cast<std::size_t>(image->step) * height);
    return image;
  });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  PublishRateEstimator estimator(rate);
  CaptureMetrics metrics;
  const auto period = std::chrono::duration_cast<Clock::duration>(
      std::chrono::duration<double>(1.0 / rate));
  auto next_publication = Clock::now(), diagnostics_due = Clock::now();
  bool have_deadline = false;
  std::int64_t last_stamp = 0;
  std::uint64_t previous_no_signal = 0;
  RCLCPP_INFO(node->get_logger(), "Capturing %ux%u BGRA at up to %.3f Hz", width, height, rate);
  while (rclcpp::ok() && (duration == 0 || seconds_since(metrics.started) < duration)) {
    executor.spin_some();
    if (!rclcpp::ok()) break;
    if (Clock::now() >= diagnostics_due) {
      publisher.diagnostics(metrics.json(capture.stats()));
      diagnostics_due = Clock::now() + std::chrono::seconds(1);
    }
    const auto now = Clock::now();
    if (now < next_publication) {
      std::this_thread::sleep_for(std::min(next_publication - now,
          std::chrono::duration_cast<Clock::duration>(std::chrono::milliseconds(20))));
      continue;
    }
    auto frame = capture.wait_for_frame(std::chrono::milliseconds(250));
    const auto state = capture.stats();
    if (state.no_signal != previous_no_signal) {
      estimator.reset();
      have_deadline = false;
    }
    previous_no_signal = state.no_signal;
    if (state.format_changes) {
      RCLCPP_ERROR(node->get_logger(), "Input format changed; reconfigure the capture mode and restart");
      break;
    }
    if (!frame) {
      estimator.reset();
      have_deadline = false;
      RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 5000,
                           "Waiting for valid capture input");
      continue;
    }
    auto message = pool.acquire();
    if (!message) {
      ++metrics.pool_exhausted;
      continue;
    }
    if (frame->timestamp_ns() <= last_stamp) {
      ++metrics.nonmonotonic_stamps;
      RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 5000,
                            "Repeated or backwards timestamp; frame rejected");
      continue;
    }
    try {
      if (frame->width() != static_cast<int>(width) || frame->height() != static_cast<int>(height))
        throw std::runtime_error("Frame size changed from configured mode");
      const auto stamp_seconds = frame->timestamp_ns() / 1000000000LL;
      if (stamp_seconds > std::numeric_limits<std::int32_t>::max())
        throw std::overflow_error("Source timestamp exceeds ROS2 Time range");
      const auto copied = Clock::now();
      frame->copy_to(message->data.data(), message->data.size());
      metrics.sample(metrics.copy_ms, seconds_since(copied) * 1000);
      metrics.copied_bytes += message->data.size();
      message->header.stamp.sec = static_cast<std::int32_t>(stamp_seconds);
      message->header.stamp.nanosec = static_cast<std::uint32_t>(frame->timestamp_ns() % 1000000000LL);
      const auto publishing = Clock::now();
      const auto measured = estimator.record(publishing);
      if (!publisher.publish(message, rate, measured, frame->sequence())) {
        ++metrics.frame_processing_errors;
        estimator.reset();
      } else {
        ++metrics.published;
        metrics.sample(metrics.publish_ms, seconds_since(publishing) * 1000);
        metrics.last_source_sequence = frame->sequence();
        last_stamp = frame->timestamp_ns();
      }
      if (have_deadline) next_publication += period;
      if (!have_deadline || next_publication <= publishing) next_publication = publishing + period;
      have_deadline = true;
    } catch (const std::exception& error) {
      ++metrics.frame_processing_errors;
      RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 5000,
                            "Capture frame rejected: %s", error.what());
    }
  }
  capture.stop();
  const auto report = metrics.json(capture.stats());
  write_report(metrics_output, report);
  write_report(split_metrics_output, publisher.stats_json());
  RCLCPP_INFO(node->get_logger(), "Capture stopped: %s", report.c_str());
  const auto stats = capture.stats();
  return (!metrics.published || metrics.frame_processing_errors || metrics.pool_exhausted ||
          metrics.nonmonotonic_stamps || stats.sdk_errors || stats.invalid_frames ||
          stats.format_changes) ? 1 : 0;
}
}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("capture");
  int result = 1;
  try {
    result = run(node);
  } catch (const std::exception& error) {
    RCLCPP_FATAL(node->get_logger(), "Capture failed: %s", error.what());
  }
  rclcpp::shutdown();
  return result;
}
