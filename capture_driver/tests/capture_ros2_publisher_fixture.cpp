// Synthetic integration fixture for the production ROS2 publisher.
#include "endoscope_capture/ros2_publisher.hpp"

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <cstdint>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {
using Clock = std::chrono::steady_clock;
constexpr std::uint64_t kFrameRate = 30;
constexpr std::uint64_t kFrameCount = 60;

int run(const std::shared_ptr<rclcpp::Node>& node) {
  endoscope_capture::SplitterConfig config;
  config.left_calibration = node->declare_parameter<std::string>("left_calibration", "");
  config.right_calibration = node->declare_parameter<std::string>("right_calibration", "");
  config.scale_x = 0.5;
  config.scale_y = 0.5;
  config.opencv_threads = 1;
  config.buffer_count = 4;
  const auto reliability = node->declare_parameter<std::string>("reliability", "reliable");
  const auto qos_depth = node->declare_parameter<std::int64_t>("qos_depth", 4);
  endoscope_capture::CameraInfo combined_info;
  endoscope_capture::Ros2Publisher publisher(
      *node, config, combined_info, reliability, static_cast<std::size_t>(qos_depth));
  publisher.validate_input_size(8, 8);
  endoscope_capture::PublishRateEstimator rate(kFrameRate);

  endoscope_capture::Image pattern;
  pattern.width = pattern.height = 8;
  pattern.encoding = "bgra8";
  pattern.step = pattern.width * 4 + 8;
  pattern.data.assign(pattern.step * pattern.height, 238);
  pattern.header.frame_id = "synthetic_camera";
  for (std::uint32_t row = 0; row < pattern.height; ++row) {
    for (std::uint32_t column = 0; column < pattern.width; ++column) {
      const auto offset = row * pattern.step + column * 4;
      const auto base = (row % 2 ? 110 : 10) + (row / 2) * 4 + (column % 2) * 40;
      pattern.data[offset] = static_cast<std::uint8_t>(base);
      pattern.data[offset + 1] = static_cast<std::uint8_t>(base + 20);
      pattern.data[offset + 2] = static_cast<std::uint8_t>(base + 40);
      pattern.data[offset + 3] = static_cast<std::uint8_t>((column * 31 + row * 17) % 256);
    }
  }

  const std::string prefix = node->get_namespace();
  const std::string private_prefix = node->get_fully_qualified_name();
  const std::vector<std::string> topics = {
      private_prefix + "/image_raw", private_prefix + "/camera_info",
      private_prefix + "/status", private_prefix + "/diagnostics",
      prefix + "/left/image_raw", prefix + "/left/camera_info",
      prefix + "/right/image_raw", prefix + "/right/camera_info"};
  const auto started = Clock::now();
  auto ready = Clock::time_point{};
  auto due = started;
  auto diagnostic_due = started;
  auto no_input_since = Clock::time_point{};
  std::uint64_t frames = 0;
  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    const auto now = Clock::now();
    if (now - started > std::chrono::seconds(20))
      throw std::runtime_error("Synthetic ROS2 publisher timed out");
    if (ready == Clock::time_point{}) {
      bool connected = true;
      for (const auto& topic : topics)
        connected = connected && node->count_subscribers(topic) > 0;
      if (connected) ready = now + std::chrono::milliseconds(300);
    }
    if (ready != Clock::time_point{} && now >= ready && now >= due && frames < kFrameCount) {
      auto image = std::make_shared<endoscope_capture::Image>(pattern);
      image->header.stamp.sec = static_cast<std::int32_t>(200 + frames / kFrameRate);
      image->header.stamp.nanosec =
          static_cast<std::uint32_t>((frames % kFrameRate) * 1000000000 / kFrameRate + 12345);
      if (!publisher.publish(image, kFrameRate, rate.record(now), frames + 1))
        throw std::runtime_error("Production ROS2 publisher rejected synthetic input");
      ++frames;
      due = now + std::chrono::nanoseconds(1000000000 / kFrameRate);
      if (frames == kFrameCount) {
        no_input_since = now;
        rate.reset();
      }
    }
    if (now >= diagnostic_due) {
      const char* phase = frames == kFrameCount ? "no_input" : frames ? "frames" : "waiting";
      publisher.diagnostics(std::string("{\"phase\":\"") + phase +
                            "\",\"publisher\":" + publisher.stats_json() + "}");
      diagnostic_due = now + std::chrono::milliseconds(100);
    }
    if (frames == kFrameCount && now - no_input_since >= std::chrono::seconds(1)) {
      publisher.diagnostics("{\"phase\":\"complete\",\"publisher\":" +
                            publisher.stats_json() + "}");
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      std::cout << publisher.stats_json() << '\n';
      return 0;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  return 1;
}
}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int result = 1;
  try {
    auto node = std::make_shared<rclcpp::Node>("capture_fixture");
    result = run(node);
  } catch (const std::exception& error) {
    std::cerr << "ROS2 publisher fixture failed: " << error.what() << '\n';
  }
  rclcpp::shutdown();
  return result;
}
