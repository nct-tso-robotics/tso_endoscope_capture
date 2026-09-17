#pragma once

#include "endoscope_capture/publish_rate.hpp"
#include "endoscope_capture/stereo_processor.hpp"

#include <rclcpp/rclcpp.hpp>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

namespace endoscope_capture {

// One publishing thread supplies combined frames. The adapter shares the stereo
// processor with ROS1 and serializes directly from its pooled image storage.
class Ros2Publisher {
 public:
  Ros2Publisher(rclcpp::Node& node, const SplitterConfig& config,
                const CameraInfo& combined_calibration,
                const std::string& reliability = "reliable",
                std::size_t qos_depth = 4);
  ~Ros2Publisher();
  Ros2Publisher(const Ros2Publisher&) = delete;
  Ros2Publisher& operator=(const Ros2Publisher&) = delete;

  void validate_input_size(std::uint32_t width, std::uint32_t height) const;
  bool publish(const ImageConstPtr& source, double configured_hz,
               const PublishRateEstimator::Sample& measured,
               std::uint64_t source_sequence);
  void diagnostics(const std::string& report);
  std::string stats_json() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace endoscope_capture
