#pragma once

#include "endoscope_capture/stereo_processor.hpp"

#include <ros/node_handle.h>

#include <cstdint>
#include <memory>
#include <string>

namespace endoscope_capture {

// Creates publishers beneath output_namespace, then accepts input either from
// a ROS callback or directly from a capture process. Public methods serialize
// on one mutex. Construct all instances before processing begins:
// OpenCV's thread-count setting is process-wide.
class StereoSplitter {
 public:
  StereoSplitter(ros::NodeHandle output_namespace, const SplitterConfig& config);
  ~StereoSplitter();
  StereoSplitter(const StereoSplitter&) = delete;
  StereoSplitter& operator=(const StereoSplitter&) = delete;

  // Rejections and exhausted pools return false and are reported in metrics.
  bool process(const sensor_msgs::ImageConstPtr& source);
  void validate_input_size(std::uint32_t width, std::uint32_t height) const;
  std::string stats_json() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace endoscope_capture
