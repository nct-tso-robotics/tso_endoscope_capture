#pragma once

#include "endoscope_capture/ros_messages.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

namespace endoscope_capture {

struct SplitterConfig {
  std::string left_calibration;
  std::string right_calibration;
  double scale_x = 0.5;
  double scale_y = 1.0;
  std::size_t buffer_count = 4;
  int opencv_threads = 1;
};

struct StereoPair {
  ImagePtr left;
  ImagePtr right;
  CameraInfoPtr left_info;
  CameraInfoPtr right_info;
};

// Processes ROS messages without a node, subscriptions, or publication.
// One thread calls process(). Output buffers remain immutable while retained by
// the caller, publisher, or subscriber. Retaining every slot causes a bounded
// drop. Construct all instances before processing begins: OpenCV's thread-count
// setting is process-wide.
class StereoProcessor {
 public:
  explicit StereoProcessor(const SplitterConfig& config);
  ~StereoProcessor();
  StereoProcessor(const StereoProcessor&) = delete;
  StereoProcessor& operator=(const StereoProcessor&) = delete;

  // Throws on malformed input, clears output, and leaves source unchanged.
  // Output pixels are BGR8; zero-based odd rows are left, even rows are right.
  bool process(const ImageConstPtr& source, StereoPair& output);
  // Validate combined input geometry against the full-resolution calibration.
  // Throws on mismatch; usable before receiving or publishing any frames.
  void validate_input_size(std::uint32_t width, std::uint32_t height) const;
  std::uint32_t output_width() const noexcept;
  std::uint32_t output_height() const noexcept;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace endoscope_capture
