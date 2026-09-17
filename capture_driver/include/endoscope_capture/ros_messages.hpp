#pragma once

#ifdef CAPTURE_ROS2
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#else
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#endif

namespace endoscope_capture {

#ifdef CAPTURE_ROS2
using Image = sensor_msgs::msg::Image;
using ImagePtr = Image::SharedPtr;
using ImageConstPtr = Image::ConstSharedPtr;
using CameraInfo = sensor_msgs::msg::CameraInfo;
using CameraInfoPtr = CameraInfo::SharedPtr;

inline auto& calibration_d(CameraInfo& info) { return info.d; }
inline const auto& calibration_d(const CameraInfo& info) { return info.d; }
inline auto& calibration_k(CameraInfo& info) { return info.k; }
inline const auto& calibration_k(const CameraInfo& info) { return info.k; }
inline auto& calibration_r(CameraInfo& info) { return info.r; }
inline const auto& calibration_r(const CameraInfo& info) { return info.r; }
inline auto& calibration_p(CameraInfo& info) { return info.p; }
inline const auto& calibration_p(const CameraInfo& info) { return info.p; }
#else
using Image = sensor_msgs::Image;
using ImagePtr = sensor_msgs::ImagePtr;
using ImageConstPtr = sensor_msgs::ImageConstPtr;
using CameraInfo = sensor_msgs::CameraInfo;
using CameraInfoPtr = sensor_msgs::CameraInfoPtr;

inline auto& calibration_d(CameraInfo& info) { return info.D; }
inline const auto& calibration_d(const CameraInfo& info) { return info.D; }
inline auto& calibration_k(CameraInfo& info) { return info.K; }
inline const auto& calibration_k(const CameraInfo& info) { return info.K; }
inline auto& calibration_r(CameraInfo& info) { return info.R; }
inline const auto& calibration_r(const CameraInfo& info) { return info.R; }
inline auto& calibration_p(CameraInfo& info) { return info.P; }
inline const auto& calibration_p(const CameraInfo& info) { return info.P; }
#endif

inline void clear_calibration_region(CameraInfo& info) {
  info.binning_x = 0;
  info.binning_y = 0;
  info.roi = decltype(info.roi){};
}

}  // namespace endoscope_capture
