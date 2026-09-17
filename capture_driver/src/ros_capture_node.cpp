// Maintained ROS adapter for the DeckLink capture backend.
// The original BSD capture package's notice is retained in LICENSE.upstream.
#include "endoscope_capture/decklink_capture.hpp"
#include "endoscope_capture/capture_metrics.hpp"
#include "endoscope_capture/message_pool.hpp"
#include "endoscope_capture/stereo_splitter.hpp"
#include "endoscope_capture/publish_rate.hpp"
#include <endoscope_capture/CaptureStatus.h>

#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>
#include <thread>

namespace {
using Clock = std::chrono::steady_clock;

double seconds_since(Clock::time_point start) {
  return std::chrono::duration<double>(Clock::now() - start).count();
}


int run() {
  ros::NodeHandle nh("~");
  int device = 0, buffer_count = 4;
  double rate = 30, duration = 0;
  std::string mode, frame_id, calibration_url, metrics_file;
  nh.param("device_index", device, 0);
  nh.param("buffer_count", buffer_count, 4);
  nh.param("publish_rate", rate, 30.0);
  nh.param("duration_seconds", duration, 0.0);
  nh.param<std::string>("mode", mode, "1080p5994");
  nh.param<std::string>("frame_id", frame_id, "camera");
  nh.param<std::string>("camera_info_url", calibration_url, "");
  nh.param<std::string>("metrics_file", metrics_file, "");
  bool simulated_time = false;
  ros::param::param("/use_sim_time", simulated_time, false);
  if (simulated_time) throw std::invalid_argument("Hardware capture requires system ROS time");
  if (device < 0 || buffer_count < 2 || buffer_count > 32 || !std::isfinite(rate) ||
      rate < 0.1 || rate > 240 || !std::isfinite(duration) || duration < 0 || frame_id.empty()) {
    throw std::invalid_argument("Invalid device, buffer count (2..32), publish_rate [0.1,240], duration or frame_id");
  }
  // Check the report path before taking ownership of hardware.
  std::ofstream metrics_output;
  if (!metrics_file.empty()) {
    metrics_output.open(metrics_file, std::ios::out | std::ios::trunc);
    if (!metrics_output) throw std::runtime_error("Cannot open metrics_file");
  }
  sensor_msgs::CameraInfo base_info;
  if (!calibration_url.empty()) {
    camera_info_manager::CameraInfoManager manager(nh, "capture");
    if (!manager.loadCameraInfo(calibration_url)) {
      throw std::runtime_error("Cannot load the requested combined-image calibration");
    }
    base_info = manager.getCameraInfo();
  }
  endoscope_capture::SplitterConfig config;
  std::string split_metrics_file;
  if (!nh.getParam("left_calibration", config.left_calibration) ||
      !nh.getParam("right_calibration", config.right_calibration)) {
    throw std::invalid_argument("Set left_calibration and right_calibration");
  }
  nh.param<std::string>("split_metrics_file", split_metrics_file, "");
  nh.param("scale_x", config.scale_x, 0.5);
  nh.param("scale_y", config.scale_y, 1.0);
  nh.param("opencv_threads", config.opencv_threads, 1);
  config.buffer_count = static_cast<std::size_t>(buffer_count);
  std::ofstream split_metrics;
  if (!split_metrics_file.empty()) {
    if (split_metrics_file == metrics_file)
      throw std::invalid_argument("Capture and splitter metrics require different files");
    split_metrics.open(split_metrics_file, std::ios::out | std::ios::trunc);
    if (!split_metrics) throw std::runtime_error("Cannot open split_metrics_file");
  }
  endoscope_capture::StereoSplitter splitter(ros::NodeHandle(), config);
  endoscope_capture::DeckLinkCapture capture;
  capture.start(static_cast<unsigned>(device), mode);
  const auto width = static_cast<std::uint32_t>(capture.width());
  const auto height = static_cast<std::uint32_t>(capture.height());
  splitter.validate_input_size(width, height);
  if (width > std::numeric_limits<std::uint32_t>::max() / 4 ||
      static_cast<std::size_t>(width) * 4 > std::numeric_limits<std::size_t>::max() / height) {
    throw std::runtime_error("Capture dimensions overflow ROS image storage");
  }
  if (!calibration_url.empty() && (base_info.width != width || base_info.height != height)) {
    throw std::runtime_error("Combined calibration dimensions differ from the selected capture mode");
  }
  base_info.width = width;
  base_info.height = height;
  // Combined-image calibration is supplied through camera_info_url.
  // An empty URL yields the standard uncalibrated CameraInfo (K[0] == 0).
  endoscope_capture::MessagePool<sensor_msgs::ImagePtr> pool(
      static_cast<std::size_t>(buffer_count), [&] {
        sensor_msgs::ImagePtr image(new sensor_msgs::Image);
        image->width = width;
        image->height = height;
        image->encoding = "bgra8";
        image->is_bigendian = false;
        image->step = width * 4;
        image->header.frame_id = frame_id;
        image->data.resize(static_cast<std::size_t>(image->step) * height);
        return image;
      });
  image_transport::ImageTransport transport(nh);
  auto publisher = transport.advertiseCamera("image_raw", 1);
  auto status_publisher = nh.advertise<endoscope_capture::CaptureStatus>("status", 1);
  auto diagnostics_publisher = nh.advertise<std_msgs::String>("diagnostics", 1);
  ros::AsyncSpinner spinner(1);
  spinner.start();
  endoscope_capture::PublishRateEstimator rate_estimator(rate);
  const auto period = std::chrono::duration_cast<Clock::duration>(
      std::chrono::duration<double>(1.0 / rate));
  auto next_publication = Clock::now();
  bool have_publication_deadline = false;
  endoscope_capture::CaptureMetrics metrics;
  auto diagnostics_due = Clock::now();
  std::int64_t last_stamp = 0;
  std::uint64_t previous_no_signal = 0;
  ROS_INFO_STREAM("Capturing " << width << "x" << height << " BGRA at up to " << rate
                  << " Hz; timestamps are software SDK callback arrival times");
  while (ros::ok() && (duration == 0 || seconds_since(metrics.started) < duration)) {
    if (Clock::now() >= diagnostics_due) {
      std_msgs::String diagnostics;
      diagnostics.data = metrics.json(capture.stats());
      diagnostics_publisher.publish(diagnostics);
      diagnostics_due = Clock::now() + std::chrono::seconds(1);
    }
    const auto now = Clock::now();
    if (now < next_publication) {
      // Bounded sleeps keep shutdown, duration limits and the heartbeat
      // responsive even at low configured rates. Missed deadlines never cause
      // catch-up bursts after a missed interval.
      std::this_thread::sleep_for(std::min(
          next_publication - now,
          std::chrono::duration_cast<Clock::duration>(std::chrono::milliseconds(20))));
      continue;
    }
    auto frame = capture.wait_for_frame(std::chrono::milliseconds(250));
    const auto capture_state = capture.stats();
    if (capture_state.no_signal != previous_no_signal) {
      rate_estimator.reset();
      have_publication_deadline = false;
    }
    previous_no_signal = capture_state.no_signal;
    if (capture_state.format_changes) {
      ROS_ERROR("Input format changed; explicit reconfiguration and restart required");
      break;
    }
    if (frame) {
      auto message = pool.acquire();
      if (!message) {
        ++metrics.pool_exhausted;
        ROS_WARN_THROTTLE(5, "Capture output buffers retained; frame dropped, memory stays bounded");
      } else if (frame->timestamp_ns() <= last_stamp) {
        ++metrics.nonmonotonic_stamps;
        ROS_ERROR_THROTTLE(5, "System clock moved backwards or repeated; frame rejected");
      } else {
        try {
          if (frame->width() != static_cast<int>(width) || frame->height() != static_cast<int>(height)) {
            throw std::runtime_error("Frame size changed from configured mode");
          }
          const auto copy_started = Clock::now();
          frame->copy_to(message->data.data(), message->data.size());
          metrics.sample(metrics.copy_ms, seconds_since(copy_started) * 1000);
          metrics.copied_bytes += message->data.size();
          message->header.stamp.fromNSec(static_cast<std::uint64_t>(frame->timestamp_ns()));
          // A direct splitter sees the object before transport serialization.
          // Give it the publication sequence, separate from SDK selection.
          message->header.seq = static_cast<std::uint32_t>(metrics.published);
          sensor_msgs::CameraInfoPtr info(new sensor_msgs::CameraInfo(base_info));
          info->header = message->header;
          const auto publish_started = Clock::now();
          // Preserve shared ownership. Neither message nor its bytes are modified
          // until all consumers have released it and the pool makes it available.
          publisher.publish(message, info);
          if (have_publication_deadline) next_publication += period;
          if (!have_publication_deadline || next_publication <= publish_started)
            next_publication = publish_started + period;
          have_publication_deadline = true;
          const auto measured = rate_estimator.record(publish_started);
          endoscope_capture::CaptureStatus status;
          status.header = message->header;
          status.configured_publish_hz = rate;
          status.measured_publish_hz = measured.hz;
          status.measured_rate_valid = measured.valid;
          status.rate_window_samples = measured.samples;
          status.rate_window_seconds = measured.window_seconds;
          status.source_sequence = frame->sequence();
          status_publisher.publish(status);
          if (!splitter.process(message)) {
            ++metrics.frame_processing_errors;
          }
          metrics.sample(metrics.publish_ms, seconds_since(publish_started) * 1000);
          last_stamp = frame->timestamp_ns();
          metrics.last_source_sequence = frame->sequence();
          ++metrics.published;
        } catch (const std::exception& error) {
          ++metrics.frame_processing_errors;
          ROS_ERROR_STREAM_THROTTLE(5, "Capture frame rejected: " << error.what());
        }
      }
    } else {
      rate_estimator.reset();
      have_publication_deadline = false;
      ROS_WARN_THROTTLE(5, "Waiting for valid capture input; no stale frame is republished");
    }
    frame.reset();
  }
  capture.stop();
  const std::string report = metrics.json(capture.stats());
  if (metrics_output.is_open()) {
    metrics_output << report << '\n';
    metrics_output.flush();
    if (!metrics_output) throw std::runtime_error("Failed writing capture metrics");
  }
  if (split_metrics.is_open()) {
    split_metrics << splitter.stats_json() << '\n';
    split_metrics.flush();
    if (!split_metrics) throw std::runtime_error("Failed writing splitter metrics");
  }
  ROS_INFO_STREAM("Capture stopped: " << report);
  spinner.stop();
  return (!metrics.published || metrics.frame_processing_errors || metrics.pool_exhausted ||
          metrics.nonmonotonic_stamps || capture.stats().sdk_errors || capture.stats().invalid_frames ||
          capture.stats().format_changes) ? 1 : 0;
}
}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "endoscope_capture");
  // Keep ROS logging alive while run() unwinds after a startup failure.
  ros::NodeHandle lifecycle;
  try {
    return run();
  } catch (const std::exception& error) {
    ROS_FATAL_STREAM("Capture failed: " << error.what());
    return 1;
  }
}
