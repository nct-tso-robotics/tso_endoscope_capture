#include "endoscope_capture/stereo_processor.hpp"
#include "endoscope_capture/message_pool.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace endoscope_capture {
namespace {

void validate_scale(double scale) {
  if (!std::isfinite(scale) || scale <= 0 || scale > 1)
    throw std::invalid_argument("Scale factors must be finite and in (0, 1]");
}

std::uint32_t output_dimension(std::uint32_t input, double scale) {
  validate_scale(scale);
  const double value = input * scale;
  double integral = std::floor(value);
  const double fraction = value - integral;
  // Round half-integer sizes to the nearest even integer, independently
  // of the floating-point environment.
  if (fraction > 0.5 || (fraction == 0.5 && std::fmod(integral, 2.0) != 0))
    integral += 1;
  if (integral < 1 || integral > std::numeric_limits<int>::max())
    throw std::invalid_argument("Scale factors produce an invalid output dimension");
  return static_cast<std::uint32_t>(integral);
}

template <class Destination>
void load_matrix(const YAML::Node& root, const char* name, int rows, int columns,
                 Destination& destination) {
  const auto matrix = root[name];
  if (!matrix.IsMap() || matrix["rows"].as<int>() != rows ||
      matrix["cols"].as<int>() != columns)
    throw std::invalid_argument(std::string("Invalid calibration matrix shape: ") + name);
  const auto values = matrix["data"];
  if (!values.IsSequence() || values.size() != static_cast<std::size_t>(rows * columns) ||
      destination.size() != values.size())
    throw std::invalid_argument(std::string("Invalid calibration matrix length: ") + name);
  for (std::size_t i = 0; i < values.size(); ++i) {
    const double value = values[i].as<double>();
    if (!std::isfinite(value))
      throw std::invalid_argument(std::string("Nonfinite calibration matrix: ") + name);
    destination[i] = value;
  }
}

CameraInfo load_calibration(const std::string& path) {
  if (path.empty()) throw std::invalid_argument("Both calibration files are required");
  const auto root = YAML::LoadFile(path);
  if (!root.IsMap()) throw std::invalid_argument("Calibration must be a YAML mapping");
  const auto width = root["image_width"].as<std::int64_t>();
  const auto height = root["image_height"].as<std::int64_t>();
  if (width <= 0 || height <= 0 || width > std::numeric_limits<int>::max() ||
      height > std::numeric_limits<int>::max() / 2)
    throw std::invalid_argument("Calibration dimensions must be positive supported integers");
  CameraInfo info;
  info.width = static_cast<std::uint32_t>(width);
  info.height = static_cast<std::uint32_t>(height);
  info.distortion_model = root["distortion_model"].as<std::string>();
  if (info.distortion_model != "plumb_bob")
    throw std::invalid_argument("Expected plumb_bob calibration");
  calibration_d(info).resize(5);
  load_matrix(root, "distortion_coefficients", 1, 5, calibration_d(info));
  load_matrix(root, "camera_matrix", 3, 3, calibration_k(info));
  load_matrix(root, "rectification_matrix", 3, 3, calibration_r(info));
  load_matrix(root, "projection_matrix", 3, 4, calibration_p(info));
  const auto& k = calibration_k(info);
  const auto& p = calibration_p(info);
  if (std::min({k[0], k[4], p[0], p[5]}) <= 0)
    throw std::invalid_argument("Calibration focal lengths must be positive");
  return info;
}

CameraInfo scale_calibration(CameraInfo info, std::uint32_t width,
                             std::uint32_t height) {
  const double sx = static_cast<double>(width) / info.width;
  const double sy = static_cast<double>(height) / info.height;
  auto& k = calibration_k(info);
  auto& p = calibration_p(info);
  for (std::size_t i = 0; i < 3; ++i) k[i] *= sx;
  for (std::size_t i = 3; i < 6; ++i) k[i] *= sy;
  for (std::size_t i = 0; i < 4; ++i) p[i] *= sx;
  for (std::size_t i = 4; i < 8; ++i) p[i] *= sy;
  info.width = width;
  info.height = height;
  clear_calibration_region(info);
  return info;
}

std::size_t image_bytes(std::uint32_t width, std::uint32_t height) {
  if (width > std::numeric_limits<std::uint32_t>::max() / 3 ||
      static_cast<std::size_t>(width) * 3 >
          std::numeric_limits<std::size_t>::max() / height)
    throw std::invalid_argument("Output dimensions overflow image storage");
  return static_cast<std::size_t>(width) * height * 3;
}

}  // namespace

struct StereoProcessor::Impl {
  explicit Impl(const SplitterConfig& config)
      : original_left(load_calibration(config.left_calibration)),
        original_right(load_calibration(config.right_calibration)) {
    if (config.buffer_count < 1 || config.buffer_count > 32 || config.opencv_threads < 1)
      throw std::invalid_argument("Buffer count must be 1..32 and OpenCV threads positive");
    if (original_left.width != original_right.width ||
        original_left.height != original_right.height)
      throw std::invalid_argument("Left and right calibration dimensions differ");
    width = output_dimension(original_left.width, config.scale_x);
    height = output_dimension(original_left.height, config.scale_y);
    const auto bytes = image_bytes(width, height);
    const auto scratch_bytes = image_bytes(original_left.width, original_left.height);
    constexpr std::size_t memory_limit = 512UL * 1024 * 1024;
    // Bound all image pools plus the two BGRA-to-BGR scratch images before any
    // large allocations. CameraInfo/header overhead is small and also bounded.
    if (scratch_bytes > memory_limit / 2 ||
        bytes > (memory_limit - scratch_bytes * 2) / (config.buffer_count * 2))
      throw std::invalid_argument("Configured splitter image buffers exceed 512 MiB");
    const auto output_left = scale_calibration(original_left, width, height);
    const auto output_right = scale_calibration(original_right, width, height);
    const auto image_factory = [this, bytes] {
      ImagePtr image(new Image);
      image->width = width;
      image->height = height;
      image->encoding = "bgr8";
      image->is_bigendian = false;
      image->step = width * 3;
      image->data.resize(bytes);
      return image;
    };
    left.reset(new MessagePool<ImagePtr>(config.buffer_count, image_factory));
    right.reset(new MessagePool<ImagePtr>(config.buffer_count, image_factory));
    left_info.reset(new MessagePool<CameraInfoPtr>(
        config.buffer_count, [&output_left] {
          return CameraInfoPtr(new CameraInfo(output_left));
        }));
    right_info.reset(new MessagePool<CameraInfoPtr>(
        config.buffer_count, [&output_right] {
          return CameraInfoPtr(new CameraInfo(output_right));
        }));
    cv::setNumThreads(config.opencv_threads);
  }

  CameraInfo original_left;
  CameraInfo original_right;
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  std::unique_ptr<MessagePool<ImagePtr>> left;
  std::unique_ptr<MessagePool<ImagePtr>> right;
  std::unique_ptr<MessagePool<CameraInfoPtr>> left_info;
  std::unique_ptr<MessagePool<CameraInfoPtr>> right_info;
  cv::Mat scratch_left;
  cv::Mat scratch_right;

  void process_eye(const cv::Mat& source, bool bgra, Image& output, cv::Mat& scratch) {
    cv::Mat destination(static_cast<int>(height), static_cast<int>(width), CV_8UC3,
                         output.data.data(), output.step);
    const bool resize = source.cols != destination.cols || source.rows != destination.rows;
    if (bgra) {
      if (resize) {
        cv::cvtColor(source, scratch, cv::COLOR_BGRA2BGR);
        cv::resize(scratch, destination, destination.size(), 0, 0, cv::INTER_AREA);
      } else {
        cv::cvtColor(source, destination, cv::COLOR_BGRA2BGR);
      }
    } else if (resize) {
      cv::resize(source, destination, destination.size(), 0, 0, cv::INTER_AREA);
    } else {
      source.copyTo(destination);
    }
  }
};

StereoProcessor::StereoProcessor(const SplitterConfig& config) : impl_(new Impl(config)) {}
StereoProcessor::~StereoProcessor() = default;

void StereoProcessor::validate_input_size(std::uint32_t width, std::uint32_t height) const {
  if (width != impl_->original_left.width || height != impl_->original_left.height * 2)
    throw std::invalid_argument(
        "Input dimensions " + std::to_string(width) + "x" + std::to_string(height) +
        " do not match per-eye calibration " + std::to_string(impl_->original_left.width) +
        "x" + std::to_string(impl_->original_left.height));
}

bool StereoProcessor::process(const ImageConstPtr& source, StereoPair& output) {
  output = StereoPair{};
  if (!source) throw std::invalid_argument("Null source image");
  validate_input_size(source->width, source->height);
  const bool bgra = source->encoding == "bgra8";
  if (!bgra && source->encoding != "bgr8")
    throw std::invalid_argument("Supported encodings are bgr8 and bgra8");
  const auto channels = bgra ? 4U : 3U;
  if (static_cast<std::size_t>(source->width) * channels > source->step ||
      source->step > std::numeric_limits<std::size_t>::max() / source->height ||
      source->data.size() != static_cast<std::size_t>(source->step) * source->height)
    throw std::invalid_argument("Input data length or row stride is invalid");
  StereoPair pair;
  pair.left = impl_->left->acquire();
  pair.right = impl_->right->acquire();
  pair.left_info = impl_->left_info->acquire();
  pair.right_info = impl_->right_info->acquire();
  if (!pair.left || !pair.right || !pair.left_info || !pair.right_info) return false;

  // Each OpenCV header views only one eye. Its stride skips the other eye's
  // row, so color conversion and area filtering never mix the two views.
  auto* bytes = const_cast<std::uint8_t*>(source->data.data());
  const auto eye_stride = static_cast<std::size_t>(source->step) * 2;
  const int type = bgra ? CV_8UC4 : CV_8UC3;
  cv::Mat left(static_cast<int>(impl_->original_left.height),
               static_cast<int>(source->width), type, bytes + source->step, eye_stride);
  cv::Mat right(static_cast<int>(impl_->original_left.height),
                static_cast<int>(source->width), type, bytes, eye_stride);
  impl_->process_eye(left, bgra, *pair.left, impl_->scratch_left);
  impl_->process_eye(right, bgra, *pair.right, impl_->scratch_right);
  pair.left->header = source->header;
  pair.right->header = source->header;
  pair.left_info->header = source->header;
  pair.right_info->header = source->header;
  output = std::move(pair);
  return true;
}

std::uint32_t StereoProcessor::output_width() const noexcept { return impl_->width; }
std::uint32_t StereoProcessor::output_height() const noexcept { return impl_->height; }

}  // namespace endoscope_capture
