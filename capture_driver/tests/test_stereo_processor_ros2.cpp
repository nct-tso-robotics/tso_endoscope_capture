#include "endoscope_capture/stereo_processor.hpp"

#ifndef CAPTURE_ROS2
#error "This test requires ROS2 message types"
#endif

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <unistd.h>

namespace {
using namespace endoscope_capture;

void require(bool condition, const char* expression, int line) {
  if (!condition)
    throw std::runtime_error(std::string("line ") + std::to_string(line) + ": " + expression);
}
#define CHECK(expression) require(static_cast<bool>(expression), #expression, __LINE__)

template <class Function>
void must_throw(Function&& function) {
  bool threw = false;
  try {
    function();
  } catch (const std::exception&) {
    threw = true;
  }
  CHECK(threw);
}

void near(double actual, double expected) {
  CHECK(std::isfinite(actual));
  CHECK(std::abs(actual - expected) < 1e-11);
}

class CalibrationFiles {
 public:
  explicit CalibrationFiles(unsigned width = 6, unsigned height = 3) {
    char name[] = "/tmp/stereo-processor-ros2-tests-XXXXXX";
    const char* created = ::mkdtemp(name);
    if (!created) throw std::runtime_error("Cannot create calibration test directory");
    directory_ = created;
    config.left_calibration = directory_ + "/left.yaml";
    config.right_calibration = directory_ + "/right.yaml";
    config.scale_x = 1;
    config.scale_y = 1;
    for (const bool right : {false, true}) {
      std::ofstream file(right ? config.right_calibration : config.left_calibration);
      file << "image_width: " << width << "\nimage_height: " << height
           << "\ndistortion_model: plumb_bob\n"
           << "camera_matrix:\n  rows: 3\n  cols: 3\n"
           << "  data: [120, 3, 2.5, 4, 90, 1.5, 0, 0, 1]\n"
           << "distortion_coefficients:\n  rows: 1\n  cols: 5\n"
           << "  data: [-0.1, 0.01, 0.001, 0.002, 0.0001]\n"
           << "rectification_matrix:\n  rows: 3\n  cols: 3\n"
           << "  data: [1, 0, 0, 0, 1, 0, 0, 0, 1]\n"
           << "projection_matrix:\n  rows: 3\n  cols: 4\n"
           << "  data: [130, 5, 2, " << (right ? -52 : -26)
           << ", 6, 100, 1, -50, 0, 0, 1, 0]\n";
      file.close();
      if (!file) throw std::runtime_error("Cannot write test calibration");
    }
  }
  ~CalibrationFiles() {
    std::remove(config.left_calibration.c_str());
    std::remove(config.right_calibration.c_str());
    ::rmdir(directory_.c_str());
  }
  CalibrationFiles(const CalibrationFiles&) = delete;
  CalibrationFiles& operator=(const CalibrationFiles&) = delete;
  SplitterConfig config;

 private:
  std::string directory_;
};

ImagePtr input(unsigned channels, unsigned width = 6, unsigned eye_height = 3) {
  auto image = std::make_shared<Image>();
  image->header.frame_id = "test_optical_frame";
  image->header.stamp.sec = -7;
  image->header.stamp.nanosec = 123456789;
  image->width = width;
  image->height = eye_height * 2;
  image->encoding = channels == 4 ? "bgra8" : "bgr8";
  image->step = width * channels + 11;
  image->data.assign(image->step * image->height, 251);
  return image;
}

void pixel(Image& image, unsigned row, unsigned column, std::uint8_t value) {
  const auto channels = image.encoding == "bgra8" ? 4U : 3U;
  const auto offset = static_cast<std::size_t>(row) * image.step + column * channels;
  for (unsigned channel = 0; channel < 3; ++channel)
    image.data[offset + channel] = value + channel;
  if (channels == 4) image.data[offset + 3] = 17;
}

void check_pixel(const Image& image, unsigned column, std::uint8_t value) {
  for (unsigned channel = 0; channel < 3; ++channel)
    CHECK(image.data.at(column * 3 + channel) == value + channel);
}

void check_geometry(const StereoPair& pair, const Image& source,
                    unsigned width, unsigned height) {
  for (const auto& image : {pair.left, pair.right}) {
    CHECK(image->width == width && image->height == height);
    CHECK(image->step == width * 3 && image->encoding == "bgr8");
    CHECK(image->data.size() == static_cast<std::size_t>(width) * height * 3);
    CHECK(image->header == source.header);
  }
  for (const auto& info : {pair.left_info, pair.right_info}) {
    CHECK(info->width == width && info->height == height);
    CHECK(info->header == source.header);
    CHECK(info->binning_x == 0 && info->binning_y == 0);
    CHECK(info->roi.width == 0 && info->roi.height == 0 && !info->roi.do_rectify);
  }
}

void ros2_field_accessors_refer_to_message_storage() {
  CameraInfo info;
  calibration_d(info) = {-0.1, 0.01, 0.001, 0.002, 0.0001};
  calibration_k(info)[0] = 120;
  calibration_r(info)[0] = 1;
  calibration_p(info)[3] = -52;
  const CameraInfo& saved = info;
  CHECK(&calibration_d(saved) == &info.d);
  CHECK(&calibration_k(saved) == &info.k);
  CHECK(&calibration_r(saved) == &info.r);
  CHECK(&calibration_p(saved) == &info.p);
  near(info.d[0], -0.1);
  near(info.k[0], 120);
  near(info.r[0], 1);
  near(info.p[3], -52);
  info.binning_x = 2;
  info.binning_y = 3;
  info.roi.x_offset = 1;
  info.roi.y_offset = 2;
  info.roi.width = 40;
  info.roi.height = 30;
  info.roi.do_rectify = true;
  clear_calibration_region(info);
  CHECK(info.binning_x == 0 && info.binning_y == 0);
  CHECK(info.roi.x_offset == 0 && info.roi.y_offset == 0);
  CHECK(info.roi.width == 0 && info.roi.height == 0 && !info.roi.do_rectify);
}

void padded_images_filter_separated_eyes_and_preserve_headers() {
  CalibrationFiles files;
  files.config.scale_x = 1.0 / 3;
  files.config.scale_y = 1.0 / 3;
  StereoProcessor processor(files.config);
  const unsigned columns[] = {0, 60, 0, 30, 0, 60};
  const unsigned rows[] = {0, 60, 0};
  for (const unsigned channels : {3U, 4U}) {
    auto source = input(channels);
    for (unsigned row = 0; row < source->height; ++row)
      for (unsigned col = 0; col < source->width; ++col)
        pixel(*source, row, col, static_cast<std::uint8_t>(
            columns[col] + rows[row / 2] + (row % 2 ? 100 : 0)));
    const auto bytes = source->data;
    for (const bool zero_timestamp : {false, true}) {
      if (zero_timestamp) source->header.stamp = decltype(source->header.stamp){};
      StereoPair output;
      CHECK(processor.process(source, output));
      check_geometry(output, *source, 2, 1);
      check_pixel(*output.right, 0, 40);
      check_pixel(*output.right, 1, 50);
      check_pixel(*output.left, 0, 140);
      check_pixel(*output.left, 1, 150);
      CHECK(source->data == bytes);
    }
  }
}

void rounded_output_scales_ros2_calibration() {
  CalibrationFiles files(5, 3);
  files.config.scale_x = 0.5;
  files.config.scale_y = 0.5;
  StereoProcessor processor(files.config);
  const auto source = input(4, 5, 3);
  StereoPair output;
  CHECK(processor.process(source, output));
  check_geometry(output, *source, 2, 2);
  const std::array<double, 9> k{{120, 3, 2.5, 4, 90, 1.5, 0, 0, 1}};
  const std::array<double, 12> p{{130, 5, 2, -26, 6, 100, 1, -50, 0, 0, 1, 0}};
  const std::array<double, 9> r{{1, 0, 0, 0, 1, 0, 0, 0, 1}};
  for (const auto& info : {output.left_info, output.right_info}) {
    for (unsigned i = 0; i < k.size(); ++i) {
      near(info->k[i], k[i] * (i < 3 ? 0.4 : i < 6 ? 2.0 / 3 : 1));
      near(info->r[i], r[i]);
    }
    for (unsigned i = 0; i < p.size(); ++i)
      near(info->p[i], p[i] * (i < 4 ? 0.4 : i < 8 ? 2.0 / 3 : 1) *
                           (info == output.right_info && i == 3 ? 2 : 1));
    CHECK(info->d == std::vector<double>({-0.1, 0.01, 0.001, 0.002, 0.0001}));
    CHECK(info->distortion_model == "plumb_bob");
  }
}

void shared_outputs_remain_immutable_until_released() {
  CalibrationFiles files;
  files.config.buffer_count = 1;
  StereoProcessor processor(files.config);
  auto source = input(4);
  StereoPair first;
  CHECK(processor.process(source, first));
  const auto address = first.left->data.data();
  ImageConstPtr retained_image = first.left;
  std::shared_ptr<const CameraInfo> retained_info = first.right_info;
  const auto bytes = retained_image->data;
  const auto header = retained_info->header;
  first = StereoPair{};
  source->data.assign(source->data.size(), 7);
  ++source->header.stamp.sec;
  StereoPair next;
  CHECK(!processor.process(source, next));
  CHECK(!next.left && !next.right && !next.left_info && !next.right_info);
  CHECK(retained_image->data == bytes && retained_info->header == header);
  retained_image.reset();
  CHECK(!processor.process(source, next));
  CHECK(!next.left && !next.right && !next.left_info && !next.right_info);
  retained_info.reset();
  CHECK(processor.process(source, next));
  CHECK(next.left->data.data() == address);
  CHECK(std::all_of(next.left->data.begin(), next.left->data.end(),
                    [](std::uint8_t value) { return value == 7; }));
}

void startup_geometry_and_invalid_frames_are_rejected() {
  CalibrationFiles files;
  StereoProcessor processor(files.config);
  processor.validate_input_size(6, 6);
  for (const auto& size : std::array<std::pair<std::uint32_t, std::uint32_t>, 5>{{
           {0, 6}, {6, 0}, {6, 3}, {5, 6}, {6, std::numeric_limits<std::uint32_t>::max()}}})
    must_throw([&] { processor.validate_input_size(size.first, size.second); });
  StereoPair output;
  must_throw([&] { processor.process(ImageConstPtr{}, output); });
  auto source = input(4);
  source->step = 23;
  must_throw([&] { processor.process(source, output); });
  source = input(4);
  source->data.pop_back();
  must_throw([&] { processor.process(source, output); });
  source = input(4);
  CHECK(processor.process(source, output));
}
}  // namespace

int main() {
  const std::pair<const char*, void (*)()> tests[] = {
      {"ROS2 calibration accessors", ros2_field_accessors_refer_to_message_storage},
      {"padded BGR/BGRA area pixels and headers", padded_images_filter_separated_eyes_and_preserve_headers},
      {"ROS2 rounded calibration and translation", rounded_output_scales_ros2_calibration},
      {"std::shared_ptr output retention", shared_outputs_remain_immutable_until_released},
      {"startup geometry and invalid frames", startup_geometry_and_invalid_frames_are_rejected},
  };
  unsigned failures = 0;
  for (const auto& test : tests) {
    try {
      test.second();
      std::cout << "PASS " << test.first << '\n';
    } catch (const std::exception& error) {
      ++failures;
      std::cerr << "FAIL " << test.first << ": " << error.what() << '\n';
    }
  }
  return failures ? 1 : 0;
}
