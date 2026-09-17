#include "endoscope_capture/stereo_splitter.hpp"

#include <sensor_msgs/Image.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <unistd.h>

namespace {
using endoscope_capture::SplitterConfig;
using endoscope_capture::StereoPair;
using endoscope_capture::StereoProcessor;

void require(bool condition, const char* expression, int line) {
  if (!condition)
    throw std::runtime_error(std::string("line ") + std::to_string(line) +
                             ": " + expression);
}
#define CHECK(expression) require(static_cast<bool>(expression), #expression, __LINE__)

template <typename Function> void must_throw(Function&& function) {
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

std::string calibration(unsigned width, unsigned height, bool right = false) {
  return "image_width: " + std::to_string(width) +
         "\nimage_height: " + std::to_string(height) +
         "\ncamera_name: test_camera\n"
         "camera_matrix:\n  rows: 3\n  cols: 3\n"
         "  data: [120, 3, 2.5, 4, 90, 1.5, 0, 0, 1]\n"
         "distortion_model: plumb_bob\n"
         "distortion_coefficients:\n  rows: 1\n  cols: 5\n"
         "  data: [-0.1, 0.01, 0.001, 0.002, 0.0001]\n"
         "rectification_matrix:\n  rows: 3\n  cols: 3\n"
         "  data: [1, 0, 0, 0, 1, 0, 0, 0, 1]\n"
         "projection_matrix:\n  rows: 3\n  cols: 4\n"
         "  data: [130, 5, 2, " +
         std::string(right ? "-52" : "-26") +
         ", 6, 100, 1, -50, 0, 0, 1, 0]\n";
}

std::string replace_once(std::string value, const std::string& needle,
                         const std::string& replacement) {
  const auto location = value.find(needle);
  CHECK(location != std::string::npos);
  value.replace(location, needle.size(), replacement);
  return value;
}

class CalibrationFiles {
 public:
  explicit CalibrationFiles(unsigned width = 6, unsigned height = 3) {
    char name[] = "/tmp/stereo-splitter-tests-XXXXXX";
    const char* created = ::mkdtemp(name);
    if (!created) throw std::runtime_error("Cannot create calibration test directory");
    directory_ = created;
    config.left_calibration = directory_ + "/left.yaml";
    config.right_calibration = directory_ + "/right.yaml";
    config.scale_x = 1;
    config.scale_y = 1;
    write(false, calibration(width, height));
    write(true, calibration(width, height, true));
  }
  ~CalibrationFiles() {
    std::remove(config.left_calibration.c_str());
    std::remove(config.right_calibration.c_str());
    ::rmdir(directory_.c_str());
  }
  CalibrationFiles(const CalibrationFiles&) = delete;
  CalibrationFiles& operator=(const CalibrationFiles&) = delete;
  void write(bool right, const std::string& contents) {
    std::ofstream output(right ? config.right_calibration : config.left_calibration);
    output << contents;
    output.close();
    if (!output) throw std::runtime_error("Cannot write calibration test fixture");
  }
  SplitterConfig config;

 private:
  std::string directory_;
};

sensor_msgs::ImagePtr input(unsigned width, unsigned eye_height, unsigned channels,
                            unsigned padding = 0) {
  sensor_msgs::ImagePtr image(new sensor_msgs::Image);
  image->header.seq = 91;
  image->header.stamp = ros::Time(123, 456789123);
  image->header.frame_id = "test_optical_frame";
  image->width = width;
  image->height = eye_height * 2;
  image->encoding = channels == 4 ? "bgra8" : "bgr8";
  image->is_bigendian = 0;
  image->step = width * channels + padding;
  image->data.assign(image->step * image->height, 251);
  return image;
}

void pixel(sensor_msgs::Image& image, unsigned row, unsigned column,
           const std::array<std::uint8_t, 3>& bgr) {
  const unsigned channels = image.encoding == "bgra8" ? 4 : 3;
  const std::size_t offset = static_cast<std::size_t>(row) * image.step +
                             column * channels;
  std::copy(bgr.begin(), bgr.end(), image.data.begin() + offset);
  if (channels == 4)
    image.data[offset + 3] = static_cast<std::uint8_t>((row * 31 + column * 17) % 256);
}

void check_pixel(const sensor_msgs::Image& image, unsigned row, unsigned column,
                 const std::array<std::uint8_t, 3>& expected) {
  CHECK(image.encoding == "bgr8");
  const std::size_t offset = static_cast<std::size_t>(row) * image.step + column * 3;
  for (unsigned channel = 0; channel < 3; ++channel)
    CHECK(image.data.at(offset + channel) == expected[channel]);
}

void check_header(const std_msgs::Header& actual, const std_msgs::Header& expected) {
  CHECK(actual.seq == expected.seq);
  CHECK(actual.stamp.sec == expected.stamp.sec);
  CHECK(actual.stamp.nsec == expected.stamp.nsec);
  CHECK(actual.frame_id == expected.frame_id);
}

void check_geometry(const StereoPair& pair, unsigned width, unsigned height,
                     const std_msgs::Header& header) {
  for (const auto& image : {pair.left, pair.right}) {
    CHECK(image);
    check_header(image->header, header);
    CHECK(image->width == width && image->height == height);
    CHECK(image->encoding == "bgr8" && image->is_bigendian == 0);
    CHECK(image->step == width * 3);
    CHECK(image->data.size() == static_cast<std::size_t>(width) * height * 3);
  }
  for (const auto& info : {pair.left_info, pair.right_info}) {
    CHECK(info);
    check_header(info->header, header);
    CHECK(info->width == width && info->height == height);
    CHECK(info->binning_x == 0 && info->binning_y == 0);
    CHECK(info->roi.x_offset == 0 && info->roi.y_offset == 0);
    CHECK(info->roi.width == 0 && info->roi.height == 0 && !info->roi.do_rectify);
  }
}

void padded_bgr_and_bgra_preserve_eye_order() {
  CalibrationFiles files;
  StereoProcessor processor(files.config);
  for (const unsigned channels : {3u, 4u}) {
    auto source = input(6, 3, channels, 11);
    for (unsigned row = 0; row < source->height; ++row) {
      for (unsigned column = 0; column < source->width; ++column) {
        const auto value = static_cast<std::uint8_t>(row * 30 + column);
        pixel(*source, row, column, {{value, static_cast<std::uint8_t>(value + 7),
                                     static_cast<std::uint8_t>(value + 13)}});
      }
    }
    const auto original = source->data;
    StereoPair result;
    CHECK(processor.process(source, result));
    check_geometry(result, 6, 3, source->header);
    for (unsigned row = 0; row < 3; ++row) {
      for (unsigned column = 0; column < 6; ++column) {
        const auto right = static_cast<std::uint8_t>(row * 60 + column);
        const auto left = static_cast<std::uint8_t>(right + 30);
        check_pixel(*result.right, row, column,
                    {{right, static_cast<std::uint8_t>(right + 7),
                      static_cast<std::uint8_t>(right + 13)}});
        check_pixel(*result.left, row, column,
                    {{left, static_cast<std::uint8_t>(left + 7),
                      static_cast<std::uint8_t>(left + 13)}});
      }
    }
    CHECK(source->data == original);
    std::fill(source->data.begin(), source->data.end(), 0);
    check_pixel(*result.left, 0, 0, {{30, 37, 43}});
  }
}

void area_horizontal_uses_all_samples() {
  CalibrationFiles files;
  files.config.scale_x = 1.0 / 3;
  StereoProcessor processor(files.config);
  // AREA averages each group of three. Nearest/linear at this integer ratio
  // select the middle sample (90 or 30), which deliberately differs.
  const std::uint8_t columns[] = {0, 90, 0, 60, 30, 120};
  for (const unsigned channels : {3u, 4u}) {
    auto source = input(6, 3, channels, 7);
    for (unsigned row = 0; row < source->height; ++row) {
      const unsigned eye_offset = row % 2 ? 100 : 0;
      for (unsigned col = 0; col < 6; ++col) {
        const auto value = static_cast<std::uint8_t>(columns[col] + eye_offset);
        pixel(*source, row, col, {{value, value, value}});
      }
    }
    StereoPair result;
    CHECK(processor.process(source, result));
    check_geometry(result, 2, 3, source->header);
    for (unsigned row = 0; row < 3; ++row) {
      check_pixel(*result.right, row, 0, {{30, 30, 30}});
      check_pixel(*result.right, row, 1, {{70, 70, 70}});
      check_pixel(*result.left, row, 0, {{130, 130, 130}});
      check_pixel(*result.left, row, 1, {{170, 170, 170}});
    }
  }
}

void area_vertical_separates_eyes_before_filtering() {
  CalibrationFiles files;
  files.config.scale_x = 1.0 / 3;
  files.config.scale_y = 1.0 / 3;
  StereoProcessor processor(files.config);
  const unsigned columns[] = {0, 60, 0, 30, 0, 60};
  const unsigned rows[] = {0, 60, 0};
  for (const unsigned channels : {3u, 4u}) {
    auto source = input(6, 3, channels, 5);
    for (unsigned row = 0; row < source->height; ++row) {
      for (unsigned col = 0; col < 6; ++col) {
        const auto value = static_cast<std::uint8_t>(
            columns[col] + rows[row / 2] + (row % 2 ? 100 : 0));
        pixel(*source, row, col, {{value, value, value}});
      }
    }
    StereoPair result;
    CHECK(processor.process(source, result));
    check_geometry(result, 2, 1, source->header);
    check_pixel(*result.right, 0, 0, {{40, 40, 40}});
    check_pixel(*result.right, 0, 1, {{50, 50, 50}});
    check_pixel(*result.left, 0, 0, {{140, 140, 140}});
    check_pixel(*result.left, 0, 1, {{150, 150, 150}});
  }
}

void header_including_zero_timestamp_is_preserved() {
  CalibrationFiles files;
  StereoProcessor processor(files.config);
  auto source = input(6, 3, 4);
  for (const auto& stamp : {ros::Time(123, 456789123), ros::Time(0, 0)}) {
    source->header.stamp = stamp;
    StereoPair result;
    CHECK(processor.process(source, result));
    const auto saved = source->header;
    source->header.seq = 500;
    source->header.frame_id = "subsequently_changed";
    source->header.stamp = ros::Time(987, 1);
    check_geometry(result, 6, 3, saved);
    source->header = saved;
  }
}

void calibration_scales_actual_resolution_and_translation() {
  CalibrationFiles files(5, 3);
  files.config.scale_x = 0.5;
  files.config.scale_y = 0.5;
  StereoProcessor processor(files.config);
  CHECK(processor.output_width() == 2 && processor.output_height() == 2);
  StereoPair result;
  auto source = input(5, 3, 3);
  CHECK(processor.process(source, result));
  check_geometry(result, 2, 2, source->header);
  const std::array<double, 9> k{{120, 3, 2.5, 4, 90, 1.5, 0, 0, 1}};
  const std::array<double, 9> r{{1, 0, 0, 0, 1, 0, 0, 0, 1}};
  const std::array<double, 12> p{{130, 5, 2, -26, 6, 100, 1, -50, 0, 0, 1, 0}};
  const std::array<double, 5> d{{-0.1, 0.01, 0.001, 0.002, 0.0001}};
  for (const auto& info : {result.left_info, result.right_info}) {
    CHECK(info->distortion_model == "plumb_bob");
    CHECK(info->D.size() == d.size());
    for (unsigned i = 0; i < k.size(); ++i) {
      const double scale = i < 3 ? 0.4 : i < 6 ? 2.0 / 3 : 1;
      near(info->K[i], k[i] * scale);
      near(info->R[i], r[i]);
    }
    for (unsigned i = 0; i < p.size(); ++i) {
      const double scale = i < 4 ? 0.4 : i < 8 ? 2.0 / 3 : 1;
      const double baseline_factor = info == result.right_info && i == 3 ? 2 : 1;
      near(info->P[i], p[i] * scale * baseline_factor);
    }
    for (unsigned i = 0; i < d.size(); ++i) near(info->D[i], d[i]);
  }
  near(-result.right_info->P[3] / result.right_info->P[0], 52.0 / 130);
  near(result.left_info->P[7], -100.0 / 3);
  CalibrationFiles other(7, 3);
  other.config.scale_x = 0.5;
  StereoProcessor round_up(other.config);
  CHECK(round_up.output_width() == 4);
}

void packaged_calibrations_preserve_geometry_and_baseline() {
  const std::string directory = ENDOSCOPE_TEST_CALIBRATION_DIR;
  SplitterConfig config;
  config.left_calibration = directory + "/left.yaml";
  config.right_calibration = directory + "/right.yaml";
  config.buffer_count = 2;
  StereoProcessor processor(config);
  processor.validate_input_size(1920, 1080);
  must_throw([&] { processor.validate_input_size(1280, 720); });
  StereoPair result;
  const auto source = input(1920, 540, 4);
  CHECK(processor.process(source, result));
  check_geometry(result, 960, 540, source->header);
  for (const bool right : {false, true}) {
    const auto original = YAML::LoadFile(right ? config.right_calibration : config.left_calibration);
    CHECK(original["image_width"].as<unsigned>() == 1920);
    CHECK(original["image_height"].as<unsigned>() == 540);
    const auto k = original["camera_matrix"]["data"].as<std::vector<double>>();
    const auto p = original["projection_matrix"]["data"].as<std::vector<double>>();
    const auto d = original["distortion_coefficients"]["data"].as<std::vector<double>>();
    const auto r = original["rectification_matrix"]["data"].as<std::vector<double>>();
    const auto& info = right ? result.right_info : result.left_info;
    CHECK(info->distortion_model == original["distortion_model"].as<std::string>());
    CHECK(k.size() == info->K.size() && p.size() == info->P.size());
    CHECK(r.size() == info->R.size() && d.size() == info->D.size());
    for (std::size_t i = 0; i < k.size(); ++i) near(info->K[i], k[i] * (i < 3 ? 0.5 : 1));
    for (std::size_t i = 0; i < p.size(); ++i) near(info->P[i], p[i] * (i < 4 ? 0.5 : 1));
    for (std::size_t i = 0; i < d.size(); ++i) near(info->D[i], d[i]);
    for (std::size_t i = 0; i < r.size(); ++i) near(info->R[i], r[i]);
    near(-info->P[3] / info->P[0], -p[3] / p[0]);
  }
}

void invalid_image_and_configuration_are_rejected() {
  CalibrationFiles files;
  StereoProcessor processor(files.config);
  StereoPair result;
  must_throw([&] { processor.process(sensor_msgs::ImageConstPtr(), result); });
  for (unsigned mutation = 0; mutation < 10; ++mutation) {
    auto source = input(6, 3, 4);
    switch (mutation) {
      case 0: source->encoding = "rgb8"; break;
      case 1: source->width = 0; break;
      case 2: source->height = 0; break;
      case 3: source->height = 5; break;
      case 4: source->width = 5; break;
      case 5: source->height = 4; break;
      case 6: source->step = 23; break;
      case 7: source->data.pop_back(); break;
      case 8: source->data.push_back(0); break;
      case 9: source->step = std::numeric_limits<std::uint32_t>::max(); break;
    }
    must_throw([&] { processor.process(source, result); });
  }
  for (const double scale : {0.0, -1.0, 1.1, 0.001,
                             std::numeric_limits<double>::quiet_NaN(),
                             std::numeric_limits<double>::infinity()}) {
    auto config = files.config;
    config.scale_x = scale;
    must_throw([&] { StereoProcessor invalid(config); });
    config = files.config;
    config.scale_y = scale;
    must_throw([&] { StereoProcessor invalid(config); });
  }
  auto config = files.config;
  config.buffer_count = 0;
  must_throw([&] { StereoProcessor invalid(config); });
}

void input_geometry_is_checked_before_frames_arrive() {
  CalibrationFiles files;
  files.config.scale_x = 0.5;
  files.config.scale_y = 2.0 / 3;
  StereoProcessor processor(files.config);
  processor.validate_input_size(6, 6);
  for (const auto& size : std::vector<std::pair<std::uint32_t, std::uint32_t>>{
           {0, 6}, {6, 0}, {5, 6}, {6, 5}, {6, 3}, {3, 2},
           {std::numeric_limits<std::uint32_t>::max(), 6},
           {6, std::numeric_limits<std::uint32_t>::max()}}) {
    must_throw([&] { processor.validate_input_size(size.first, size.second); });
  }
  StereoPair result;
  const auto source = input(6, 3, 4);
  CHECK(processor.process(source, result));
  check_geometry(result, 3, 2, source->header);
}

void malformed_calibration_is_rejected() {
  const auto valid = calibration(6, 3);
  const std::vector<std::string> malformed{
      "[]\n",
      replace_once(valid, "image_width: 6", "image_width: 0"),
      replace_once(valid, "image_width: 6", "image_width: -6"),
      replace_once(valid, "image_width: 6", "image_width: 6.5"),
      replace_once(valid, "image_height: 3", "image_height: 0"),
      replace_once(valid, "plumb_bob", "equidistant"),
      replace_once(valid, "rows: 3", "rows: 2"),
      replace_once(valid, "cols: 5", "cols: 4"),
      replace_once(valid, "data: [120, 3", "data: [.nan, 3"),
      replace_once(valid, "data: [120, 3", "data: [0, 3"),
      replace_once(valid, "data: [-0.1,", "data: [.inf,"),
      replace_once(valid, "data: [1, 0, 0", "data: [1, .nan, 0"),
      replace_once(valid, "data: [130, 5", "data: [130, .inf"),
      replace_once(valid, "data: [130, 5", "data: [-130, 5"),
      replace_once(valid, "0.002, 0.0001]", "0.002]"),
  };
  for (const auto& contents : malformed) {
    CalibrationFiles files;
    files.write(false, contents);
    must_throw([&] { StereoProcessor invalid(files.config); });
  }
  CalibrationFiles mismatch;
  mismatch.write(true, calibration(7, 3, true));
  must_throw([&] { StereoProcessor invalid(mismatch.config); });
  CalibrationFiles missing;
  std::remove(missing.config.right_calibration.c_str());
  must_throw([&] { StereoProcessor invalid(missing.config); });
}

void pooled_outputs_remain_owned_by_consumers() {
  CalibrationFiles files;
  files.config.buffer_count = 2;
  StereoProcessor processor(files.config);
  auto source = input(6, 3, 4);
  StereoPair first;
  CHECK(processor.process(source, first));
  const auto retained_bytes = first.left->data;
  const auto retained_header = first.left_info->header;
  const auto first_message_address = first.left.get();
  const auto first_pixels_address = first.left->data.data();
  std::fill(source->data.begin(), source->data.end(), 17);
  ++source->header.seq;
  StereoPair second;
  CHECK(processor.process(source, second));
  CHECK(first.left.get() != second.left.get());
  CHECK(first.right.get() != second.right.get());
  CHECK(first.left_info.get() != second.left_info.get());
  CHECK(first.right_info.get() != second.right_info.get());
  StereoPair exhausted;
  CHECK(!processor.process(source, exhausted));
  CHECK(first.left->data == retained_bytes);
  check_header(first.left_info->header, retained_header);
  first = StereoPair{};
  StereoPair recycled;
  CHECK(processor.process(source, recycled));
  CHECK(recycled.left.get() == first_message_address);
  CHECK(recycled.left->data.data() == first_pixels_address);
  CHECK(recycled.left.get() != second.left.get());
  CHECK(std::all_of(second.left->data.begin(), second.left->data.end(),
                    [](std::uint8_t value) { return value == 17; }));
}

void retained_camera_info_alone_blocks_its_reuse() {
  CalibrationFiles files;
  files.config.buffer_count = 1;
  StereoProcessor processor(files.config);
  auto source = input(6, 3, 3);
  StereoPair result;
  CHECK(processor.process(source, result));
  auto retained = result.right_info;
  const auto original_header = retained->header;
  result = StereoPair{};
  ++source->header.seq;
  StereoPair next;
  CHECK(!processor.process(source, next));
  check_header(retained->header, original_header);
  retained.reset();
  CHECK(processor.process(source, next));
  check_header(next.right_info->header, source->header);
}

void consumers_can_outlive_the_processor() {
  CalibrationFiles files;
  StereoPair retained;
  const auto source = input(6, 3, 4);
  {
    StereoProcessor processor(files.config);
    CHECK(processor.process(source, retained));
  }
  check_geometry(retained, 6, 3, source->header);
  CHECK(std::all_of(retained.left->data.begin(), retained.left->data.end(),
                    [](std::uint8_t value) { return value == 251; }));
  CHECK(std::all_of(retained.right->data.begin(), retained.right->data.end(),
                    [](std::uint8_t value) { return value == 251; }));
}

}  // namespace

int main() {
  const std::pair<const char*, void (*)()> tests[] = {
      {"padded BGR/BGRA eye order", padded_bgr_and_bgra_preserve_eye_order},
      {"horizontal area filtering", area_horizontal_uses_all_samples},
      {"vertical area after eye separation", area_vertical_separates_eyes_before_filtering},
      {"input headers including zero stamp", header_including_zero_timestamp_is_preserved},
      {"calibration geometry and rounded resolution", calibration_scales_actual_resolution_and_translation},
      {"packaged calibration geometry and baseline", packaged_calibrations_preserve_geometry_and_baseline},
      {"invalid images and configuration", invalid_image_and_configuration_are_rejected},
      {"input geometry before frame processing", input_geometry_is_checked_before_frames_arrive},
      {"malformed calibration", malformed_calibration_is_rejected},
      {"pooled output retention", pooled_outputs_remain_owned_by_consumers},
      {"retained CameraInfo ownership", retained_camera_info_alone_blocks_its_reuse},
      {"consumer outlives processor", consumers_can_outlive_the_processor},
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
