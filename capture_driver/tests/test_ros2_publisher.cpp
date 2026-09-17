// End-to-end checks of the production publisher over an isolated local DDS domain.
#include <endoscope_capture/msg/capture_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <yaml-cpp/yaml.h>

#include <fcntl.h>
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <map>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {
namespace fs = std::filesystem;
using Clock = std::chrono::steady_clock;
using Image = sensor_msgs::msg::Image;
using CameraInfo = sensor_msgs::msg::CameraInfo;
using Status = endoscope_capture::msg::CaptureStatus;
constexpr std::size_t kFrames = 60;
constexpr std::uint64_t kRate = 30;
constexpr std::size_t kDepth = 3;

void require(bool condition, const std::string& reason) {
  if (!condition) throw std::runtime_error(reason);
}

class TemporaryDirectory {
 public:
  TemporaryDirectory() {
    const auto pattern = (fs::temp_directory_path() / "capture_ros2_test_XXXXXX").string();
    std::vector<char> writable(pattern.begin(), pattern.end());
    writable.push_back('\0');
    const auto created = mkdtemp(writable.data());
    if (!created) throw std::runtime_error("Cannot create test directory");
    path = created;
  }
  ~TemporaryDirectory() {
    std::error_code error;
    fs::remove_all(path, error);
    if (error) std::cerr << "Temporary directory cleanup failed: " << error.message() << '\n';
  }
  fs::path path;
};

class RosContext {
 public:
  RosContext() { rclcpp::init(0, nullptr); }
  ~RosContext() { rclcpp::shutdown(); }
};

// The fixture owns no child processes. Its process group is terminated on every
// exceptional exit, with a bounded escalation before temporary files are removed.
class Child {
 public:
  Child(const std::vector<std::string>& arguments, const fs::path& log) {
    std::vector<char*> argv;
    for (const auto& argument : arguments) argv.push_back(const_cast<char*>(argument.c_str()));
    argv.push_back(nullptr);
    char* const* command = argv.data();
    const int output = open(log.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0600);
    if (output < 0) throw std::runtime_error("Cannot open fixture log");
    pid_ = fork();
    if (pid_ == 0) {
      // Only async-signal-safe operations after fork: ROS has background threads.
      if (setsid() < 0 || dup2(output, STDOUT_FILENO) < 0 || dup2(output, STDERR_FILENO) < 0)
        _exit(126);
      close(output);
      execv(command[0], command);
      _exit(127);
    }
    close(output);
    if (pid_ < 0) throw std::runtime_error("Cannot start fixture");
  }
  Child(const Child&) = delete;
  Child& operator=(const Child&) = delete;
  ~Child() {
    for (const int action : {SIGINT, SIGTERM, SIGKILL}) {
      if (poll()) return;
      if (kill(-pid_, action) < 0 && errno == ESRCH) kill(pid_, action);
      const auto deadline = Clock::now() + std::chrono::seconds(1);
      while (!poll() && Clock::now() < deadline)
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (!poll()) std::cerr << "Fixture process did not exit after SIGKILL\n";
  }
  bool poll() noexcept {
    if (finished_) return true;
    const auto result = waitpid(pid_, &status_, WNOHANG);
    if (result == pid_ || (result < 0 && errno == ECHILD)) finished_ = true;
    return finished_;
  }
  bool succeeded() const { return finished_ && WIFEXITED(status_) && WEXITSTATUS(status_) == 0; }
  void wait(std::chrono::seconds timeout) {
    const auto deadline = Clock::now() + timeout;
    while (!poll() && Clock::now() < deadline)
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    require(poll(), "Fixture exceeded its bounded runtime");
  }

 private:
  pid_t pid_ = -1;
  int status_ = -1;
  bool finished_ = false;
};

std::string read_text(const fs::path& path) {
  std::ifstream stream(path);
  return {std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>()};
}

void write_calibration(const fs::path& path, bool right) {
  std::ofstream output(path);
  output << "image_width: 8\nimage_height: 4\ndistortion_model: plumb_bob\n"
            "distortion_coefficients: {rows: 1, cols: 5, data: [0.01, 0, 0, 0, 0]}\n"
            "camera_matrix: {rows: 3, cols: 3, data: [80, 0, 3.5, 0, 90, 1.5, 0, 0, 1]}\n"
            "rectification_matrix: {rows: 3, cols: 3, data: [1, 0, 0, 0, 1, 0, 0, 0, 1]}\n"
            "projection_matrix: {rows: 3, cols: 4, data: [80, 0, 3.5, "
         << (right ? -8 : 0) << ", 0, 90, 1.5, -2, 0, 0, 1, 0]}\n";
  require(output.good(), "Cannot write test calibration");
}

std::vector<std::string> command(const fs::path& fixture, const fs::path& folder,
                                 const std::string& prefix, const std::string& reliability,
                                 int depth) {
  return {fixture.string(), "--ros-args", "-r", "__ns:=" + prefix,
          "-p", "left_calibration:=" + (folder / "left.yaml").string(),
          "-p", "right_calibration:=" + (folder / "right.yaml").string(),
          "-p", "reliability:=" + reliability, "-p", "qos_depth:=" + std::to_string(depth)};
}

template <typename Message>
using Received = std::vector<std::pair<Clock::time_point, std::shared_ptr<const Message>>>;

template <typename Message>
void subscribe(const std::shared_ptr<rclcpp::Node>& node, const std::string& topic,
               const rclcpp::QoS& qos, Received<Message>& received,
               std::vector<rclcpp::SubscriptionBase::SharedPtr>& subscriptions,
               std::map<std::string, bool>& discovered) {
  discovered.emplace(topic, false);
  subscriptions.push_back(node->create_subscription<Message>(
      topic, qos, [&received](const std::shared_ptr<const Message> message) {
        received.emplace_back(Clock::now(), message);
      }));
}

template <typename Message>
void check_headers(const Received<Message>& received, Clock::time_point& latest,
                   const std::string& topic) {
  require(received.size() == kFrames, "Frame count differs: " + topic);
  for (std::size_t index = 0; index < received.size(); ++index) {
    const auto& header = received[index].second->header;
    const auto stamp = static_cast<std::int64_t>(header.stamp.sec) * 1000000000 + header.stamp.nanosec;
    const auto expected = 200000000000LL + index * 1000000000 / kRate + 12345;
    require(stamp == static_cast<std::int64_t>(expected), "Missing or altered timestamp: " + topic);
    require(header.frame_id == "synthetic_camera", "Frame ID changed: " + topic);
    latest = std::max(latest, received[index].first);
  }
}

template <typename Values, std::size_t Size>
void check_values(const Values& values, const std::array<double, Size>& expected) {
  require(values.size() == Size, "Calibration array size changed");
  for (std::size_t index = 0; index < Size; ++index)
    require(std::abs(values[index] - expected[index]) < 1e-12, "Calibration matrix differs");
}

void check_combined(const Received<Image>& images, const Received<CameraInfo>& infos) {
  for (const auto& item : images) {
    const auto& image = *item.second;
    require(image.width == 8 && image.height == 8 && image.step == 40 &&
                image.encoding == "bgra8" && image.data.size() == 320,
            "Combined image layout changed");
    for (std::size_t row = 0; row < 8; ++row) {
      for (std::size_t column = 0; column < 8; ++column) {
        const auto base = (row % 2 ? 110 : 10) + (row / 2) * 4 + (column % 2) * 40;
        for (std::size_t channel = 0; channel < 4; ++channel) {
          const auto expected = channel == 3 ? (column * 31 + row * 17) % 256 : base + channel * 20;
          require(image.data[row * 40 + column * 4 + channel] == expected, "Combined pixel changed");
        }
      }
      for (std::size_t offset = 32; offset < 40; ++offset)
        require(image.data[row * 40 + offset] == 238, "Combined padding changed");
    }
  }
  for (const auto& item : infos)
    require(item.second->width == 8 && item.second->height == 8 && item.second->k[0] == 0,
            "Combined camera metadata changed");
}

void check_eye(const Received<Image>& images, const Received<CameraInfo>& infos, bool right) {
  for (const auto& item : images) {
    const auto& image = *item.second;
    require(image.width == 4 && image.height == 2 && image.step == 12 &&
                image.encoding == "bgr8" && image.data.size() == 24,
            "Stereo image layout changed");
    // The area average is computed directly from the four original samples,
    // independently of the production splitter or an OpenCV reference path.
    for (std::size_t row = 0; row < 2; ++row)
      for (std::size_t column = 0; column < 4; ++column)
        for (std::size_t channel = 0; channel < 3; ++channel)
          require(image.data[row * 12 + column * 3 + channel] ==
                      (right ? 10 : 110) + row * 8 + 22 + channel * 20,
                  "Stereo pixels or eye ordering changed");
  }
  for (const auto& item : infos) {
    const auto& info = *item.second;
    require(info.width == 4 && info.height == 2 && info.distortion_model == "plumb_bob",
            "Stereo calibration dimensions or model changed");
    check_values(info.k, std::array<double, 9>{40, 0, 1.75, 0, 45, 0.75, 0, 0, 1});
    check_values(info.p, std::array<double, 12>{40, 0, 1.75, right ? -4.0 : 0.0,
                                               0, 45, 0.75, -1, 0, 0, 1, 0});
    check_values(info.r, std::array<double, 9>{1, 0, 0, 0, 1, 0, 0, 0, 1});
    check_values(info.d, std::array<double, 5>{0.01, 0, 0, 0, 0});
    require(info.binning_x == 0 && info.binning_y == 0 && info.roi.x_offset == 0 &&
                info.roi.y_offset == 0 && info.roi.width == 0 && info.roi.height == 0 &&
                !info.roi.do_rectify,
            "Unexpected binning or ROI");
  }
}

void check_status(const Received<Status>& statuses) {
  require(!statuses.front().second->measured_rate_valid, "Initial rate must be invalid");
  require(statuses.back().second->measured_rate_valid, "Steady rate never became valid");
  for (std::size_t index = 0; index < statuses.size(); ++index) {
    const auto& status = *statuses[index].second;
    require(status.source_sequence == index + 1 && status.configured_publish_hz == kRate,
            "Source sequence or configured rate changed");
    require(std::isfinite(status.measured_publish_hz), "Nonfinite measured rate");
    if (status.measured_rate_valid) {
      require(status.rate_window_seconds >= 0.5 && status.rate_window_samples >= 2,
              "Rate estimate has insufficient samples");
      const auto expected = (status.rate_window_samples - 1) / status.rate_window_seconds;
      require(std::abs(status.measured_publish_hz - expected) < 1e-9,
              "Rate estimate disagrees with its window");
      require(status.measured_publish_hz > 24 && status.measured_publish_hz < 36,
              "Unexpected achieved rate");
    } else {
      require(status.measured_publish_hz == 0, "Invalid rate must be zero");
    }
  }
}

void check_diagnostics(const Received<std_msgs::msg::String>& received, Clock::time_point latest) {
  std::vector<Clock::time_point> no_input;
  bool complete = false;
  for (const auto& item : received) {
    const auto value = YAML::Load(item.second->data);
    const auto phase = value["phase"].as<std::string>();
    complete = complete || phase == "complete";
    if (phase != "no_input") continue;
    no_input.push_back(item.first);
    const auto stats = value["publisher"];
    require(stats["qos_depth"].as<std::size_t>() == kDepth, "Actual publisher queue depth differs");
    require(stats["received"].as<std::size_t>() == kFrames &&
                stats["processed"].as<std::size_t>() == kFrames,
            "Publisher frame counters differ");
    require(stats["errors"].as<int>() == 0 && stats["pool_exhausted"].as<int>() == 0,
            "Publisher reported errors");
  }
  require(complete, "Missing completion diagnostic");
  require(no_input.size() >= 3 && no_input.back() - no_input.front() >= std::chrono::milliseconds(500),
          "Diagnostics stopped during input loss");
  require(latest < no_input.front() + std::chrono::milliseconds(200),
          "Image, metadata or status continued during input loss");
}

void run_case(const fs::path& fixture, const fs::path& folder, const std::string& reliability) {
  const auto prefix = "/capture_ros2_test_" + std::to_string(getpid()) + "_" + reliability;
  const auto private_prefix = prefix + "/capture_fixture";
  auto node = std::make_shared<rclcpp::Node>("publisher_observer_" + reliability);
  const auto policy = reliability == "reliable" ? RMW_QOS_POLICY_RELIABILITY_RELIABLE
                                                : RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  auto qos = rclcpp::QoS(100).durability_volatile().reliability(policy);
  Received<Image> combined, left, right;
  Received<CameraInfo> combined_info, left_info, right_info;
  Received<Status> statuses;
  Received<std_msgs::msg::String> diagnostics;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions;
  std::map<std::string, bool> discovered;
  subscribe(node, private_prefix + "/image_raw", qos, combined, subscriptions, discovered);
  subscribe(node, private_prefix + "/camera_info", qos, combined_info, subscriptions, discovered);
  subscribe(node, private_prefix + "/status", qos, statuses, subscriptions, discovered);
  subscribe(node, private_prefix + "/diagnostics", qos, diagnostics, subscriptions, discovered);
  subscribe(node, prefix + "/left/image_raw", qos, left, subscriptions, discovered);
  subscribe(node, prefix + "/left/camera_info", qos, left_info, subscriptions, discovered);
  subscribe(node, prefix + "/right/image_raw", qos, right, subscriptions, discovered);
  subscribe(node, prefix + "/right/camera_info", qos, right_info, subscriptions, discovered);
  const auto log = folder / (reliability + ".log");
  Child child(command(fixture, folder, prefix, reliability, kDepth), log);
  const auto deadline = Clock::now() + std::chrono::seconds(25);
  while (!child.poll() && rclcpp::ok() && Clock::now() < deadline) {
    rclcpp::spin_some(node);
    for (auto& topic : discovered) {
      if (topic.second) continue;
      const auto endpoints = node->get_publishers_info_by_topic(topic.first);
      if (endpoints.empty()) continue;
      require(endpoints.size() == 1 &&
                  endpoints.front().qos_profile().get_rmw_qos_profile().reliability == policy,
              "Advertised QoS reliability differs: " + topic.first);
      topic.second = true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  require(child.poll(), "Fixture exceeded its bounded runtime");
  const auto drain_until = Clock::now() + std::chrono::milliseconds(200);
  while (Clock::now() < drain_until) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  require(child.succeeded(), "Fixture failed: " + read_text(log));
  for (const auto& topic : discovered) require(topic.second, "Publisher was not discovered: " + topic.first);
  Clock::time_point latest{};
  check_headers(combined, latest, "combined");
  check_headers(combined_info, latest, "combined_info");
  check_headers(left, latest, "left");
  check_headers(left_info, latest, "left_info");
  check_headers(right, latest, "right");
  check_headers(right_info, latest, "right_info");
  check_headers(statuses, latest, "status");
  check_combined(combined, combined_info);
  check_eye(left, left_info, false);
  check_eye(right, right_info, true);
  check_status(statuses);
  check_diagnostics(diagnostics, latest);
  std::cout << "PASS ROS2 " << reliability << ": 60 complete stereo frames and no-input diagnostics\n";
}

void reject_invalid_depth(const fs::path& fixture, const fs::path& folder, int depth) {
  const auto prefix = "/capture_ros2_invalid_" + std::to_string(getpid()) + "_" + std::to_string(depth);
  const auto log = folder / ("invalid_" + std::to_string(depth) + ".log");
  Child child(command(fixture, folder, prefix, "reliable", depth), log);
  child.wait(std::chrono::seconds(5));
  require(!child.succeeded(), "Invalid QoS depth was accepted");
  require(read_text(log).find("qos_depth must be between 1 and 32") != std::string::npos,
          "Invalid depth did not fail in publisher configuration: " + read_text(log));
}
}  // namespace

int main(int argc, char** argv) {
  try {
    require(argc == 2, "Usage: capture_ros2_publisher_tests FIXTURE_EXECUTABLE");
    const auto fixture = fs::absolute(argv[1]);
    TemporaryDirectory temporary;
    const auto domain = std::to_string(std::random_device{}() % 100 + 1);
    require(setenv("ROS_DOMAIN_ID", domain.c_str(), 1) == 0 &&
                setenv("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST", 1) == 0 &&
                setenv("ROS_LOG_DIR", (temporary.path / "ros_logs").c_str(), 1) == 0,
            "Cannot configure isolated ROS environment");
    for (const auto* name : {"ROS_LOCALHOST_ONLY", "ROS_STATIC_PEERS", "ROS_DISCOVERY_SERVER"})
      require(unsetenv(name) == 0, "Cannot isolate ROS discovery");
    RosContext context;
    write_calibration(temporary.path / "left.yaml", false);
    write_calibration(temporary.path / "right.yaml", true);
    run_case(fixture, temporary.path, "reliable");
    run_case(fixture, temporary.path, "best_effort");
    for (const int depth : {0, 33}) reject_invalid_depth(fixture, temporary.path, depth);
    std::cout << "PASS ROS2 invalid QoS depths rejected before frame publication\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "FAIL ROS2 publisher: " << error.what() << '\n';
    return 1;
  }
}
