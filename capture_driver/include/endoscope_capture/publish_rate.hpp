#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <stdexcept>

namespace endoscope_capture {

// Recent publication throughput measured from a monotonic clock. Configured
// frequency is used only for window/stall thresholds, never as measured data.
// The caller resets on input loss; a long gap also invalidates old samples.
class PublishRateEstimator {
 public:
  using Clock = std::chrono::steady_clock;
  struct Sample {
    double hz = 0;
    bool valid = false;
    std::uint32_t samples = 0;
    double window_seconds = 0;
  };

  explicit PublishRateEstimator(double configured_hz) {
    if (!std::isfinite(configured_hz) || configured_hz < 0.1 || configured_hz > 240)
      throw std::invalid_argument("Publication rate must be finite and in [0.1, 240] Hz");
    // At low publication rates a fixed two-second window would never contain
    // two frames. Retain at least three configured periods in that case.
    const double three_periods = 3.0 / configured_hz;
    window_seconds_ = std::max(2.0, three_periods);
    stall_seconds_ = std::max(0.5, three_periods);
  }

  Sample record(Clock::time_point timestamp) {
    if (!timestamps_.empty()) {
      const double gap = seconds(timestamp - timestamps_.back());
      if (gap <= 0 || gap > stall_seconds_) timestamps_.clear();
    }
    timestamps_.push_back(timestamp);
    while (timestamps_.size() > 1 &&
           seconds(timestamp - timestamps_.front()) > window_seconds_)
      timestamps_.pop_front();
    // Normal supported rates need at most 481 entries. Keep malformed callers
    // with unexpectedly frequent timestamps bounded as well.
    while (timestamps_.size() > 4096) timestamps_.pop_front();
    Sample sample;
    sample.samples = static_cast<std::uint32_t>(timestamps_.size());
    sample.window_seconds = seconds(timestamp - timestamps_.front());
    sample.valid = sample.samples >= 2 && sample.window_seconds >= 0.5;
    if (sample.valid)
      sample.hz = (sample.samples - 1) / sample.window_seconds;
    return sample;
  }

  void reset() { timestamps_.clear(); }

 private:
  static double seconds(Clock::duration duration) {
    return std::chrono::duration<double>(duration).count();
  }
  std::deque<Clock::time_point> timestamps_;
  double window_seconds_ = 2;
  double stall_seconds_ = 0.5;
};

}  // namespace endoscope_capture
