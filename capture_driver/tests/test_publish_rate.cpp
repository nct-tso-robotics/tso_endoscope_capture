#include "endoscope_capture/publish_rate.hpp"

#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <utility>

namespace {
using namespace std::chrono_literals;
using Estimator = endoscope_capture::PublishRateEstimator;
const Estimator::Clock::time_point origin{};

void require(bool condition, const char* expression, int line) {
  if (!condition)
    throw std::runtime_error(std::string("line ") + std::to_string(line) +
                             ": " + expression);
}
#define CHECK(expression) require(static_cast<bool>(expression), #expression, __LINE__)

void near(double actual, double expected) {
  CHECK(std::isfinite(actual));
  CHECK(std::abs(actual - expected) < 1e-9);
}

void startup_needs_a_measured_interval() {
  Estimator rate(30);
  auto sample = rate.record(origin);
  CHECK(!sample.valid && sample.samples == 1);
  near(sample.hz, 0);
  near(sample.window_seconds, 0);
  sample = rate.record(origin + 100ms);
  CHECK(!sample.valid && sample.samples == 2);
  near(sample.hz, 0);
  near(sample.window_seconds, 0.1);
  sample = rate.record(origin + 500ms);
  CHECK(sample.valid && sample.samples == 3);
  near(sample.hz, 4);
  near(sample.window_seconds, 0.5);
}

void requested_rate_can_exceed_delivered_rate() {
  Estimator rate(120);
  Estimator::Sample sample;
  for (unsigned i = 0; i <= 20; ++i)
    sample = rate.record(origin + 50ms * i);
  CHECK(sample.valid && sample.samples == 21);
  near(sample.hz, 20);
  near(sample.window_seconds, 1);
}

void window_tracks_recent_throughput() {
  Estimator rate(60);
  Estimator::Sample sample;
  for (unsigned i = 0; i <= 20; ++i)
    sample = rate.record(origin + 50ms * i);
  near(sample.hz, 20);
  for (unsigned i = 1; i <= 30; ++i)
    sample = rate.record(origin + 1s + 100ms * i);
  CHECK(sample.valid && sample.samples == 21);
  near(sample.hz, 10);
  near(sample.window_seconds, 2);
}

void loss_and_long_gaps_invalidate_stale_samples() {
  Estimator rate(30);
  Estimator::Sample sample;
  for (unsigned i = 0; i <= 10; ++i)
    sample = rate.record(origin + 50ms * i);
  CHECK(sample.valid);
  sample = rate.record(origin + 1500ms);
  CHECK(!sample.valid && sample.samples == 1);
  near(sample.hz, 0);
  near(sample.window_seconds, 0);
  for (unsigned i = 1; i <= 10; ++i)
    sample = rate.record(origin + 1500ms + 50ms * i);
  CHECK(sample.valid);
  near(sample.hz, 20);
  rate.reset();
  sample = rate.record(origin + 2050ms);
  CHECK(!sample.valid && sample.samples == 1);
  near(sample.hz, 0);
}

void slow_configured_rates_are_measurable() {
  Estimator rate(0.1);
  auto sample = rate.record(origin);
  CHECK(!sample.valid);
  sample = rate.record(origin + 10s);
  CHECK(sample.valid && sample.samples == 2);
  near(sample.hz, 0.1);
  for (unsigned i = 2; i <= 4; ++i)
    sample = rate.record(origin + 10s * i);
  CHECK(sample.valid && sample.samples == 4);
  near(sample.hz, 0.1);
  near(sample.window_seconds, 30);
  sample = rate.record(origin + 71s);
  CHECK(!sample.valid && sample.samples == 1);
}

void repeated_or_backwards_times_restart_estimation() {
  Estimator rate(20);
  rate.record(origin);
  auto sample = rate.record(origin + 500ms);
  CHECK(sample.valid);
  sample = rate.record(origin + 500ms);
  CHECK(!sample.valid && sample.samples == 1);
  sample = rate.record(origin + 250ms);
  CHECK(!sample.valid && sample.samples == 1);
  sample = rate.record(origin + 750ms);
  CHECK(sample.valid && sample.samples == 2);
  near(sample.hz, 2);
}

void supported_rates_and_memory_are_bounded() {
  for (const double configured : {0.0, -1.0, 0.01, 240.1,
                                  std::numeric_limits<double>::quiet_NaN(),
                                  std::numeric_limits<double>::infinity()}) {
    bool rejected = false;
    try {
      Estimator invalid(configured);
    } catch (const std::invalid_argument&) {
      rejected = true;
    }
    CHECK(rejected);
  }
  Estimator maximum(240);
  CHECK(!maximum.record(origin).valid);
  Estimator rate(0.1);
  Estimator::Sample sample;
  for (unsigned i = 0; i < 10000; ++i)
    sample = rate.record(origin + 10us * i);
  CHECK(sample.samples == 4096);
  CHECK(!sample.valid);
}
}  // namespace

int main() {
  const std::pair<const char*, void (*)()> tests[] = {
      {"startup validity", startup_needs_a_measured_interval},
      {"requested versus delivered rate", requested_rate_can_exceed_delivered_rate},
      {"recent throughput window", window_tracks_recent_throughput},
      {"loss and stall invalidate history", loss_and_long_gaps_invalidate_stale_samples},
      {"low configured rate", slow_configured_rates_are_measurable},
      {"nonincreasing clock defense", repeated_or_backwards_times_restart_estimation},
      {"configuration and memory bounds", supported_rates_and_memory_are_bounded},
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
