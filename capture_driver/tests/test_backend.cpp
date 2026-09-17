#include "endoscope_capture/decklink_capture.hpp"
#include "sdk_mocks.hpp"

#include <algorithm>
#include <chrono>
#include <climits>
#include <ctime>
#include <future>
#include <iostream>
#include <stdexcept>
#include <thread>

namespace {
using namespace std::chrono_literals;
using capture_test::DeviceState;
using capture_test::MockFrame;
using endoscope_capture::DeckLinkCapture;
using endoscope_capture::Frame;
using endoscope_capture::FrameHandoff;

void check(bool condition, const char* expression, int line) {
  if (!condition)
    throw std::runtime_error(std::string("line ") + std::to_string(line) +
                             ": " + expression);
}
#define CHECK(expression) check(static_cast<bool>(expression), #expression, __LINE__)

template <typename Function> void must_throw(Function&& function) {
  bool threw = false;
  try {
    function();
  } catch (const std::exception&) {
    threw = true;
  }
  CHECK(threw);
}

template <typename T> struct Release {
  void operator()(T* value) const {
    if (value) value->Release();
  }
};
template <typename T> using Owned = std::unique_ptr<T, Release<T>>;

Owned<FrameHandoff> handoff() {
  Owned<FrameHandoff> result(new FrameHandoff);
  result->begin();
  return result;
}

void frame_lifetime_and_padded_copy() {
  auto destroyed = std::make_shared<std::atomic<unsigned>>(0);
  Owned<MockFrame> source(new MockFrame(3, 2, 16, destroyed));
  {
    Frame frame(source.get(), 1234567890123, 42);
    CHECK(source->references() == 2);
    CHECK(source->get_bytes_calls == 0);
    CHECK(frame.width() == 3);
    CHECK(frame.height() == 2);
    CHECK(frame.packed_size() == 24);
    CHECK(frame.timestamp_ns() == 1234567890123);
    CHECK(frame.sequence() == 42);
    std::vector<std::uint8_t> destination(26, 0xAA);
    frame.copy_to(destination.data() + 1, 24);
    CHECK(source->get_bytes_calls == 1);
    CHECK(destination.front() == 0xAA && destination.back() == 0xAA);
    for (std::size_t i = 0; i < 24; ++i) CHECK(destination[i + 1] == i);
    must_throw([&] { frame.copy_to(destination.data(), 23); });
    must_throw([&] { frame.copy_to(nullptr, 24); });
    Frame moved(std::move(frame));
    CHECK(source->references() == 2);
    source.reset();
    CHECK(*destroyed == 0);
    moved.copy_to(destination.data() + 1, 24);
    CHECK(destination[13] == 12);
  }
  CHECK(*destroyed == 1);
}

void invalid_frames_and_bytes_failures() {
  must_throw([] { Frame invalid(nullptr, 0, 0); });
  for (const auto& dimensions : {std::vector<long>{0, 2, 16}, {-1, 2, 16},
                                {3, 0, 16}, {3, -1, 16}, {3, 2, 11},
                                {3, 2, -1}, {LONG_MAX, 2, LONG_MAX},
                                {3, 3, LONG_MAX}}) {
    Owned<MockFrame> source(
        new MockFrame(dimensions[0], dimensions[1], dimensions[2]));
    must_throw([&] { Frame invalid(source.get(), 0, 0); });
    CHECK(source->references() == 1);
  }
  Owned<MockFrame> source(new MockFrame);
  source->format = bmdFormat8BitYUV;
  must_throw([&] { Frame invalid(source.get(), 0, 0); });
  CHECK(source->references() == 1);
  source->format = bmdFormat8BitBGRA;
  Frame frame(source.get(), 0, 0);
  std::vector<std::uint8_t> destination(frame.packed_size(), 0xAA);
  source->bytes_result = E_FAIL;
  must_throw([&] { frame.copy_to(destination.data(), destination.size()); });
  CHECK(std::all_of(destination.begin(), destination.end(),
                    [](std::uint8_t value) { return value == 0xAA; }));
  source->bytes_result = S_OK;
  source->null_bytes = true;
  must_throw([&] { frame.copy_to(destination.data(), destination.size()); });
}

void latest_frame_and_held_lifetime() {
  auto queue = handoff();
  auto destroyed = std::make_shared<std::atomic<unsigned>>(0);
  Owned<MockFrame> first(new MockFrame(3, 2, 16, destroyed));
  CHECK(queue->VideoInputFrameArrived(first.get(), nullptr) == S_OK);
  first.reset();
  auto held = queue->wait_for_frame(10ms);
  CHECK(held && *destroyed == 0);
  const auto first_sequence = held->sequence();
  CHECK(held->timestamp_ns() > 0);
  for (unsigned i = 0; i < 3; ++i) {
    Owned<MockFrame> next(new MockFrame(3, 2, 16, destroyed));
    CHECK(queue->VideoInputFrameArrived(next.get(), nullptr) == S_OK);
  }
  CHECK(*destroyed == 2);
  auto newest = queue->wait_for_frame(10ms);
  CHECK(newest && newest->sequence() == first_sequence + 3);
  CHECK(newest->timestamp_ns() >= held->timestamp_ns());
  const auto stats = queue->stats();
  CHECK(stats.frames_received == 4);
  CHECK(stats.frames_delivered == 2);
  CHECK(stats.overwritten == 2);
  std::vector<std::uint8_t> bytes(held->packed_size());
  held->copy_to(bytes.data(), bytes.size());
  CHECK(bytes[12] == 12);
  queue->close();
  CHECK(*destroyed == 2);
  held.reset();
  newest.reset();
  CHECK(*destroyed == 4);
}

void callback_query_interface() {
  auto queue = handoff();
  void* value = reinterpret_cast<void*>(1);
  CHECK(queue->QueryInterface(IID_IDeckLinkInputCallback, &value) == S_OK);
  CHECK(value == static_cast<IDeckLinkInputCallback*>(queue.get()));
  static_cast<IDeckLinkInputCallback*>(value)->Release();
  const REFIID unknown = IID_IUnknown;
  CHECK(queue->QueryInterface(unknown, &value) == S_OK);
  static_cast<IUnknown*>(value)->Release();
  CHECK(queue->QueryInterface(IID_IDeckLinkInput, &value) == E_NOINTERFACE);
  CHECK(value == nullptr);
  CHECK(queue->QueryInterface(unknown, nullptr) == E_POINTER);
  queue->close();
}

void signal_loss_and_invalid_callback() {
  auto queue = handoff();
  CHECK(queue->VideoInputFrameArrived(nullptr, nullptr) == S_OK);
  Owned<MockFrame> lost(new MockFrame);
  lost->flags = bmdFrameHasNoInputSource;
  CHECK(queue->VideoInputFrameArrived(lost.get(), nullptr) == S_OK);
  CHECK(!queue->stats().signal_present);
  CHECK(queue->stats().no_signal == 1);
  CHECK(lost->references() == 1);
  Owned<MockFrame> invalid(new MockFrame(3, 2, 11));
  CHECK(queue->VideoInputFrameArrived(invalid.get(), nullptr) == S_OK);
  CHECK(queue->stats().invalid_frames == 1);
  CHECK(invalid->references() == 1);
  Owned<MockFrame> valid(new MockFrame);
  CHECK(queue->VideoInputFrameArrived(valid.get(), nullptr) == S_OK);
  CHECK(queue->stats().signal_present);
  CHECK(queue->wait_for_frame(10ms));
  // Losing signal must discard a previously waiting frame, so a reader cannot
  // mistake the old picture for new input readiness.
  CHECK(queue->VideoInputFrameArrived(valid.get(), nullptr) == S_OK);
  CHECK(queue->VideoInputFrameArrived(lost.get(), nullptr) == S_OK);
  CHECK(!queue->wait_for_frame(0ms));
  CHECK(!queue->stats().signal_present);
  CHECK(queue->VideoInputFrameArrived(valid.get(), nullptr) == S_OK);
  CHECK(queue->VideoInputFrameArrived(invalid.get(), nullptr) == S_OK);
  CHECK(!queue->wait_for_frame(0ms));
  CHECK(!queue->stats().signal_present);
  CHECK(queue->VideoInputFrameArrived(valid.get(), nullptr) == S_OK);
  CHECK(queue->VideoInputFormatChanged(bmdVideoInputDisplayModeChanged, nullptr,
                                      bmdDetectedVideoInputRGB444) == S_OK);
  CHECK(queue->stats().format_changes == 1);
  CHECK(!queue->wait_for_frame(0ms));
  CHECK(!queue->stats().signal_present);
  CHECK(queue->stats().discarded == 3);
  CHECK(queue->stats().pending_frames == 0);
  CHECK(queue->stats().frames_received == 4);
  CHECK(queue->stats().frames_delivered == 1);
  queue->close();
}

void format_change_latches_until_restart() {
  auto queue = handoff();
  std::promise<void> entering;
  auto entered = entering.get_future();
  auto waiter = std::async(std::launch::async, [&] {
    entering.set_value();
    return queue->wait_for_frame(5s);
  });
  entered.wait();
  std::this_thread::sleep_for(10ms);
  CHECK(queue->VideoInputFormatChanged(bmdVideoInputDisplayModeChanged, nullptr,
                                      bmdDetectedVideoInputRGB444) == S_OK);
  CHECK(waiter.wait_for(1s) == std::future_status::ready);
  CHECK(!waiter.get());
  CHECK(!queue->stats().running);
  Owned<MockFrame> next(new MockFrame);
  CHECK(queue->VideoInputFrameArrived(next.get(), nullptr) == S_OK);
  CHECK(!queue->wait_for_frame(5s));
  CHECK(!queue->stats().signal_present);
  CHECK(next->references() == 1);
  queue->begin();
  CHECK(queue->stats().running);
  CHECK(queue->stats().format_changes == 0);
  CHECK(queue->VideoInputFrameArrived(next.get(), nullptr) == S_OK);
  CHECK(queue->wait_for_frame(10ms));
  queue->close();
}

void no_input_timeout_and_cancel() {
  auto queue = handoff();
  const auto wall_start = std::chrono::steady_clock::now();
  const auto cpu_start = std::clock();
  CHECK(!queue->wait_for_frame(80ms));
  const auto wall = std::chrono::steady_clock::now() - wall_start;
  const double cpu_seconds = static_cast<double>(std::clock() - cpu_start) /
                             CLOCKS_PER_SEC;
  CHECK(wall >= 60ms && wall < 2s);
  CHECK(cpu_seconds < 0.04);
  CHECK(queue->stats().timeouts == 1);
  std::promise<void> entering;
  auto entered = entering.get_future();
  auto waiter = std::async(std::launch::async, [&] {
    entering.set_value();
    return queue->wait_for_frame(5s);
  });
  entered.wait();
  std::this_thread::sleep_for(10ms);
  queue->close();
  CHECK(waiter.wait_for(1s) == std::future_status::ready);
  CHECK(!waiter.get());
  CHECK(!queue->wait_for_frame(5s));
  CHECK(!queue->stats().running);
  CHECK(queue->stats().timeouts == 1);
  Owned<MockFrame> late(new MockFrame);
  CHECK(queue->VideoInputFrameArrived(late.get(), nullptr) == S_OK);
  CHECK(late->references() == 1);
  CHECK(!queue->wait_for_frame(1ms));
}

void concurrent_handoff_accounting() {
  auto queue = handoff();
  constexpr unsigned count = 2000;
  auto destroyed = std::make_shared<std::atomic<unsigned>>(0);
  std::atomic<bool> producer_done{false};
  auto producer = std::async(std::launch::async, [&] {
    for (unsigned i = 0; i < count; ++i) {
      Owned<MockFrame> frame(new MockFrame(3, 2, 16, destroyed));
      queue->VideoInputFrameArrived(frame.get(), nullptr);
      if (i % 7 == 0) std::this_thread::yield();
    }
    producer_done = true;
  });
  unsigned consumed = 0;
  std::uint64_t previous_sequence = 0;
  bool had_previous = false;
  while (true) {
    auto frame = queue->wait_for_frame(1ms);
    if (!frame) {
      if (producer_done) break;
      continue;
    }
    if (had_previous) CHECK(frame->sequence() > previous_sequence);
    previous_sequence = frame->sequence();
    had_previous = true;
    std::vector<std::uint8_t> bytes(frame->packed_size());
    frame->copy_to(bytes.data(), bytes.size());
    CHECK(bytes[12] == 12);
    ++consumed;
  }
  producer.get();
  const auto stats = queue->stats();
  CHECK(stats.frames_received == count);
  CHECK(stats.frames_delivered == consumed);
  CHECK(stats.frames_delivered + stats.overwritten == count);
  queue->close();
  CHECK(*destroyed == count);
}

void concurrent_close_releases_all_frames() {
  auto queue = handoff();
  constexpr unsigned count = 2000;
  auto destroyed = std::make_shared<std::atomic<unsigned>>(0);
  std::promise<void> midpoint;
  auto reached_midpoint = midpoint.get_future();
  std::promise<void> resume;
  auto can_resume = resume.get_future();
  auto producer = std::async(std::launch::async, [&] {
    for (unsigned i = 0; i < count; ++i) {
      if (i == count / 2) {
        midpoint.set_value();
        can_resume.wait();
      }
      Owned<MockFrame> frame(new MockFrame(3, 2, 16, destroyed));
      queue->VideoInputFrameArrived(frame.get(), nullptr);
    }
  });
  auto consumer = std::async(std::launch::async, [&] {
    while (queue->stats().running) {
      auto frame = queue->wait_for_frame(5s);
      if (frame) {
        std::vector<std::uint8_t> bytes(frame->packed_size());
        frame->copy_to(bytes.data(), bytes.size());
        CHECK(bytes[12] == 12);
      }
    }
  });
  reached_midpoint.wait();
  resume.set_value();
  queue->close();
  producer.get();
  CHECK(consumer.wait_for(1s) == std::future_status::ready);
  consumer.get();
  CHECK(!queue->wait_for_frame(5s));
  CHECK(*destroyed == count);
  const auto stats = queue->stats();
  CHECK(stats.pending_frames == 0);
  CHECK(stats.frames_received ==
        stats.frames_delivered + stats.overwritten + stats.discarded);
}

std::shared_ptr<DeviceState> device_state;
IDeckLinkIterator* mock_factory() {
  if (device_state->fail == "factory") return nullptr;
  return new capture_test::MockIterator(device_state);
}
bool called(const char* operation) {
  const auto& calls = device_state->calls;
  return std::find(calls.begin(), calls.end(), operation) != calls.end();
}
void check_destroyed(const std::string& failure) {
  CHECK(device_state->input == nullptr);
  CHECK(*device_state->iterators_destroyed == (failure == "factory" ? 0 : 1));
  const bool made_device = failure != "factory" && failure != "no_device" &&
                           failure != "next";
  CHECK(*device_state->devices_destroyed == (made_device ? 1 : 0));
  CHECK(*device_state->inputs_destroyed ==
        (made_device && failure != "input" ? 1 : 0));
}

void capture_start_stop_and_restart() {
  device_state = std::make_shared<DeviceState>();
  {
    DeckLinkCapture capture(mock_factory);
    capture.start();
    CHECK(capture.stats().running);
    CHECK(capture.width() == 1920 && capture.height() == 1080);
    CHECK(called("enable") && called("callback") && called("start"));
    Owned<MockFrame> source(new MockFrame);
    device_state->input->arrive(source.get());
    auto frame = capture.wait_for_frame(10ms);
    CHECK(frame && frame->width() == 3);
    capture.stop();
    CHECK(!capture.stats().running);
    CHECK(called("stop") && called("detach") && called("disable"));
    const auto call_count = device_state->calls.size();
    capture.stop();
    CHECK(device_state->calls.size() == call_count);
    check_destroyed("");
    std::vector<std::uint8_t> bytes(frame->packed_size());
    frame->copy_to(bytes.data(), bytes.size());
    CHECK(bytes[12] == 12);
    device_state = std::make_shared<DeviceState>();
    capture.start();
    CHECK(capture.stats().frames_received == 0);
    CHECK(capture.stats().running);
  }
  check_destroyed("");
}

void capture_startup_failure_cleanup() {
  for (const std::string failure : {"factory", "no_device", "next", "input",
                                    "support", "unsupported", "null_mode", "enable",
                                    "callback", "start"}) {
    device_state = std::make_shared<DeviceState>();
    device_state->fail = failure;
    device_state->install_callback_before_failure = failure == "callback";
    {
      DeckLinkCapture capture(mock_factory);
      must_throw([&] { capture.start(); });
      CHECK(!capture.stats().running);
      capture.stop();
      if (called("enable")) CHECK(called("disable"));
      if (called("callback")) CHECK(called("detach"));
      if (called("start")) CHECK(called("stop"));
    }
    check_destroyed(failure);
  }
}

void capture_shutdown_errors_still_cleanup() {
  for (const std::string failure : {"stop", "detach", "disable"}) {
    device_state = std::make_shared<DeviceState>();
    {
      DeckLinkCapture capture(mock_factory);
      capture.start();
      device_state->fail = failure;
      capture.stop();
      CHECK(!capture.stats().running);
      CHECK(called("stop") && called("detach") && called("disable"));
      CHECK(capture.stats().sdk_errors >= 1);
    }
    check_destroyed("");
  }
}

}  // namespace

int main() {
  const std::pair<const char*, void (*)()> tests[] = {
      {"frame lifetime and padded copy", frame_lifetime_and_padded_copy},
      {"invalid frames and GetBytes failures", invalid_frames_and_bytes_failures},
      {"latest frame and held lifetime", latest_frame_and_held_lifetime},
      {"callback QueryInterface", callback_query_interface},
      {"signal loss and invalid callback", signal_loss_and_invalid_callback},
      {"format change latches until restart", format_change_latches_until_restart},
      {"no-input timeout and cancellation", no_input_timeout_and_cancel},
      {"concurrent handoff accounting", concurrent_handoff_accounting},
      {"concurrent close releases all frames", concurrent_close_releases_all_frames},
      {"capture start/stop and restart", capture_start_stop_and_restart},
      {"startup failure cleanup", capture_startup_failure_cleanup},
      {"shutdown errors still clean up", capture_shutdown_errors_still_cleanup},
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
