// Maintained replacement for the legacy BlackMagicCapture backend (original
// author: mediassist, 2011) and ROS integration (Takashi Ogura, 2015).
// See NOTICE.md for attribution; vendor SDK files retain their own notices.
#include "endoscope_capture/decklink_capture.hpp"

#include <atomic>
#include <condition_variable>
#include <cstring>
#include <iomanip>
#include <limits>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace endoscope_capture {
namespace {

template <typename T>
class ComPtr {
 public:
  ComPtr() noexcept = default;
  // Adopts an existing owned reference (e.g. one returned by QueryInterface).
  explicit ComPtr(T* pointer) noexcept : pointer_(pointer) {}
  ~ComPtr() { reset(); }
  ComPtr(const ComPtr&) = delete;
  ComPtr& operator=(const ComPtr&) = delete;
  ComPtr(ComPtr&& other) noexcept : pointer_(other.release()) {}
  ComPtr& operator=(ComPtr&& other) noexcept {
    if (this != &other) reset(other.release());
    return *this;
  }
  T* get() const noexcept { return pointer_; }
  T* operator->() const noexcept { return pointer_; }
  explicit operator bool() const noexcept { return pointer_ != nullptr; }
  T* release() noexcept {
    T* value = pointer_;
    pointer_ = nullptr;
    return value;
  }
  void reset(T* pointer = nullptr) noexcept {
    T* previous = pointer_;
    pointer_ = pointer;
    if (previous) previous->Release();
  }

 private:
  T* pointer_ = nullptr;
};

void check_sdk(HRESULT result, const char* operation) {
  if (result != S_OK) {
    std::ostringstream message;
    message << operation << " failed (HRESULT 0x" << std::hex
            << static_cast<std::uint32_t>(result) << ')';
    throw std::runtime_error(message.str());
  }
}

BMDDisplayMode parse_mode(const std::string& requested) {
  struct Mode {
    const char* name;
    const char* legacy_name;
    BMDDisplayMode value;
  };
  static const Mode modes[] = {
      {"ntsc", "eNTSC", bmdModeNTSC},
      {"ntsc2398", "eNTSC2398", bmdModeNTSC2398},
      {"pal", "ePAL", bmdModePAL},
      {"ntscp", "eNTSCp", bmdModeNTSCp},
      {"palp", "ePALp", bmdModePALp},
      {"1080p2398", "eHD1080p2398", bmdModeHD1080p2398},
      {"1080p24", "eHD1080p24", bmdModeHD1080p24},
      {"1080p25", "eHD1080p25", bmdModeHD1080p25},
      {"1080p2997", "eHD1080p2997", bmdModeHD1080p2997},
      {"1080p30", "eHD1080p30", bmdModeHD1080p30},
      {"1080i50", "eHD1080i50", bmdModeHD1080i50},
      {"1080i5994", "eHD1080i5994", bmdModeHD1080i5994},
      {"1080i60", "eHD1080i60", bmdModeHD1080i6000},
      {"1080p50", "eHD1080p50", bmdModeHD1080p50},
      {"1080p5994", "eHD1080p5994", bmdModeHD1080p5994},
      {"1080p60", "eHD1080p60", bmdModeHD1080p6000},
      {"720p50", "eHD720p50", bmdModeHD720p50},
      {"720p5994", "eHD720p5994", bmdModeHD720p5994},
      {"720p60", "eHD720p60", bmdModeHD720p60},
  };
  for (const auto& mode : modes) {
    if (requested == mode.name || requested == mode.legacy_name)
      return mode.value;
  }
  throw std::invalid_argument("Unsupported capture mode: " + requested);
}

std::int64_t system_time_ns() noexcept {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}

}  // namespace

struct Frame::Impl {
  ComPtr<IDeckLinkVideoInputFrame> frame;
  int width = 0;
  int height = 0;
  std::size_t row_bytes = 0;
  std::size_t packed_row_bytes = 0;
  std::size_t packed_size = 0;
  std::int64_t timestamp_ns = 0;
  std::uint64_t sequence = 0;
};

Frame::Frame(IDeckLinkVideoInputFrame* frame, std::int64_t timestamp_ns,
             std::uint64_t sequence)
    : impl_(new Impl) {
  if (!frame) throw std::invalid_argument("Null DeckLink video frame");
  if (frame->GetPixelFormat() != bmdFormat8BitBGRA)
    throw std::runtime_error("Capture accepts BGRA8 frames only");
  const long width = frame->GetWidth();
  const long height = frame->GetHeight();
  const long stride = frame->GetRowBytes();
  if (width <= 0 || height <= 0 || stride <= 0 ||
      width > std::numeric_limits<int>::max() ||
      height > std::numeric_limits<int>::max())
    throw std::runtime_error("Invalid DeckLink frame dimensions or row stride");
  const auto max_size = std::numeric_limits<std::size_t>::max();
  const auto w = static_cast<std::size_t>(width);
  const auto h = static_cast<std::size_t>(height);
  const auto row_bytes = static_cast<std::size_t>(stride);
  if (w > max_size / 4 || row_bytes < w * 4 || row_bytes > max_size / h)
    throw std::runtime_error("Invalid or overflowing DeckLink frame row stride");
  impl_->width = static_cast<int>(width);
  impl_->height = static_cast<int>(height);
  impl_->row_bytes = row_bytes;
  impl_->packed_row_bytes = w * 4;
  impl_->packed_size = w * 4 * h;
  impl_->timestamp_ns = timestamp_ns;
  impl_->sequence = sequence;
  frame->AddRef();
  impl_->frame.reset(frame);
}

Frame::~Frame() = default;
Frame::Frame(Frame&&) noexcept = default;
Frame& Frame::operator=(Frame&&) noexcept = default;
int Frame::width() const noexcept { return impl_->width; }
int Frame::height() const noexcept { return impl_->height; }
std::size_t Frame::packed_size() const noexcept { return impl_->packed_size; }
std::int64_t Frame::timestamp_ns() const noexcept { return impl_->timestamp_ns; }
std::uint64_t Frame::sequence() const noexcept { return impl_->sequence; }

void Frame::copy_to(std::uint8_t* destination, std::size_t capacity) const {
  if (!destination || capacity < impl_->packed_size)
    throw std::invalid_argument("Destination is smaller than the packed BGRA frame");
  void* bytes = nullptr;
  check_sdk(impl_->frame->GetBytes(&bytes), "IDeckLinkVideoInputFrame::GetBytes");
  if (!bytes) throw std::runtime_error("DeckLink returned a null frame buffer");
  const auto* source = static_cast<const std::uint8_t*>(bytes);
  if (impl_->row_bytes == impl_->packed_row_bytes) {
    std::memcpy(destination, source, impl_->packed_size);
    return;
  }
  for (int row = 0; row < impl_->height; ++row) {
    const auto offset = static_cast<std::size_t>(row);
    std::memcpy(destination + offset * impl_->packed_row_bytes,
                source + offset * impl_->row_bytes, impl_->packed_row_bytes);
  }
}

struct FrameHandoff::Impl {
  std::atomic<ULONG> references{1};
  mutable std::mutex mutex;
  std::condition_variable available;
  std::unique_ptr<Frame> pending;
  CaptureStats stats;
};

FrameHandoff::FrameHandoff() : impl_(new Impl) {}
FrameHandoff::~FrameHandoff() = default;

void FrameHandoff::begin() {
  std::unique_ptr<Frame> previous;
  {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    previous = std::move(impl_->pending);
    impl_->stats = CaptureStats{};
    impl_->stats.running = true;
  }
}

void FrameHandoff::close() noexcept {
  std::unique_ptr<Frame> previous;
  {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->stats.running = false;
    impl_->stats.signal_present = false;
    if (impl_->pending) ++impl_->stats.discarded;
    previous = std::move(impl_->pending);
  }
  impl_->available.notify_all();
}

std::unique_ptr<Frame> FrameHandoff::wait_for_frame(
    std::chrono::milliseconds timeout) {
  if (timeout.count() < 0)
    throw std::invalid_argument("Frame timeout cannot be negative");
  std::unique_lock<std::mutex> lock(impl_->mutex);
  if (!impl_->available.wait_for(lock, timeout, [this] {
        return !impl_->stats.running || impl_->pending != nullptr;
      })) {
    ++impl_->stats.timeouts;
    return nullptr;
  }
  if (!impl_->stats.running) return nullptr;
  ++impl_->stats.frames_delivered;
  return std::move(impl_->pending);
}

CaptureStats FrameHandoff::stats() const {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  auto value = impl_->stats;
  value.pending_frames = impl_->pending ? 1 : 0;
  return value;
}

HRESULT FrameHandoff::QueryInterface(REFIID iid, LPVOID* object) {
  if (!object) return E_POINTER;
  *object = nullptr;
  // Avoid the legacy SDK's GNU compound-literal IID_IUnknown macro so this
  // file also compiles in strict ISO C++ mode.
  static const REFIID unknown = {0, 0, 0, 0, 0, 0, 0, 0,
                                0xC0, 0, 0, 0, 0, 0, 0, 0x46};
  if (std::memcmp(&iid, &unknown, sizeof(REFIID)) == 0 ||
      std::memcmp(&iid, &IID_IDeckLinkInputCallback, sizeof(REFIID)) == 0) {
    *object = static_cast<IDeckLinkInputCallback*>(this);
    AddRef();
    return S_OK;
  }
  return E_NOINTERFACE;
}

ULONG FrameHandoff::AddRef() {
  return impl_->references.fetch_add(1, std::memory_order_relaxed) + 1;
}

ULONG FrameHandoff::Release() {
  const auto count =
      impl_->references.fetch_sub(1, std::memory_order_acq_rel) - 1;
  if (count == 0) delete this;
  return count;
}

HRESULT FrameHandoff::VideoInputFormatChanged(
    BMDVideoInputFormatChangedEvents, IDeckLinkDisplayMode*,
    BMDDetectedVideoInputFormatFlags) {
  std::unique_ptr<Frame> previous;
  {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (!impl_->stats.running) return S_OK;
    ++impl_->stats.format_changes;
    impl_->stats.running = false;
    impl_->stats.signal_present = false;
    if (impl_->pending) ++impl_->stats.discarded;
    previous = std::move(impl_->pending);
  }
  impl_->available.notify_all();
  // Mode changes require an explicit stop/restart; never reconfigure devices
  // from the SDK's callback thread.
  return S_OK;
}

HRESULT FrameHandoff::VideoInputFrameArrived(IDeckLinkVideoInputFrame* video,
                                           IDeckLinkAudioInputPacket*) {
  const auto arrival_time = system_time_ns();
  std::unique_ptr<Frame> previous;
  {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (!impl_->stats.running) return S_OK;
    ++impl_->stats.callbacks;
    if (!video) return S_OK;  // Audio-only callbacks are not video frames.
    if (video->GetFlags() & bmdFrameHasNoInputSource) {
      ++impl_->stats.no_signal;
      impl_->stats.signal_present = false;
      if (impl_->pending) ++impl_->stats.discarded;
      previous = std::move(impl_->pending);
    } else {
      try {
        std::unique_ptr<Frame> next(
            new Frame(video, arrival_time, impl_->stats.frames_received + 1));
        ++impl_->stats.frames_received;
        impl_->stats.signal_present = true;
        if (impl_->pending) ++impl_->stats.overwritten;
        previous = std::move(impl_->pending);
        impl_->pending = std::move(next);
      } catch (...) {
        // Never let validation/allocation exceptions cross the SDK boundary.
        ++impl_->stats.invalid_frames;
        impl_->stats.signal_present = false;
        if (impl_->pending) ++impl_->stats.discarded;
        previous = std::move(impl_->pending);
      }
    }
  }
  impl_->available.notify_one();
  // Release the overwritten SDK frame outside the handoff mutex.
  return S_OK;
}

struct DeckLinkCapture::Impl {
  explicit Impl(IteratorFactory factory_value)
      : factory(factory_value), callback(new FrameHandoff) {}

  IteratorFactory factory;
  mutable std::mutex lifecycle;
  ComPtr<FrameHandoff> callback;
  ComPtr<IDeckLink> device;
  ComPtr<IDeckLinkInput> input;
  bool enable_attempted = false;
  bool callback_attempted = false;
  bool start_attempted = false;
  bool started = false;
  int width = 0;
  int height = 0;
  std::uint64_t sdk_errors = 0;

  // Caller holds lifecycle. Close the handoff before any potentially blocking
  // SDK operation, so waiting workers can leave immediately.
  void cleanup() noexcept {
    callback->close();
    if (input) {
      if (start_attempted && input->StopStreams() != S_OK) ++sdk_errors;
      if (callback_attempted && input->SetCallback(nullptr) != S_OK) ++sdk_errors;
      if (enable_attempted && input->DisableVideoInput() != S_OK) ++sdk_errors;
    }
    start_attempted = false;
    callback_attempted = false;
    enable_attempted = false;
    started = false;
    input.reset();
    device.reset();
  }
};

DeckLinkCapture::DeckLinkCapture(IteratorFactory factory)
    : impl_(new Impl(factory)) {
  if (!factory) throw std::invalid_argument("Null DeckLink iterator factory");
}

DeckLinkCapture::~DeckLinkCapture() { stop(); }

void DeckLinkCapture::start(unsigned device_index, const std::string& mode) {
  const auto requested_mode = parse_mode(mode);
  std::lock_guard<std::mutex> lock(impl_->lifecycle);
  if (impl_->started)
    throw std::logic_error("Capture is already started; stop it before restarting");
  impl_->sdk_errors = 0;
  impl_->width = 0;
  impl_->height = 0;
  impl_->callback->begin();
  try {
    ComPtr<IDeckLinkIterator> iterator(impl_->factory());
    if (!iterator)
      throw std::runtime_error(
          "Cannot create DeckLink iterator; check the installed Desktop Video driver");
    for (unsigned index = 0;; ++index) {
      IDeckLink* device = nullptr;
      const auto result = iterator->Next(&device);
      ComPtr<IDeckLink> candidate(device);
      check_sdk(result, "IDeckLinkIterator::Next (requested device not found)");
      if (!candidate) throw std::runtime_error("DeckLink iterator returned a null device");
      if (index == device_index) {
        impl_->device = std::move(candidate);
        break;
      }
    }

    IDeckLinkInput* input = nullptr;
    const auto query_result = impl_->device->QueryInterface(
        IID_IDeckLinkInput, reinterpret_cast<void**>(&input));
    impl_->input.reset(input);
    check_sdk(query_result, "IDeckLink::QueryInterface(IDeckLinkInput)");
    if (!impl_->input)
      throw std::runtime_error("DeckLink returned a null input interface");

    BMDDisplayModeSupport support = bmdDisplayModeNotSupported;
    IDeckLinkDisplayMode* display_mode = nullptr;
    const auto support_result = impl_->input->DoesSupportVideoMode(
        requested_mode, bmdFormat8BitBGRA, bmdVideoInputFlagDefault, &support,
        &display_mode);
    ComPtr<IDeckLinkDisplayMode> selected_mode(display_mode);
    check_sdk(support_result, "IDeckLinkInput::DoesSupportVideoMode");
    if (support == bmdDisplayModeNotSupported)
      throw std::runtime_error("Device does not support requested BGRA8 mode: " + mode);
    if (!selected_mode)
      throw std::runtime_error("DeckLink returned no display mode for a supported input");
    const long width = selected_mode->GetWidth();
    const long height = selected_mode->GetHeight();
    if (width <= 0 || height <= 0 || width > std::numeric_limits<int>::max() ||
        height > std::numeric_limits<int>::max())
      throw std::runtime_error("DeckLink returned invalid display-mode dimensions");
    impl_->width = static_cast<int>(width);
    impl_->height = static_cast<int>(height);

    impl_->enable_attempted = true;
    check_sdk(impl_->input->EnableVideoInput(requested_mode, bmdFormat8BitBGRA,
                                            bmdVideoInputFlagDefault),
              "IDeckLinkInput::EnableVideoInput");
    impl_->callback_attempted = true;
    check_sdk(impl_->input->SetCallback(impl_->callback.get()),
              "IDeckLinkInput::SetCallback");
    impl_->start_attempted = true;
    check_sdk(impl_->input->StartStreams(), "IDeckLinkInput::StartStreams");
    impl_->started = true;
  } catch (...) {
    ++impl_->sdk_errors;
    impl_->cleanup();
    throw;
  }
}

std::unique_ptr<Frame> DeckLinkCapture::wait_for_frame(
    std::chrono::milliseconds timeout) {
  // callback is owned for this object's entire lifetime. Like other member
  // functions, a wait must complete before the DeckLinkCapture is destroyed.
  return impl_->callback->wait_for_frame(timeout);
}

void DeckLinkCapture::stop() noexcept {
  std::lock_guard<std::mutex> lock(impl_->lifecycle);
  impl_->cleanup();
}

CaptureStats DeckLinkCapture::stats() const {
  std::lock_guard<std::mutex> lock(impl_->lifecycle);
  auto value = impl_->callback->stats();
  value.sdk_errors += impl_->sdk_errors;
  return value;
}

int DeckLinkCapture::width() const {
  std::lock_guard<std::mutex> lock(impl_->lifecycle);
  return impl_->width;
}

int DeckLinkCapture::height() const {
  std::lock_guard<std::mutex> lock(impl_->lifecycle);
  return impl_->height;
}

}  // namespace endoscope_capture
