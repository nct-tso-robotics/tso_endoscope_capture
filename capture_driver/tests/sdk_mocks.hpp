#pragma once

#include <DeckLinkAPI.h>

#include <atomic>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace capture_test {

inline bool same_id(REFIID left, REFIID right) {
  return std::memcmp(&left, &right, sizeof(left)) == 0;
}

template <typename Interface> class RefCounted : public Interface {
 public:
  explicit RefCounted(std::shared_ptr<std::atomic<unsigned>> destroyed = {})
      : destroyed_(std::move(destroyed)) {}
  HRESULT QueryInterface(REFIID iid, void** value) override {
    if (!value) return E_POINTER;
    *value = nullptr;
    const REFIID unknown = IID_IUnknown;
    if (!same_id(iid, unknown)) return E_NOINTERFACE;
    *value = static_cast<Interface*>(this);
    AddRef();
    return S_OK;
  }
  ULONG AddRef() override { return ++references_; }
  ULONG Release() override {
    const auto remaining = --references_;
    if (!remaining) delete this;
    return remaining;
  }
  unsigned references() const { return references_.load(); }

 protected:
  virtual ~RefCounted() {
    if (destroyed_) ++*destroyed_;
  }

 private:
  std::atomic<unsigned> references_{1};
  std::shared_ptr<std::atomic<unsigned>> destroyed_;
};

class MockFrame final : public RefCounted<IDeckLinkVideoInputFrame> {
 public:
  explicit MockFrame(long width = 3, long height = 2, long stride = 16,
                     std::shared_ptr<std::atomic<unsigned>> destroyed = {})
      : RefCounted(std::move(destroyed)), width(width), height(height), stride(stride),
        bytes(stride > 0 && height > 0 && stride < 100000 && height < 100000
                  ? static_cast<std::size_t>(stride * height)
                  : 0,
              0xEE) {
    if (!bytes.empty() && width > 0 && width <= stride / 4) {
      for (long row = 0; row < height; ++row)
        for (long col = 0; col < width * 4; ++col)
          bytes[static_cast<std::size_t>(row * stride + col)] =
              static_cast<std::uint8_t>(row * width * 4 + col);
    }
  }
  long GetWidth() override { return width; }
  long GetHeight() override { return height; }
  long GetRowBytes() override { return stride; }
  BMDPixelFormat GetPixelFormat() override { return format; }
  BMDFrameFlags GetFlags() override { return flags; }
  HRESULT GetBytes(void** out) override {
    ++get_bytes_calls;
    *out = null_bytes ? nullptr : bytes.data();
    return bytes_result;
  }
  HRESULT GetTimecode(BMDTimecodeFormat, IDeckLinkTimecode**) override {
    return E_NOTIMPL;
  }
  HRESULT GetAncillaryData(IDeckLinkVideoFrameAncillary**) override {
    return E_NOTIMPL;
  }
  HRESULT GetStreamTime(BMDTimeValue* time, BMDTimeValue* duration,
                        BMDTimeScale scale) override {
    *time = scale;
    *duration = scale / 60;
    return S_OK;
  }
  HRESULT GetHardwareReferenceTimestamp(BMDTimeScale scale, BMDTimeValue* time,
                                         BMDTimeValue* duration) override {
    return GetStreamTime(time, duration, scale);
  }

  long width;
  long height;
  long stride;
  BMDPixelFormat format{bmdFormat8BitBGRA};
  BMDFrameFlags flags{bmdFrameFlagDefault};
  HRESULT bytes_result{S_OK};
  bool null_bytes{false};
  unsigned get_bytes_calls{0};
  std::vector<std::uint8_t> bytes;
};

class MockInput;
struct DeviceState {
  std::string fail;
  bool install_callback_before_failure{false};
  std::vector<std::string> calls;
  std::shared_ptr<std::atomic<unsigned>> iterators_destroyed{
      std::make_shared<std::atomic<unsigned>>(0)};
  std::shared_ptr<std::atomic<unsigned>> devices_destroyed{
      std::make_shared<std::atomic<unsigned>>(0)};
  std::shared_ptr<std::atomic<unsigned>> inputs_destroyed{
      std::make_shared<std::atomic<unsigned>>(0)};
  std::shared_ptr<std::atomic<unsigned>> modes_destroyed{
      std::make_shared<std::atomic<unsigned>>(0)};
  MockInput* input{nullptr};
  HRESULT result(const char* operation) {
    calls.emplace_back(operation);
    return fail == operation ? E_FAIL : S_OK;
  }
};

class MockMode final : public RefCounted<IDeckLinkDisplayMode> {
 public:
  explicit MockMode(const std::shared_ptr<DeviceState>& state)
      : RefCounted(state->modes_destroyed) {}
  HRESULT GetName(const char** name) override {
    *name = ::strdup("1080p59.94");
    return *name ? S_OK : E_OUTOFMEMORY;
  }
  BMDDisplayMode GetDisplayMode() override { return bmdModeHD1080p5994; }
  long GetWidth() override { return 1920; }
  long GetHeight() override { return 1080; }
  HRESULT GetFrameRate(BMDTimeValue* duration, BMDTimeScale* scale) override {
    *duration = 1001;
    *scale = 60000;
    return S_OK;
  }
  BMDFieldDominance GetFieldDominance() override { return bmdProgressiveFrame; }
  BMDDisplayModeFlags GetFlags() override { return 0; }
};

class MockModeIterator final : public RefCounted<IDeckLinkDisplayModeIterator> {
 public:
  explicit MockModeIterator(std::shared_ptr<DeviceState> state)
      : state_(std::move(state)) {}
  HRESULT Next(IDeckLinkDisplayMode** mode) override {
    *mode = nullptr;
    if (seen_) return S_FALSE;
    seen_ = true;
    *mode = new MockMode(state_);
    return S_OK;
  }

 private:
  std::shared_ptr<DeviceState> state_;
  bool seen_{false};
};

class MockInput final : public RefCounted<IDeckLinkInput> {
 public:
  explicit MockInput(std::shared_ptr<DeviceState> state)
      : RefCounted(state->inputs_destroyed), state_(std::move(state)) {
    state_->input = this;
  }
  ~MockInput() override {
    if (callback_) callback_->Release();
    state_->input = nullptr;
  }
  HRESULT DoesSupportVideoMode(BMDDisplayMode, BMDPixelFormat,
                              BMDVideoInputFlags, BMDDisplayModeSupport* support,
                              IDeckLinkDisplayMode** mode) override {
    *support = state_->fail == "unsupported" ? bmdDisplayModeNotSupported
                                             : bmdDisplayModeSupported;
    if (mode)
      *mode = state_->fail == "null_mode" ? nullptr : new MockMode(state_);
    return state_->result("support");
  }
  HRESULT GetDisplayModeIterator(IDeckLinkDisplayModeIterator** iterator) override {
    *iterator = new MockModeIterator(state_);
    return state_->result("mode_iterator");
  }
  HRESULT SetScreenPreviewCallback(IDeckLinkScreenPreviewCallback*) override {
    return E_NOTIMPL;
  }
  HRESULT EnableVideoInput(BMDDisplayMode, BMDPixelFormat,
                           BMDVideoInputFlags) override {
    return state_->result("enable");
  }
  HRESULT DisableVideoInput() override { return state_->result("disable"); }
  HRESULT GetAvailableVideoFrameCount(uint32_t* count) override {
    *count = 0;
    return S_OK;
  }
  HRESULT SetVideoInputFrameMemoryAllocator(IDeckLinkMemoryAllocator*) override {
    return E_NOTIMPL;
  }
  HRESULT EnableAudioInput(BMDAudioSampleRate, BMDAudioSampleType,
                           uint32_t) override {
    return E_NOTIMPL;
  }
  HRESULT DisableAudioInput() override { return S_OK; }
  HRESULT GetAvailableAudioSampleFrameCount(uint32_t* count) override {
    *count = 0;
    return S_OK;
  }
  HRESULT StartStreams() override { return state_->result("start"); }
  HRESULT StopStreams() override { return state_->result("stop"); }
  HRESULT PauseStreams() override { return E_NOTIMPL; }
  HRESULT FlushStreams() override { return S_OK; }
  HRESULT SetCallback(IDeckLinkInputCallback* callback) override {
    const auto result = state_->result(callback ? "callback" : "detach");
    if (result != S_OK && !state_->install_callback_before_failure) return result;
    if (callback) callback->AddRef();
    if (callback_) callback_->Release();
    callback_ = callback;
    return result;
  }
  HRESULT GetHardwareReferenceClock(BMDTimeScale, BMDTimeValue*, BMDTimeValue*,
                                     BMDTimeValue*) override {
    return E_NOTIMPL;
  }
  void arrive(IDeckLinkVideoInputFrame* frame) {
    if (callback_) callback_->VideoInputFrameArrived(frame, nullptr);
  }

 private:
  std::shared_ptr<DeviceState> state_;
  IDeckLinkInputCallback* callback_{nullptr};
};

class MockDevice final : public RefCounted<IDeckLink> {
 public:
  explicit MockDevice(std::shared_ptr<DeviceState> state)
      : RefCounted(state->devices_destroyed), state_(std::move(state)) {}
  HRESULT QueryInterface(REFIID iid, void** value) override {
    if (!value) return E_POINTER;
    *value = nullptr;
    if (!same_id(iid, IID_IDeckLinkInput))
      return RefCounted::QueryInterface(iid, value);
    const auto result = state_->result("input");
    if (result == S_OK) *value = static_cast<IDeckLinkInput*>(new MockInput(state_));
    return result;
  }
  HRESULT GetModelName(const char** name) override {
    *name = ::strdup("Mock capture device");
    return *name ? S_OK : E_OUTOFMEMORY;
  }
  HRESULT GetDisplayName(const char** name) override { return GetModelName(name); }

 private:
  std::shared_ptr<DeviceState> state_;
};

class MockIterator final : public RefCounted<IDeckLinkIterator> {
 public:
  explicit MockIterator(std::shared_ptr<DeviceState> state)
      : RefCounted(state->iterators_destroyed), state_(std::move(state)) {}
  HRESULT Next(IDeckLink** device) override {
    *device = nullptr;
    if (seen_ || state_->fail == "no_device") return S_FALSE;
    seen_ = true;
    const auto result = state_->result("next");
    if (result == S_OK) *device = new MockDevice(state_);
    return result;
  }

 private:
  std::shared_ptr<DeviceState> state_;
  bool seen_{false};
};

}  // namespace capture_test
