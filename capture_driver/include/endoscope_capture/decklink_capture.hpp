// Maintained DeckLink capture backend. See NOTICE.md for the predecessor's
// attribution and vendor/decklink for the SDK's retained license notices.
// The interface exposes SDK frame ownership and capture lifecycle operations.
#pragma once

#include <DeckLinkAPI.h>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

namespace endoscope_capture {

struct CaptureStats {
  std::uint64_t callbacks = 0;
  // Accepted valid frames = delivered + overwritten + discarded + pending.
  // Overwritten means replaced by a newer valid frame; discarded means a
  // pending frame cleared on signal loss, invalid input, format change, or stop.
  std::uint64_t frames_received = 0;
  std::uint64_t frames_delivered = 0;
  std::uint64_t overwritten = 0;
  std::uint64_t discarded = 0;
  std::uint64_t pending_frames = 0;
  std::uint64_t no_signal = 0;
  std::uint64_t format_changes = 0;
  std::uint64_t invalid_frames = 0;
  std::uint64_t timeouts = 0;
  std::uint64_t sdk_errors = 0;
  bool signal_present = false;
  bool running = false;
};

// One retained SDK frame. This object owns an SDK reference until destruction;
// moving it transfers that reference. The timestamp is callback arrival on the
// host's system clock, NOT camera exposure time or a mapped hardware timestamp.
class Frame {
 public:
  Frame(IDeckLinkVideoInputFrame* frame, std::int64_t timestamp_ns,
        std::uint64_t sequence);
  ~Frame();
  Frame(Frame&&) noexcept;
  Frame& operator=(Frame&&) noexcept;
  Frame(const Frame&) = delete;
  Frame& operator=(const Frame&) = delete;

  int width() const noexcept;
  int height() const noexcept;
  std::size_t packed_size() const noexcept;
  std::int64_t timestamp_ns() const noexcept;
  std::uint64_t sequence() const noexcept;

  // Copies BGRA rows directly into caller-owned storage, removing SDK padding.
  // Throws on an invalid destination or an SDK GetBytes failure. The caller
  // must not reuse published storage while downstream readers still hold it.
  void copy_to(std::uint8_t* destination, std::size_t capacity) const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

// A bounded latest-frame handoff, exposed so ownership, signal loss, and stop
// behavior can be exercised with fake SDK frames. Create with new and manage
// its COM reference through AddRef/Release (initial reference count is one).
class FrameHandoff final : public IDeckLinkInputCallback {
 public:
  FrameHandoff();
  ~FrameHandoff() override;
  FrameHandoff(const FrameHandoff&) = delete;
  FrameHandoff& operator=(const FrameHandoff&) = delete;

  // Starts a fresh session and resets counters. Call after the previous
  // session's consumers have stopped; begin is not a concurrent restart API.
  void begin();
  // Discards any waiting frame and wakes consumers. Counters remain readable.
  // An unexpected format-change event also closes the handoff until begin().
  void close() noexcept;
  std::unique_ptr<Frame> wait_for_frame(std::chrono::milliseconds timeout);
  CaptureStats stats() const;

  HRESULT STDMETHODCALLTYPE QueryInterface(REFIID iid, LPVOID* object) override;
  ULONG STDMETHODCALLTYPE AddRef() override;
  ULONG STDMETHODCALLTYPE Release() override;
  HRESULT VideoInputFormatChanged(BMDVideoInputFormatChangedEvents events,
                                  IDeckLinkDisplayMode* mode,
                                  BMDDetectedVideoInputFormatFlags flags) override;
  HRESULT VideoInputFrameArrived(IDeckLinkVideoInputFrame* video,
                                 IDeckLinkAudioInputPacket* audio) override;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

class DeckLinkCapture {
 public:
  using IteratorFactory = IDeckLinkIterator* (*)();

  // Construction does not access a capture device. The injectable factory is
  // used by hardware-free tests and invoked only by start().
  explicit DeckLinkCapture(
      IteratorFactory factory = CreateDeckLinkIteratorInstance);
  ~DeckLinkCapture();
  DeckLinkCapture(const DeckLinkCapture&) = delete;
  DeckLinkCapture& operator=(const DeckLinkCapture&) = delete;

  // Opens exactly one device in BGRA8. Throws on unsupported modes or SDK
  // errors and rolls back partial startup. Legacy eHD... spellings are accepted.
  void start(unsigned device_index = 0,
             const std::string& mode = "1080p5994");
  std::unique_ptr<Frame> wait_for_frame(std::chrono::milliseconds timeout);
  // Idempotent; SDK cleanup failures are counted in stats().sdk_errors.
  void stop() noexcept;
  CaptureStats stats() const;
  int width() const;
  int height() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace endoscope_capture
