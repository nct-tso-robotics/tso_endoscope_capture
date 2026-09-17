#pragma once

#include <cstddef>
#include <memory>
#include <stdexcept>
#include <vector>

namespace endoscope_capture {

// The pool owns one reference. A slot is writable only after all asynchronous
// publishers/subscribers release their references. Exhaustion never grows RAM.
// Acquire is called by a single publishing thread.
template <class Pointer>
class MessagePool {
 public:
  template <class Factory>
  MessagePool(std::size_t capacity, Factory factory) {
    if (capacity == 0) throw std::invalid_argument("Message pool cannot be empty");
    for (std::size_t i = 0; i < capacity; ++i) slots_.push_back(factory());
  }

  Pointer acquire() {
    for (auto& slot : slots_) {
      if (slot.use_count() == 1) return slot;
    }
    return {};
  }

 private:
  std::vector<Pointer> slots_;
};
}  // namespace endoscope_capture
