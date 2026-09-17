#include "endoscope_capture/message_pool.hpp"

#include <chrono>
#include <cstdint>
#include <future>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <vector>

namespace {
struct Message {
  std::vector<std::uint8_t> bytes = std::vector<std::uint8_t>(4096, 0);
};

using Pointer = std::shared_ptr<Message>;
using Pool = endoscope_capture::MessagePool<Pointer>;

void require(bool condition, const char* reason) {
  if (!condition) throw std::runtime_error(reason);
}

void retained_consumer_blocks_reuse() {
  unsigned allocations = 0;
  Pool pool(2, [&] {
    ++allocations;
    return std::make_shared<Message>();
  });
  auto first = pool.acquire();
  require(static_cast<bool>(first), "First slot missing");
  first->bytes.assign(first->bytes.size(), 17);
  const auto first_address = first.get();
  std::shared_ptr<const Message> retained = first;
  first.reset();
  auto second = pool.acquire();
  require(second && second.get() != first_address, "Retained slot reused");
  second->bytes.assign(second->bytes.size(), 99);
  require(!pool.acquire(), "Exhausted pool unexpectedly grew");
  require(retained->bytes == std::vector<std::uint8_t>(4096, 17),
          "Published frame changed while consumer retained it");
  retained.reset();
  auto recycled = pool.acquire();
  require(recycled && recycled.get() == first_address, "Released slot not recycled");
  require(allocations == 2, "Recycling allocated additional messages");
}

void asynchronous_consumer_blocks_reuse() {
  Pool pool(1, [] { return std::make_shared<Message>(); });
  auto frame = pool.acquire();
  frame->bytes[0] = 42;
  std::promise<void> release_consumer;
  auto release = release_consumer.get_future();
  auto reader = std::async(std::launch::async,
                          [held = std::shared_ptr<const Message>(frame),
                           ready = std::move(release)]() mutable {
                            if (ready.wait_for(std::chrono::seconds(2)) !=
                                std::future_status::ready) {
                              throw std::runtime_error("Consumer release timed out");
                            }
                            require(held->bytes[0] == 42, "In-flight buffer mutated");
                            held.reset();
                          });
  frame.reset();
  for (unsigned attempt = 0; attempt < 1000; ++attempt) {
    require(!pool.acquire(), "In-flight slot was recycled");
  }
  release_consumer.set_value();
  reader.get();
  require(static_cast<bool>(pool.acquire()), "Asynchronously released slot unavailable");
}

void consumer_outlives_pool() {
  Pointer retained;
  std::weak_ptr<Message> witness;
  {
    Pool pool(1, [] { return std::make_shared<Message>(); });
    retained = pool.acquire();
    retained->bytes[0] = 23;
    witness = retained;
  }
  require(retained->bytes[0] == 23, "Pool destruction invalidated retained message");
  retained.reset();
  require(witness.expired(), "Message leaked after final consumer release");
}

void reject_empty_pool() {
  bool rejected = false;
  try {
    Pool pool(0, [] { return std::make_shared<Message>(); });
  } catch (const std::invalid_argument&) {
    rejected = true;
  }
  require(rejected, "Empty pool accepted");
}
}  // namespace

int main() {
  try {
    retained_consumer_blocks_reuse();
    asynchronous_consumer_blocks_reuse();
    consumer_outlives_pool();
    reject_empty_pool();
    std::cout << "4 message-pool tests passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "Message-pool test failed: " << error.what() << '\n';
    return 1;
  }
}
