// SPDX-License-Identifier: Apache-2.0
// Bounded-latency reorder buffer for offline replay.
//
// Bags are read in receive order, but estimators should consume measurements
// in measurement (header stamp) order. Messages are held until the receive
// watermark has moved `window` past their ordering key, then released in
// (key, read sequence) order, so ties resolve by read order and the output is
// a pure function of the bag contents.
#pragma once

#include <cstdint>
#include <memory>
#include <queue>
#include <tuple>
#include <vector>

namespace aqua_replay
{

struct QueuedMessage
{
  int64_t order_key_ns{0};
  int64_t receive_ns{0};
  uint64_t sequence{0};
  size_t topic_index{0};
  std::shared_ptr<void> message;
};

class ReorderBuffer
{
public:
  void push(QueuedMessage message)
  {
    queue_.push(std::move(message));
  }

  // Release every message whose key is <= watermark_ns, in key order.
  void pop_ready(int64_t watermark_ns, std::vector<QueuedMessage> & out)
  {
    while (!queue_.empty() && queue_.top().order_key_ns <= watermark_ns) {
      out.push_back(queue_.top());
      queue_.pop();
    }
  }

  void drain(std::vector<QueuedMessage> & out)
  {
    while (!queue_.empty()) {
      out.push_back(queue_.top());
      queue_.pop();
    }
  }

  size_t size() const {return queue_.size();}
  bool empty() const {return queue_.empty();}

private:
  struct Later
  {
    bool operator()(const QueuedMessage & a, const QueuedMessage & b) const
    {
      return std::tie(a.order_key_ns, a.sequence) > std::tie(b.order_key_ns, b.sequence);
    }
  };

  std::priority_queue<QueuedMessage, std::vector<QueuedMessage>, Later> queue_;
};

// FNV-1a 64-bit running digest; used as a cheap determinism fingerprint of
// serialized output streams (not a security hash).
class Fnv1a64
{
public:
  void update(const uint8_t * data, size_t size)
  {
    for (size_t i = 0; i < size; ++i) {
      hash_ ^= data[i];
      hash_ *= 1099511628211ULL;
    }
  }

  void update_u64(uint64_t value)
  {
    uint8_t bytes[8];
    for (int i = 0; i < 8; ++i) {
      bytes[i] = static_cast<uint8_t>(value >> (8 * i));
    }
    update(bytes, sizeof(bytes));
  }

  uint64_t value() const {return hash_;}

private:
  uint64_t hash_{1469598103934665603ULL};
};

}  // namespace aqua_replay
