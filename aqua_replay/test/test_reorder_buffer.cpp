// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include <vector>

#include "aqua_replay/reorder_buffer.hpp"

namespace
{

aqua_replay::QueuedMessage make(int64_t key, uint64_t seq)
{
  aqua_replay::QueuedMessage q;
  q.order_key_ns = key;
  q.receive_ns = key;
  q.sequence = seq;
  return q;
}

TEST(ReorderBuffer, ReleasesInKeyOrderOnlyPastWatermark)
{
  aqua_replay::ReorderBuffer buffer;
  buffer.push(make(300, 0));
  buffer.push(make(100, 1));
  buffer.push(make(200, 2));
  std::vector<aqua_replay::QueuedMessage> out;
  buffer.pop_ready(150, out);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].order_key_ns, 100);
  out.clear();
  buffer.pop_ready(300, out);
  ASSERT_EQ(out.size(), 2u);
  EXPECT_EQ(out[0].order_key_ns, 200);
  EXPECT_EQ(out[1].order_key_ns, 300);
  EXPECT_TRUE(buffer.empty());
}

TEST(ReorderBuffer, EqualKeysKeepReadOrder)
{
  aqua_replay::ReorderBuffer buffer;
  for (uint64_t seq = 0; seq < 50; ++seq) {
    buffer.push(make(42, seq));
  }
  std::vector<aqua_replay::QueuedMessage> out;
  buffer.drain(out);
  ASSERT_EQ(out.size(), 50u);
  for (uint64_t seq = 0; seq < 50; ++seq) {
    EXPECT_EQ(out[seq].sequence, seq);
  }
}

TEST(Fnv1a64, OrderSensitiveAndStable)
{
  const uint8_t a[] = {1, 2, 3};
  const uint8_t b[] = {3, 2, 1};
  aqua_replay::Fnv1a64 ha;
  aqua_replay::Fnv1a64 hb;
  ha.update(a, sizeof(a));
  hb.update(b, sizeof(b));
  EXPECT_NE(ha.value(), hb.value());
  aqua_replay::Fnv1a64 empty;
  EXPECT_EQ(empty.value(), 1469598103934665603ULL);
}

}  // namespace
