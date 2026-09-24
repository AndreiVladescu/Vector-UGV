// Golden frames generated from protocol/vector.dbc with cantools, so this code and
// anything generated from the DBC (firmware) agree bit for bit.
#include <gtest/gtest.h>

#include <string>

#include "vector_hw/protocol.hpp"

using namespace vector::can;

namespace
{
std::string hex(const Frame & f)
{
  static const char * d = "0123456789abcdef";
  std::string s;
  for (int i = 0; i < f.len; ++i) {
    s += d[f.data[i] >> 4];
    s += d[f.data[i] & 0xF];
  }
  return s;
}

Frame from_hex(uint32_t id, const std::string & h)
{
  Frame f;
  f.id = id;
  f.len = static_cast<uint8_t>(h.size() / 2);
  for (size_t i = 0; i < f.len; ++i) {
    f.data[i] = static_cast<uint8_t>(std::stoi(h.substr(2 * i, 2), nullptr, 16));
  }
  return f;
}
}  // namespace

TEST(Protocol, Sync)
{
  const auto f = encode_sync(513, Mode::Walk, true);
  EXPECT_EQ(f.id, 0x000u);
  EXPECT_EQ(hex(f), "01021200");
}

TEST(Protocol, LegCmd)
{
  LegCmd c{-12.34, 14.46, -103.25, true, 200};
  const auto f = encode_leg_cmd(0, c);
  EXPECT_EQ(f.id, 0x011u);
  EXPECT_EQ(hex(f), "2efba605abd701c8");
}

TEST(Protocol, LegState)
{
  const auto s = decode_leg_state(from_hex(0x026, "9411ffffb1b98a0c"));
  ASSERT_TRUE(s);
  EXPECT_DOUBLE_EQ(s->coxa, 45.0);
  EXPECT_DOUBLE_EQ(s->femur, -0.01);
  EXPECT_DOUBLE_EQ(s->tibia, -179.99);
  EXPECT_EQ(s->current_ma, 3210);
  EXPECT_EQ(*leg_of(0x026), 5);

  // and back
  EXPECT_EQ(hex(encode_leg_state(5, *s)), "9411ffffb1b98a0c");
}

TEST(Protocol, LegStatus)
{
  const auto s = decode_leg_status(from_hex(0x032, "d204fe3d7a17fb81"));
  ASSERT_TRUE(s);
  EXPECT_EQ(s->tof_mm, 1234);
  EXPECT_EQ(s->vbat_mv, 15870);
  EXPECT_EQ(s->rail_mv, 6010);
  EXPECT_EQ(s->temperature, -5);
  EXPECT_EQ(s->faults, 0x81);
}

TEST(Protocol, RejectsOtherFrames)
{
  EXPECT_FALSE(decode_leg_state(from_hex(0x016, "0000000000000000")));  // a command
  EXPECT_FALSE(decode_leg_state(from_hex(0x027, "0000000000000000")));  // power node, not a leg
  EXPECT_FALSE(decode_leg_state(from_hex(0x021, "0000")));              // short
}
