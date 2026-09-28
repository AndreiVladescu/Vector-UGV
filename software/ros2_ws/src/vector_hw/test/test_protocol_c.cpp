// The firmware packs and unpacks frames in C (firmware/common/vector_can.c), this package
// in C++. Random values through both, in both directions, must give the same bytes.
#include <gtest/gtest.h>

#include <cstring>
#include <random>

#include "vector_hw/protocol.hpp"

extern "C" {
#include "vector_can.h"
}

namespace
{
std::mt19937 rng(42);

double deg() {return std::uniform_real_distribution<double>(-179.99, 179.99)(rng);}
uint32_t u(uint32_t hi) {return std::uniform_int_distribution<uint32_t>(0, hi)(rng);}

vector::can::Frame from_c(const can_frame_t & c)
{
  vector::can::Frame f;
  f.id = c.id;
  f.len = c.len;
  std::memcpy(f.data.data(), c.data, 8);
  return f;
}

void expect_same(const can_frame_t & c, const vector::can::Frame & f)
{
  EXPECT_EQ(c.id, f.id);
  EXPECT_EQ(c.len, f.len);
  EXPECT_EQ(0, std::memcmp(c.data, f.data.data(), c.len));
}
}  // namespace

TEST(ProtocolC, Sync)
{
  for (int i = 0; i < 1000; ++i) {
    const auto counter = static_cast<uint16_t>(u(0xFFFF));
    const bool estop = u(1);
    can_frame_t c;
    sync_msg m{counter, 2, estop};
    can_pack_sync(&c, &m);
    expect_same(c, vector::can::encode_sync(counter, vector::can::Mode::Walk, estop));
  }
}

TEST(ProtocolC, LegCmd)
{
  for (int i = 0; i < 1000; ++i) {
    const int leg = static_cast<int>(u(5));
    vector::can::LegCmd cmd{deg(), deg(), deg(), u(1) == 1, static_cast<uint8_t>(u(255))};
    const auto f = vector::can::encode_leg_cmd(leg, cmd);
    can_frame_t c;
    leg_cmd_msg m{{static_cast<float>(cmd.coxa), static_cast<float>(cmd.femur), static_cast<float>(cmd.tibia)},
      cmd.enable, cmd.counter};
    can_pack_leg_cmd(&c, static_cast<uint8_t>(leg + 1), &m);
    // float vs double can round a value sitting on .5 differently; allow one centidegree
    for (int k = 0; k < 6; k += 2) {
      const int a = static_cast<int16_t>(c.data[k] | c.data[k + 1] << 8);
      const int b = static_cast<int16_t>(f.data[k] | f.data[k + 1] << 8);
      EXPECT_LE(std::abs(a - b), 1);
    }
    EXPECT_EQ(c.id, f.id);
    EXPECT_EQ(c.data[6], f.data[6]);
    EXPECT_EQ(c.data[7], f.data[7]);

    leg_cmd_msg back;
    ASSERT_TRUE(can_unpack_leg_cmd(&c, &back));
    EXPECT_NEAR(back.deg[0], cmd.coxa, 0.006);
    EXPECT_EQ(back.enable, cmd.enable);
  }
}

TEST(ProtocolC, LegState)
{
  for (int i = 0; i < 1000; ++i) {
    const int leg = static_cast<int>(u(5));
    can_frame_t c;
    leg_state_msg m{{static_cast<float>(deg()), static_cast<float>(deg()), static_cast<float>(deg())},
      static_cast<uint16_t>(u(0xFFFF))};
    can_pack_leg_state(&c, static_cast<uint8_t>(leg + 1), &m);
    const auto s = vector::can::decode_leg_state(from_c(c));
    ASSERT_TRUE(s);
    EXPECT_EQ(*vector::can::leg_of(c.id), leg);
    EXPECT_NEAR(s->coxa, m.deg[0], 0.006);
    EXPECT_NEAR(s->femur, m.deg[1], 0.006);
    EXPECT_NEAR(s->tibia, m.deg[2], 0.006);
    EXPECT_EQ(s->current_ma, m.current_ma);
    expect_same(c, vector::can::encode_leg_state(leg, *s));
  }
}

TEST(ProtocolC, LegStatus)
{
  for (int i = 0; i < 1000; ++i) {
    const int leg = static_cast<int>(u(5));
    can_frame_t c;
    leg_status_msg m{static_cast<uint16_t>(u(0xFFFF)), static_cast<uint16_t>(u(40950)), static_cast<uint16_t>(u(40950)),
      static_cast<int8_t>(static_cast<int>(u(255)) - 128), static_cast<uint8_t>(u(255)), static_cast<uint8_t>(u(6))};
    can_pack_leg_status(&c, static_cast<uint8_t>(leg + 1), &m);
    const auto s = vector::can::decode_leg_status(from_c(c));
    ASSERT_TRUE(s);
    EXPECT_EQ(s->tof_mm, m.tof_mm);
    EXPECT_EQ(s->vbat_mv, m.vbat_mv / 10 * 10);
    EXPECT_EQ(s->rail_mv, m.rail_mv / 10 * 10);
    EXPECT_EQ(s->temperature, m.temp_c);
    EXPECT_EQ(s->faults, m.faults);
    EXPECT_EQ(s->state, m.state);
  }
}

TEST(ProtocolC, Enums)
{
  EXPECT_EQ(vector::can::kSync, static_cast<uint32_t>(CAN_SYNC));
  EXPECT_EQ(vector::can::kLegCmd, static_cast<uint32_t>(CAN_LEG_CMD));
  EXPECT_EQ(vector::can::kLegState, static_cast<uint32_t>(CAN_LEG_STATE));
  EXPECT_EQ(vector::can::kLegStatus, static_cast<uint32_t>(CAN_LEG_STATUS));
  EXPECT_EQ(vector::can::kActive, LEG_ACTIVE);
  EXPECT_EQ(vector::can::kFault, LEG_FAULT);
  EXPECT_EQ(vector::can::kTest, LEG_TEST);
  EXPECT_EQ(vector::can::kFaultEstop, FAULT_ESTOP);
  EXPECT_EQ(vector::can::kFaultOverload, FAULT_OVERLOAD);
  EXPECT_EQ(vector::can::kFaultConfig, FAULT_CONFIG);
}
