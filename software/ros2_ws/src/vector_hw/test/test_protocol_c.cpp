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

TEST(ProtocolC, Power)
{
  for (int i = 0; i < 1000; ++i) {
    can_frame_t c;
    power_state_msg m{static_cast<uint16_t>(u(0xFFFF)), static_cast<int32_t>(u(600000)) - 300000,
      static_cast<uint8_t>(u(200)), static_cast<int8_t>(static_cast<int>(u(255)) - 128), static_cast<uint8_t>(u(5)),
      static_cast<uint8_t>(u(15)), static_cast<uint8_t>(u(255))};
    can_pack_power_state(&c, &m);
    const auto s = vector::can::decode_power_state(from_c(c));
    ASSERT_TRUE(s);
    EXPECT_EQ(s->pack_mv, m.pack_mv);
    EXPECT_EQ(s->current_ma, m.current_ma / 10 * 10);
    EXPECT_DOUBLE_EQ(s->soc, m.soc_half / 2.0);
    EXPECT_EQ(s->temperature, m.temp_c);
    EXPECT_EQ(s->state, m.state);
    EXPECT_EQ(s->flags, m.flags);
    EXPECT_EQ(s->faults, m.faults);

    power_cells_msg pc{{static_cast<uint16_t>(u(5000)), static_cast<uint16_t>(u(5000)),
      static_cast<uint16_t>(u(5000)), static_cast<uint16_t>(u(5000))}};
    can_pack_power_cells(&c, &pc);
    const auto cells = vector::can::decode_power_cells(from_c(c));
    ASSERT_TRUE(cells);
    for (int k = 0; k < 4; ++k) {
      EXPECT_EQ(cells->mv[k], pc.cell_mv[k]);
    }

    power_detail_msg d{static_cast<uint8_t>(u(255)), static_cast<uint8_t>(u(255)), static_cast<uint8_t>(u(255)),
      static_cast<uint8_t>(u(15)), static_cast<uint8_t>(u(3)), static_cast<uint8_t>(u(3)), static_cast<uint8_t>(u(7)),
      static_cast<uint8_t>(u(255)), static_cast<uint16_t>(u(25500)), static_cast<int8_t>(static_cast<int>(u(255)) - 128)};
    can_pack_power_detail(&c, &d);
    const auto det = vector::can::decode_power_detail(from_c(c));
    ASSERT_TRUE(det);
    EXPECT_EQ(det->safety_a, d.safety_a);
    EXPECT_EQ(det->safety_c, d.safety_c);
    EXPECT_EQ(det->fets, d.fets);
    EXPECT_EQ(det->sides, d.sides);
    EXPECT_EQ(det->inputs, d.inputs);
    EXPECT_EQ(det->charge_status, d.chg_stat);
    EXPECT_EQ(det->charger_fault, d.charger_fault);
    EXPECT_EQ(det->input_mv, d.input_mv / 100 * 100);
    EXPECT_EQ(det->fet_temperature, d.fet_temp_c);
  }
}

TEST(ProtocolC, Config)
{
  for (int i = 0; i < 1000; ++i) {
    vector::can::Config cfg{static_cast<uint8_t>(u(255)), static_cast<uint8_t>(u(255)),
      static_cast<int32_t>(u(0xFFFFFFFF)), static_cast<uint8_t>(u(255)), static_cast<uint8_t>(u(5))};
    const auto f = vector::can::encode_config(POWER_NODE, cfg);
    can_frame_t c;
    leg_cfg_msg m{cfg.joint, cfg.key, cfg.value, cfg.seq, cfg.op};
    can_pack_leg_cfg(&c, CAN_LEG_CONFIG, POWER_NODE, &m);
    expect_same(c, f);
    can_pack_leg_cfg(&c, CAN_LEG_REPLY, POWER_NODE, &m);
    const auto r = vector::can::decode_reply(from_c(c));
    ASSERT_TRUE(r);
    EXPECT_EQ(r->key, cfg.key);
    EXPECT_EQ(r->value, cfg.value);
    EXPECT_EQ(r->seq, cfg.seq);
    EXPECT_EQ(r->op, cfg.op);
  }
}

TEST(ProtocolC, PowerEnums)
{
  EXPECT_EQ(vector::can::kPowerState, static_cast<uint32_t>(CAN_POWER_STATE));
  EXPECT_EQ(vector::can::kPowerCells, static_cast<uint32_t>(CAN_POWER_CELLS));
  EXPECT_EQ(vector::can::kPowerDetail, static_cast<uint32_t>(CAN_POWER_DETAIL));
  EXPECT_EQ(vector::can::kConfig, static_cast<uint32_t>(CAN_LEG_CONFIG));
  EXPECT_EQ(vector::can::kReply, static_cast<uint32_t>(CAN_LEG_REPLY));
  EXPECT_EQ(vector::can::kPowerNode, static_cast<uint32_t>(POWER_NODE));
  EXPECT_EQ(vector::can::kPowerHalting, POWER_HALTING);
  EXPECT_EQ(vector::can::kPowerFault, POWER_FAULT);
  EXPECT_EQ(vector::can::kPwrEstop, PWR_ESTOP);
  EXPECT_EQ(vector::can::kPwrLow, PWR_LOW);
  EXPECT_EQ(vector::can::kPwrFaultHot, PWR_FAULT_HOT);
  EXPECT_EQ(vector::can::kPwrFault5v, PWR_FAULT_5V);
  EXPECT_EQ(vector::can::kKeySides, PKEY_SIDES);
  EXPECT_EQ(vector::can::kKeyShutdown, PKEY_SHUTDOWN);
  EXPECT_EQ(vector::can::kOpDefaults, OP_DEFAULTS);
  EXPECT_EQ(vector::can::kStBusy, ST_BUSY);
}
