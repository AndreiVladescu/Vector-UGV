#include "vector_hw/protocol.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>

namespace vector::can
{

namespace
{
void put16(Frame & f, int at, int32_t v)
{
  const auto u = static_cast<uint16_t>(v);
  f.data[at] = u & 0xFF;
  f.data[at + 1] = u >> 8;
}

uint16_t get16(const Frame & f, int at)
{
  return static_cast<uint16_t>(f.data[at] | (f.data[at + 1] << 8));
}

int16_t centideg(double deg)
{
  return static_cast<int16_t>(std::clamp(std::lround(deg * 100.0), -18000L, 18000L));
}

double from_centideg(uint16_t raw) {return static_cast<int16_t>(raw) / 100.0;}
}  // namespace

Frame encode_sync(uint16_t counter, Mode mode, bool estop)
{
  Frame f;
  f.id = kSync;
  f.len = 4;
  put16(f, 0, counter);
  f.data[2] = (static_cast<uint8_t>(mode) & 0x0F) | (estop ? 0x10 : 0x00);
  return f;
}

Frame encode_leg_cmd(int leg, const LegCmd & c)
{
  Frame f;
  f.id = kLegCmd | node_of(leg);
  f.len = 8;
  put16(f, 0, centideg(c.coxa));
  put16(f, 2, centideg(c.femur));
  put16(f, 4, centideg(c.tibia));
  f.data[6] = c.enable ? 1 : 0;
  f.data[7] = c.counter;
  return f;
}

Frame encode_leg_state(int leg, const LegState & s)
{
  Frame f;
  f.id = kLegState | node_of(leg);
  f.len = 8;
  put16(f, 0, centideg(s.coxa));
  put16(f, 2, centideg(s.femur));
  put16(f, 4, centideg(s.tibia));
  put16(f, 6, s.current_ma);
  return f;
}

std::optional<LegState> decode_leg_state(const Frame & f)
{
  if (function_of(f.id) != kLegState || !leg_of(f.id) || f.len < 8) {
    return std::nullopt;
  }
  return LegState{from_centideg(get16(f, 0)), from_centideg(get16(f, 2)), from_centideg(get16(f, 4)),
    get16(f, 6)};
}

const char * state_name(uint8_t state)
{
  static const char * names[] = {"off", "wake", "active", "crouch", "calibrate", "fault", "test"};
  return state < std::size(names) ? names[state] : "unknown";
}

namespace
{
std::string bit_names(uint8_t bits, const char * const (&names)[8])
{
  std::string out;
  for (int i = 0; i < 8; ++i) {
    if (bits & (1 << i)) {
      out += (out.empty() ? "" : ", ") + std::string(names[i]);
    }
  }
  return out;
}
}  // namespace

std::string fault_names(uint8_t faults)
{
  static const char * const names[] = {"watchdog", "estop", "wake", "uncalibrated", "buck", "overload", "cal", "config"};
  return bit_names(faults, names);
}

double tof_range_m(uint16_t mm)
{
  if (mm == 0) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return mm == 0xFFFF ? std::numeric_limits<double>::infinity() : mm / 1000.0;
}

std::optional<LegStatus> decode_leg_status(const Frame & f)
{
  if (function_of(f.id) != kLegStatus || !leg_of(f.id) || f.len < 8) {
    return std::nullopt;
  }
  LegStatus s;
  s.tof_mm = get16(f, 0);
  s.vbat_mv = static_cast<uint16_t>((f.data[2] | ((f.data[3] & 0x0f) << 8)) * 10);
  s.rail_mv = static_cast<uint16_t>(((f.data[3] >> 4) | (f.data[4] << 4)) * 10);
  s.temperature = static_cast<int8_t>(f.data[5]);
  s.faults = f.data[6];
  s.state = f.data[7] & 0x0f;
  return s;
}

const char * power_state_name(uint8_t state)
{
  static const char * names[] = {"boot", "charge", "on", "halting", "off", "fault"};
  return state < std::size(names) ? names[state] : "unknown";
}

std::string power_fault_names(uint8_t faults)
{
  static const char * const names[] = {"bms", "bms_comm", "charger", "5v", "low_cell", "imbalance", "config", "hot"};
  return bit_names(faults, names);
}

const char * charge_status_name(uint8_t status)
{
  static const char * names[] = {"not charging", "trickle", "pre-charge", "fast", "taper", "?", "top-off", "done"};
  return status < std::size(names) ? names[status] : "unknown";
}

std::optional<PowerState> decode_power_state(const Frame & f)
{
  if (f.id != (kPowerState | kPowerNode) || f.len < 8) {
    return std::nullopt;
  }
  PowerState s;
  s.pack_mv = get16(f, 0);
  s.current_ma = static_cast<int16_t>(get16(f, 2)) * 10;
  s.soc = f.data[4] / 2.0;
  s.temperature = static_cast<int8_t>(f.data[5]);
  s.state = f.data[6] & 0x0F;
  s.flags = f.data[6] >> 4;
  s.faults = f.data[7];
  return s;
}

std::optional<PowerCells> decode_power_cells(const Frame & f)
{
  if (f.id != (kPowerCells | kPowerNode) || f.len < 8) {
    return std::nullopt;
  }
  PowerCells c;
  for (int i = 0; i < 4; ++i) {
    c.mv[i] = get16(f, 2 * i);
  }
  return c;
}

std::optional<PowerDetail> decode_power_detail(const Frame & f)
{
  if (f.id != (kPowerDetail | kPowerNode) || f.len < 8) {
    return std::nullopt;
  }
  PowerDetail d;
  d.safety_a = f.data[0];
  d.safety_b = f.data[1];
  d.safety_c = f.data[2];
  d.fets = f.data[3] & 0x0F;
  d.usb = f.data[3] & 0x10;
  d.charge_status = f.data[4] & 7;
  d.charger_fault = f.data[5];
  d.input_mv = static_cast<uint16_t>(f.data[6] * 100);
  d.fet_temperature = static_cast<int8_t>(f.data[7]);
  return d;
}

Frame encode_config(uint32_t node, const Config & c)
{
  Frame f;
  f.id = kConfig | node;
  f.len = 8;
  f.data[0] = c.joint;
  f.data[1] = c.key;
  const auto v = static_cast<uint32_t>(c.value);
  for (int i = 0; i < 4; ++i) {
    f.data[2 + i] = (v >> (8 * i)) & 0xFF;
  }
  f.data[6] = c.seq;
  f.data[7] = c.op;
  return f;
}

std::optional<Config> decode_reply(const Frame & f)
{
  if (function_of(f.id) != kReply || f.len < 8) {
    return std::nullopt;
  }
  Config c;
  c.joint = f.data[0];
  c.key = f.data[1];
  c.value = static_cast<int32_t>(f.data[2] | (f.data[3] << 8) | (f.data[4] << 16) | (static_cast<uint32_t>(f.data[5]) << 24));
  c.seq = f.data[6];
  c.op = f.data[7];
  return c;
}

}  // namespace vector::can
