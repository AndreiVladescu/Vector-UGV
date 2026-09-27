#include "vector_hw/protocol.hpp"

#include <algorithm>
#include <cmath>

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

}  // namespace vector::can
