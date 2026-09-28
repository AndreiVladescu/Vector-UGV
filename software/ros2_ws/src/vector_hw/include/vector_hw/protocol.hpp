// Leg bus frames, matching protocol/vector.dbc. Plain C++, no ROS.
// ID = (function << 4) | node. Angles in 0.01 deg, little-endian.
#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>

namespace vector::can
{

constexpr uint32_t kSync = 0x000;
constexpr uint32_t kLegCmd = 0x010;
constexpr uint32_t kLegState = 0x020;
constexpr uint32_t kLegStatus = 0x030;
constexpr uint32_t kPowerState = 0x040;

// Leg index 0..5 (L1 L2 L3 R1 R2 R3) <-> node id 1..6.
constexpr uint32_t node_of(int leg) {return static_cast<uint32_t>(leg + 1);}
constexpr uint32_t function_of(uint32_t id) {return id & 0x7F0;}
inline std::optional<int> leg_of(uint32_t id)
{
  const uint32_t node = id & 0x00F;
  if (node < 1 || node > 6) {
    return std::nullopt;
  }
  return static_cast<int>(node - 1);
}

enum class Mode : uint8_t { Idle = 0, Stand = 1, Walk = 2, Sentinel = 3, Wake = 4, Fault = 15 };

struct Frame
{
  uint32_t id = 0;
  uint8_t len = 0;
  std::array<uint8_t, 8> data{};
};

struct LegCmd
{
  double coxa = 0, femur = 0, tibia = 0;  // deg
  bool enable = false;
  uint8_t counter = 0;
};

struct LegState
{
  double coxa = 0, femur = 0, tibia = 0;  // deg
  uint16_t current_ma = 0;
};

struct LegStatus
{
  uint16_t tof_mm = 0, vbat_mv = 0, rail_mv = 0;  // vbat and rail travel in 10 mV steps
  int8_t temperature = 0;
  uint8_t faults = 0;
  uint8_t state = 0;  // see state_name()
};

enum LegStateId : uint8_t { kOff, kWake, kActive, kCrouch, kCalibrate, kFault, kTest };
// Fault bits, low to high: watchdog, estop, wake, uncalibrated, buck, overload, cal, config.
constexpr uint8_t kFaultWatchdog = 0x01, kFaultEstop = 0x02, kFaultWake = 0x04, kFaultUncalibrated = 0x08,
  kFaultBuck = 0x10, kFaultOverload = 0x20, kFaultCal = 0x40, kFaultConfig = 0x80;

const char * state_name(uint8_t state);
std::string fault_names(uint8_t faults);  // "overload, buck"; empty for none
// LEG_STATUS ToF field in metres: +inf for nothing in range (REP 117), NaN for no reading.
double tof_range_m(uint16_t mm);

Frame encode_sync(uint16_t counter, Mode mode, bool estop);
Frame encode_leg_cmd(int leg, const LegCmd & cmd);
Frame encode_leg_state(int leg, const LegState & state);   // for the fake legs / tests
std::optional<LegState> decode_leg_state(const Frame & f);
std::optional<LegStatus> decode_leg_status(const Frame & f);

}  // namespace vector::can
