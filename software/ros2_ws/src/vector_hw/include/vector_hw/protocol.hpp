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
constexpr uint32_t kConfig = 0x050;
constexpr uint32_t kReply = 0x060;
constexpr uint32_t kPowerCells = 0x090;
constexpr uint32_t kPowerDetail = 0x0A0;
constexpr uint32_t kPowerNode = 7;

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

// Power board (node 7), see protocol/README.md.
enum PowerStateId : uint8_t { kPowerBoot, kPowerCharge, kPowerOn, kPowerHalting, kPowerOff, kPowerFault };
constexpr uint8_t kPwrCharger = 0x01, kPwrCharging = 0x02, kPwrEstop = 0x04, kPwrLow = 0x08;
// Fault bits, low to high: bms, bms_comm, charger, 5v, low_cell, imbalance, config, hot.
constexpr uint8_t kPwrFaultBms = 0x01, kPwrFaultBmsComm = 0x02, kPwrFaultCharger = 0x04, kPwrFault5v = 0x08,
  kPwrFaultLowCell = 0x10, kPwrFaultImbalance = 0x20, kPwrFaultConfig = 0x40, kPwrFaultHot = 0x80;
// Config keys on the power node (joint 255)
enum PowerKey : uint8_t {
  kKeyCapacity = 32, kKeyChargeMa, kKeyChargeMv, kKeyInputMa, kKeyLowMv, kKeySides, kKeySoc, kKeyBmsMem, kKeyShutdown
};
enum ConfigOp : uint8_t { kOpRead, kOpWrite, kOpSave, kOpCalibrate, kOpDefaults, kOpSelftest };
enum ConfigStatus : uint8_t { kStOk, kStBadKey, kStBadValue, kStBusy };

struct PowerState
{
  uint16_t pack_mv = 0;
  int32_t current_ma = 0;  // positive while charging
  double soc = 0;          // %
  int8_t temperature = 0;  // warmest cell
  uint8_t state = 0, flags = 0, faults = 0;
};

struct PowerCells
{
  std::array<uint16_t, 4> mv{};
};

struct PowerDetail
{
  uint8_t safety_a = 0, safety_b = 0, safety_c = 0;
  uint8_t fets = 0;    // CHG, PCHG, DSG, PDSG
  uint8_t sides = 0;   // bit 0 left, bit 1 right
  uint8_t inputs = 0;  // bit 0 DC, bit 1 USB-C
  uint8_t charge_status = 0;
  uint8_t charger_fault = 0;
  uint16_t input_mv = 0;
  int8_t fet_temperature = 0;
};

struct Config
{
  uint8_t joint = 0xFF, key = 0;
  int32_t value = 0;
  uint8_t seq = 0, op = 0;  // op, or the status in a reply
};

const char * state_name(uint8_t state);
std::string fault_names(uint8_t faults);  // "overload, buck"; empty for none
// LEG_STATUS ToF field in metres: +inf for nothing in range (REP 117), NaN for no reading.
double tof_range_m(uint16_t mm);

Frame encode_sync(uint16_t counter, Mode mode, bool estop);
Frame encode_leg_cmd(int leg, const LegCmd & cmd);
Frame encode_leg_state(int leg, const LegState & state);   // for the fake legs / tests
std::optional<LegState> decode_leg_state(const Frame & f);
std::optional<LegStatus> decode_leg_status(const Frame & f);

const char * power_state_name(uint8_t state);
std::string power_fault_names(uint8_t faults);
const char * charge_status_name(uint8_t status);
std::optional<PowerState> decode_power_state(const Frame & f);
std::optional<PowerCells> decode_power_cells(const Frame & f);
std::optional<PowerDetail> decode_power_detail(const Frame & f);
Frame encode_config(uint32_t node, const Config & c);
std::optional<Config> decode_reply(const Frame & f);  // LEG_REPLY from any node

}  // namespace vector::can
