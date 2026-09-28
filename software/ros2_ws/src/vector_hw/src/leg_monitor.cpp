// Listens on the leg bus next to ros2_control and publishes what the legs report besides
// their angles:
//   /diagnostics         one status per leg: state, faults, current, supplies, temperature
//   legs/<leg>/tof       sensor_msgs/Range from the VL53L1X, frame <leg>_tof
// Fault changes are logged as they happen. It only reads, so it runs alongside
// VectorSystem or leg_config.py.
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "vector_hw/protocol.hpp"
#include "vector_hw/socketcan.hpp"

using namespace std::chrono_literals;
namespace can = vector::can;
using diagnostic_msgs::msg::DiagnosticStatus;

namespace
{
constexpr int kLegs = 6;
constexpr std::array<const char *, kLegs> kLegNames = {"L1", "L2", "L3", "R1", "R2", "R3"};
// faults that mean the leg can't walk, as opposed to notes like "uncalibrated"
constexpr uint8_t kSerious = can::kFaultWake | can::kFaultBuck | can::kFaultOverload | can::kFaultConfig;
}  // namespace

class LegMonitor : public rclcpp::Node
{
public:
  LegMonitor()
  : Node("leg_monitor")
  {
    const auto ifname = declare_parameter("can_interface", std::string("can0"));
    fov_ = declare_parameter("tof_fov", 0.47);          // rad, 27 deg
    max_range_ = declare_parameter("tof_max_range", 1.3);  // m, short mode
    stale_ = std::chrono::milliseconds(declare_parameter("stale_ms", 1000));
    bus_.open(ifname);

    for (int l = 0; l < kLegs; ++l) {
      tof_pub_[l] = create_publisher<sensor_msgs::msg::Range>(
        std::string("legs/") + kLegNames[l] + "/tof", rclcpp::SensorDataQoS());
    }
    diag_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
    poll_timer_ = create_wall_timer(5ms, [this] {poll();});
    report_timer_ = create_wall_timer(500ms, [this] {report();});
    RCLCPP_INFO(get_logger(), "watching %s", ifname.c_str());
  }

private:
  using Clock = std::chrono::steady_clock;

  struct Leg
  {
    bool seen = false;
    can::LegStatus status;
    uint16_t current_ma = 0;
    Clock::time_point last{};
  };

  void poll()
  {
    for (int n = 0; n < 256; ++n) {
      const auto f = bus_.receive();
      if (!f) {
        return;
      }
      const auto leg = can::leg_of(f->id);
      if (!leg) {
        continue;
      }
      if (const auto s = can::decode_leg_state(*f)) {
        legs_[*leg].current_ma = s->current_ma;
      } else if (const auto st = can::decode_leg_status(*f)) {
        on_status(*leg, *st);
      }
    }
  }

  void on_status(int l, const can::LegStatus & st)
  {
    Leg & leg = legs_[l];
    const uint8_t was = leg.seen ? leg.status.faults : 0;
    if (const uint8_t added = st.faults & ~was) {
      RCLCPP_WARN(get_logger(), "%s: %s (state %s)", kLegNames[l], can::fault_names(added).c_str(),
        can::state_name(st.state));
    }
    if (const uint8_t cleared = was & ~st.faults) {
      RCLCPP_INFO(get_logger(), "%s: cleared %s", kLegNames[l], can::fault_names(cleared).c_str());
    }
    leg.status = st;
    leg.seen = true;
    leg.last = Clock::now();

    const double range = can::tof_range_m(st.tof_mm);
    if (std::isnan(range)) {
      return;  // no sensor or no fresh reading
    }
    sensor_msgs::msg::Range msg;
    msg.header.stamp = now();
    msg.header.frame_id = std::string(kLegNames[l]) + "_tof";
    msg.radiation_type = sensor_msgs::msg::Range::INFRARED;
    msg.field_of_view = static_cast<float>(fov_);
    msg.min_range = 0.04f;
    msg.max_range = static_cast<float>(max_range_);
    msg.range = static_cast<float>(range);
    tof_pub_[l]->publish(msg);
  }

  static diagnostic_msgs::msg::KeyValue kv(const std::string & key, const std::string & value)
  {
    diagnostic_msgs::msg::KeyValue v;
    v.key = key;
    v.value = value;
    return v;
  }

  static std::string fixed(double v, int decimals)
  {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.*f", decimals, v);
    return buf;
  }

  void report()
  {
    diagnostic_msgs::msg::DiagnosticArray arr;
    arr.header.stamp = now();
    const auto t = Clock::now();
    for (int l = 0; l < kLegs; ++l) {
      const Leg & leg = legs_[l];
      DiagnosticStatus s;
      s.name = std::string("legs: ") + kLegNames[l];
      s.hardware_id = "leg node " + std::to_string(l + 1);
      if (!leg.seen || t - leg.last > stale_) {
        s.level = DiagnosticStatus::STALE;
        s.message = leg.seen ? "silent" : "not seen";
        arr.status.push_back(s);
        continue;
      }
      const auto & st = leg.status;
      const std::string faults = can::fault_names(st.faults);
      s.level = st.state == can::kFault || (st.faults & kSerious) ? DiagnosticStatus::ERROR :
        st.faults ? DiagnosticStatus::WARN : DiagnosticStatus::OK;
      s.message = std::string(can::state_name(st.state)) + (faults.empty() ? "" : ": " + faults);
      s.values = {
        kv("state", can::state_name(st.state)),
        kv("faults", faults.empty() ? "none" : faults),
        kv("current (A)", fixed(leg.current_ma / 1000.0, 2)),
        kv("battery (V)", fixed(st.vbat_mv / 1000.0, 2)),
        kv("servo rail (V)", fixed(st.rail_mv / 1000.0, 2)),
        kv("temperature (C)", std::to_string(st.temperature)),
        kv("tof (mm)", st.tof_mm == 0 ? "none" : st.tof_mm == 0xFFFF ? "nothing in range" : std::to_string(st.tof_mm)),
      };
      arr.status.push_back(s);
    }
    diag_pub_->publish(arr);
  }

  vector::can::SocketCan bus_;
  double fov_ = 0.47, max_range_ = 1.3;
  std::chrono::milliseconds stale_{1000};
  std::array<Leg, kLegs> legs_{};
  std::array<rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr, kLegs> tof_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::TimerBase::SharedPtr poll_timer_, report_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LegMonitor>());
  rclcpp::shutdown();
  return 0;
}
