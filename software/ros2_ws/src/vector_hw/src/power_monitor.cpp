// The power board on the CAN bus:
//   battery              sensor_msgs/BatteryState at 10 Hz (current negative while discharging)
//   /diagnostics         power: battery / board / charger
//   power/estop          std_msgs/Bool, latched: the e-stop line is low (button, wire or ROS)
//   power/legs           std_srvs/SetBool: side power on / off
//   power/shutdown       std_srvs/Trigger: the power board asks the CM5 to halt, then cuts power
// When the board starts a shutdown, or the charge drops under sentinel_soc while
// discharging, it asks gait_node to sit down first.
#include <chrono>
#include <cmath>
#include <cstdio>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "vector_hw/protocol.hpp"
#include "vector_hw/socketcan.hpp"

using namespace std::chrono_literals;
namespace can = vector::can;
using diagnostic_msgs::msg::DiagnosticStatus;
using sensor_msgs::msg::BatteryState;
using std_srvs::srv::SetBool;
using std_srvs::srv::Trigger;

class PowerMonitor : public rclcpp::Node
{
public:
  PowerMonitor()
  : Node("power_monitor")
  {
    const auto ifname = declare_parameter("can_interface", std::string("can0"));
    stale_ = std::chrono::milliseconds(declare_parameter("stale_ms", 1000));
    sentinel_soc_ = declare_parameter("sentinel_soc", 10.0);
    bus_.open(ifname);

    battery_pub_ = create_publisher<BatteryState>("battery", 10);
    diag_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
    estop_pub_ = create_publisher<std_msgs::msg::Bool>("power/estop", rclcpp::QoS(1).transient_local());
    sentinel_ = create_client<Trigger>("gait_node/sentinel");

    legs_srv_ = create_service<SetBool>("power/legs",
      [this](std::shared_ptr<rclcpp::Service<SetBool>> srv, std::shared_ptr<rmw_request_id_t> id,
      std::shared_ptr<SetBool::Request> req) {
        request(can::kKeySides, req->data ? 3 : 0, [srv, id](bool ok, const std::string & why) {
          SetBool::Response res;
          res.success = ok;
          res.message = why;
          srv->send_response(*id, res);
        });
      });
    shutdown_srv_ = create_service<Trigger>("power/shutdown",
      [this](std::shared_ptr<rclcpp::Service<Trigger>> srv, std::shared_ptr<rmw_request_id_t> id,
      std::shared_ptr<Trigger::Request>) {
        request(can::kKeyShutdown, 1, [srv, id](bool ok, const std::string & why) {
          Trigger::Response res;
          res.success = ok;
          res.message = ok ? "shutting down" : why;
          srv->send_response(*id, res);
        });
      });

    poll_timer_ = create_wall_timer(5ms, [this] {poll(); retry();});
    report_timer_ = create_wall_timer(500ms, [this] {report();});
    RCLCPP_INFO(get_logger(), "watching the power board on %s", ifname.c_str());
  }

private:
  using Clock = std::chrono::steady_clock;
  using Done = std::function<void(bool, const std::string &)>;

  struct Pending
  {
    can::Config cfg;
    Done done;
    Clock::time_point sent;
    int tries = 0;
  };

  void request(uint8_t key, int32_t value, Done done, uint8_t op = can::kOpWrite)
  {
    Pending p;
    p.cfg = {0xFF, key, value, ++seq_, op};
    p.done = std::move(done);
    send(p);
    pending_[p.cfg.seq] = std::move(p);
  }

  void send(Pending & p)
  {
    p.sent = Clock::now();
    p.tries++;
    bus_.send(can::encode_config(can::kPowerNode, p.cfg));
  }

  void retry()
  {
    const auto t = Clock::now();
    for (auto it = pending_.begin(); it != pending_.end(); ) {
      if (t - it->second.sent < 300ms) {
        ++it;
      } else if (it->second.tries < 3) {
        send(it->second);
        ++it;
      } else {
        it->second.done(false, "no answer from the power board");
        it = pending_.erase(it);
      }
    }
  }

  void on_reply(const can::Config & r)
  {
    const auto it = pending_.find(r.seq);
    if (it == pending_.end() || it->second.cfg.key != r.key) {
      return;
    }
    static const char * names[] = {"ok", "bad key", "bad value", "busy"};
    it->second.done(r.op == can::kStOk, r.op < 4 ? names[r.op] : "error");
    pending_.erase(it);
    if (r.key == can::kKeyCapacity && r.op == can::kStOk) {
      capacity_mah_ = r.value;
    }
  }

  void poll()
  {
    for (int n = 0; n < 256; ++n) {
      const auto f = bus_.receive();
      if (!f) {
        return;
      }
      if (const auto s = can::decode_power_state(*f)) {
        on_state(*s);
      } else if (const auto c = can::decode_power_cells(*f)) {
        cells_ = *c;
      } else if (const auto d = can::decode_power_detail(*f)) {
        detail_ = *d;
      } else if (f->id == (can::kReply | can::kPowerNode)) {
        if (const auto r = can::decode_reply(*f)) {
          on_reply(*r);
        }
      }
    }
  }

  void sit_down(const char * why)
  {
    RCLCPP_WARN(get_logger(), "%s: sitting down", why);
    if (sentinel_->service_is_ready()) {
      sentinel_->async_send_request(std::make_shared<Trigger::Request>());
    }
  }

  void on_state(const can::PowerState & s)
  {
    if (seen_ && s.state != state_.state) {
      RCLCPP_INFO(get_logger(), "power board: %s -> %s", can::power_state_name(state_.state),
        can::power_state_name(s.state));
      if (s.state == can::kPowerHalting) {
        sit_down("the power board is shutting down");
      }
    }
    const uint8_t was = seen_ ? state_.faults : 0;
    if (const uint8_t added = s.faults & ~was) {
      RCLCPP_WARN(get_logger(), "power board: %s", can::power_fault_names(added).c_str());
    }
    if (const uint8_t cleared = was & ~s.faults) {
      RCLCPP_INFO(get_logger(), "power board: cleared %s", can::power_fault_names(cleared).c_str());
    }
    const bool estop = s.flags & can::kPwrEstop;
    if (!seen_ || estop != bool(state_.flags & can::kPwrEstop)) {
      std_msgs::msg::Bool b;
      b.data = estop;
      estop_pub_->publish(b);
    }
    if (s.current_ma < 0 && s.soc < sentinel_soc_ && !low_sat_ && s.state == can::kPowerOn) {
      low_sat_ = true;
      sit_down("battery low");
    } else if (s.soc > sentinel_soc_ + 5) {
      low_sat_ = false;
    }
    state_ = s;
    seen_ = true;
    last_ = Clock::now();
    if (capacity_mah_ < 0 && pending_.empty() && ++capacity_ask_ % 50 == 1) {
      request(can::kKeyCapacity, 0, [](bool, const std::string &) {}, can::kOpRead);
    }
    publish_battery();
  }

  void publish_battery()
  {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    BatteryState b;
    b.header.stamp = now();
    b.header.frame_id = "base_link";
    b.voltage = state_.pack_mv / 1000.0f;
    b.current = state_.current_ma / 1000.0f;
    b.temperature = state_.temperature;
    b.charge = static_cast<float>(nan);
    b.capacity = b.design_capacity = capacity_mah_ > 0 ? capacity_mah_ / 1000.0f : static_cast<float>(nan);
    b.percentage = static_cast<float>(state_.soc / 100.0);
    if (state_.flags & can::kPwrCharging) {
      b.power_supply_status = BatteryState::POWER_SUPPLY_STATUS_CHARGING;
    } else if (state_.flags & can::kPwrCharger) {
      b.power_supply_status = state_.soc >= 99 ? BatteryState::POWER_SUPPLY_STATUS_FULL :
        BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING;
    } else {
      b.power_supply_status = BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
    }
    if (state_.faults & can::kPwrFaultHot) {
      b.power_supply_health = BatteryState::POWER_SUPPLY_HEALTH_OVERHEAT;
    } else if (detail_.safety_a & 0x08) {  // COV
      b.power_supply_health = BatteryState::POWER_SUPPLY_HEALTH_OVERVOLTAGE;
    } else if (state_.faults & (can::kPwrFaultBms | can::kPwrFaultBmsComm)) {
      b.power_supply_health = BatteryState::POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
    } else {
      b.power_supply_health = BatteryState::POWER_SUPPLY_HEALTH_GOOD;
    }
    b.power_supply_technology = BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;
    b.present = true;
    for (const auto mv : cells_.mv) {
      b.cell_voltage.push_back(mv ? mv / 1000.0f : static_cast<float>(nan));
      b.cell_temperature.push_back(static_cast<float>(nan));
    }
    b.location = "pack";
    battery_pub_->publish(b);
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
    DiagnosticStatus bat, board, chg;
    bat.name = "power: battery";
    board.name = "power: board";
    chg.name = "power: charger";
    bat.hardware_id = board.hardware_id = chg.hardware_id = "power node 7";
    if (!seen_ || Clock::now() - last_ > stale_) {
      for (auto * s : {&bat, &board, &chg}) {
        s->level = DiagnosticStatus::STALE;
        s->message = seen_ ? "silent" : "not seen";
        arr.status.push_back(*s);
      }
      diag_pub_->publish(arr);
      return;
    }
    const auto & s = state_;
    const auto & d = detail_;

    bat.level = s.faults & (can::kPwrFaultBms | can::kPwrFaultLowCell | can::kPwrFaultHot) ? DiagnosticStatus::ERROR :
      (s.flags & can::kPwrLow) || (s.faults & can::kPwrFaultImbalance) ? DiagnosticStatus::WARN : DiagnosticStatus::OK;
    bat.message = fixed(s.soc, 0) + " %, " + fixed(s.pack_mv / 1000.0, 2) + " V, " + fixed(s.current_ma / 1000.0, 2) + " A";
    std::string cells;
    for (const auto mv : cells_.mv) {
      cells += (cells.empty() ? "" : " ") + fixed(mv / 1000.0, 3);
    }
    char safety[16];
    std::snprintf(safety, sizeof(safety), "%02x %02x %02x", d.safety_a, d.safety_b, d.safety_c);
    bat.values = {
      kv("state of charge (%)", fixed(s.soc, 1)),
      kv("voltage (V)", fixed(s.pack_mv / 1000.0, 2)),
      kv("current (A)", fixed(s.current_ma / 1000.0, 2)),
      kv("cells (V)", cells),
      kv("warmest cell (C)", std::to_string(s.temperature)),
      kv("FETs (C)", std::to_string(d.fet_temperature)),
      kv("BMS safety status A B C", safety),
    };

    const std::string faults = can::power_fault_names(s.faults);
    board.level = s.state == can::kPowerFault || (s.faults & (can::kPwrFaultBmsComm | can::kPwrFault5v)) ?
      DiagnosticStatus::ERROR :
      (s.faults & can::kPwrFaultConfig) || (s.flags & can::kPwrEstop) ? DiagnosticStatus::WARN : DiagnosticStatus::OK;
    board.message = std::string(can::power_state_name(s.state)) + (s.flags & can::kPwrEstop ? ", e-stop" : "") +
      (faults.empty() ? "" : ": " + faults);
    board.values = {
      kv("state", can::power_state_name(s.state)),
      kv("faults", faults.empty() ? "none" : faults),
      kv("e-stop", s.flags & can::kPwrEstop ? "pressed" : "released"),
      kv("left side", d.sides & 1 ? "on" : "off"),
      kv("right side", d.sides & 2 ? "on" : "off"),
      kv("pack FETs", std::string(d.fets & 1 ? "CHG " : "") + (d.fets & 4 ? "DSG" : "")),
    };

    const bool plugged = s.flags & can::kPwrCharger;
    chg.level = s.faults & can::kPwrFaultCharger ? DiagnosticStatus::WARN : DiagnosticStatus::OK;
    chg.message = !plugged ? "not plugged in" :
      std::string(d.inputs & 1 ? "DC " : "USB-C ") + fixed(d.input_mv / 1000.0, 1) + " V, " +
      can::charge_status_name(d.charge_status);
    chg.values = {
      kv("input", !plugged ? "none" : d.inputs == 3 ? "DC and USB-C" : d.inputs & 1 ? "DC" : "USB-C"),
      kv("input (V)", fixed(d.input_mv / 1000.0, 1)),
      kv("status", can::charge_status_name(d.charge_status)),
      kv("fault register", std::to_string(d.charger_fault)),
    };
    arr.status = {bat, board, chg};
    diag_pub_->publish(arr);
  }

  can::SocketCan bus_;
  std::chrono::milliseconds stale_{1000};
  double sentinel_soc_ = 10.0;
  bool seen_ = false, low_sat_ = false;
  can::PowerState state_;
  can::PowerCells cells_;
  can::PowerDetail detail_;
  int32_t capacity_mah_ = -1;
  int capacity_ask_ = 0;
  Clock::time_point last_{};
  uint8_t seq_ = 0;
  std::map<uint8_t, Pending> pending_;

  rclcpp::Publisher<BatteryState>::SharedPtr battery_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_pub_;
  rclcpp::Client<Trigger>::SharedPtr sentinel_;
  rclcpp::Service<SetBool>::SharedPtr legs_srv_;
  rclcpp::Service<Trigger>::SharedPtr shutdown_srv_;
  rclcpp::TimerBase::SharedPtr poll_timer_, report_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PowerMonitor>());
  rclcpp::shutdown();
  return 0;
}
