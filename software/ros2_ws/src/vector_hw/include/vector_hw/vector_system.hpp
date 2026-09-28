// ros2_control hardware for the six CAN leg nodes.
//
// write(): SYNC + one LEG_CMD per leg every cycle. read(): drains LEG_STATE frames into
// joint positions. Activation waits until every leg has reported, and starts the
// commands at the measured angles so nothing jumps. A leg that goes quiet stops the
// hardware; with SYNC gone every leg crouches and powers down on its own watchdog.
// GPIO "legs": enable (command 1/0, state 1 when all six are active, 0.5 when some are)
// and estop (command 1 sets the e-stop flag in SYNC and holds the legs off).
#pragma once

#include <array>
#include <chrono>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "vector_hw/socketcan.hpp"

namespace vector_hw
{

class VectorSystem : public hardware_interface::SystemInterface
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams & params) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  using Clock = std::chrono::steady_clock;
  static constexpr int kLegs = 6;
  static constexpr int kJoints = 18;

  void drain();
  void send_all(bool enable, bool estop);

  vector::can::SocketCan bus_;
  std::string ifname_ = "can0";
  std::chrono::milliseconds timeout_{100};

  // joint i = leg * 3 + {coxa, femur, tibia}; info_.joints[k] maps to slot_[k]
  std::array<int, kJoints> slot_{};
  std::array<double, kJoints> pos_{}, vel_{}, eff_{}, cmd_{};
  std::array<Clock::time_point, kLegs> last_rx_{};
  std::array<bool, kLegs> seen_{};
  std::array<uint8_t, kLegs> leg_state_{};
  double legs_enable_cmd_ = 1.0, legs_estop_cmd_ = 0.0, legs_active_ = 0.0;
  bool has_gpio_ = false, has_estop_ = false;
  uint16_t sync_counter_ = 0;
  uint8_t cmd_counter_ = 0;
  rclcpp::Logger log_ = rclcpp::get_logger("VectorSystem");
};

}  // namespace vector_hw
