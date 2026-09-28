#include "vector_hw/vector_system.hpp"

#include <algorithm>
#include <cmath>
#include <thread>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace vector_hw
{

namespace
{
constexpr std::array<const char *, 6> kLegNames = {"L1", "L2", "L3", "R1", "R2", "R3"};
constexpr std::array<const char *, 3> kJointNames = {"coxa", "femur", "tibia"};
constexpr double kDeg = M_PI / 180.0;
}  // namespace

VectorSystem::CallbackReturn VectorSystem::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  if (info_.joints.size() != kJoints) {
    RCLCPP_ERROR(log_, "expected %d joints, got %zu", kJoints, info_.joints.size());
    return CallbackReturn::ERROR;
  }
  if (auto it = info_.hardware_parameters.find("can_interface"); it != info_.hardware_parameters.end()) {
    ifname_ = it->second;
  }
  if (auto it = info_.hardware_parameters.find("timeout_ms"); it != info_.hardware_parameters.end()) {
    timeout_ = std::chrono::milliseconds(std::stoi(it->second));
  }

  for (size_t k = 0; k < info_.joints.size(); ++k) {
    const auto & name = info_.joints[k].name;
    slot_[k] = -1;
    for (int l = 0; l < kLegs; ++l) {
      for (int j = 0; j < 3; ++j) {
        if (name == std::string(kLegNames[l]) + "_" + kJointNames[j]) {
          slot_[k] = l * 3 + j;
        }
      }
    }
    if (slot_[k] < 0) {
      RCLCPP_ERROR(log_, "unknown joint '%s', expected L1_coxa ... R3_tibia", name.c_str());
      return CallbackReturn::ERROR;
    }
    // Until the legs report, use the URDF initial values.
    for (const auto & si : info_.joints[k].state_interfaces) {
      if (si.name == hardware_interface::HW_IF_POSITION && !si.initial_value.empty()) {
        pos_[slot_[k]] = std::stod(si.initial_value);
      }
    }
  }
  cmd_ = pos_;
  for (const auto & g : info_.gpios) {
    if (g.name != "legs") {
      continue;
    }
    has_gpio_ = true;
    for (const auto & ci : g.command_interfaces) {
      has_estop_ = has_estop_ || ci.name == "estop";
    }
  }
  return CallbackReturn::SUCCESS;
}

VectorSystem::CallbackReturn VectorSystem::on_configure(const rclcpp_lifecycle::State &)
{
  try {
    bus_.open(ifname_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(log_, "can't open %s: %s", ifname_.c_str(), e.what());
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(log_, "on %s, leg timeout %ld ms", ifname_.c_str(), timeout_.count());
  return CallbackReturn::SUCCESS;
}

VectorSystem::CallbackReturn VectorSystem::on_activate(const rclcpp_lifecycle::State &)
{
  // Wait for every leg to report its measured angles (up to 1 s).
  seen_.fill(false);
  const auto deadline = Clock::now() + std::chrono::seconds(1);
  while (Clock::now() < deadline) {
    drain();
    if (std::all_of(seen_.begin(), seen_.end(), [](bool s) {return s;})) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  for (int l = 0; l < kLegs; ++l) {
    if (!seen_[l]) {
      RCLCPP_ERROR(log_, "leg %s is not responding on %s", kLegNames[l], ifname_.c_str());
      return CallbackReturn::ERROR;
    }
  }
  cmd_ = pos_;  // hold where the legs are
  vel_.fill(0);
  RCLCPP_INFO(log_, "all six legs up");
  return CallbackReturn::SUCCESS;
}

VectorSystem::CallbackReturn VectorSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  // No "enable off" here: that would drop the robot where it stands. Without SYNC the
  // legs crouch slowly and then switch off by themselves.
  RCLCPP_INFO(log_, "stopped, the legs crouch and power down on their own");
  return CallbackReturn::SUCCESS;
}

VectorSystem::CallbackReturn VectorSystem::on_cleanup(const rclcpp_lifecycle::State &)
{
  bus_.close();
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> VectorSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> out;
  for (size_t k = 0; k < info_.joints.size(); ++k) {
    const auto & name = info_.joints[k].name;
    out.emplace_back(name, hardware_interface::HW_IF_POSITION, &pos_[slot_[k]]);
    out.emplace_back(name, hardware_interface::HW_IF_VELOCITY, &vel_[slot_[k]]);
    out.emplace_back(name, hardware_interface::HW_IF_EFFORT, &eff_[slot_[k]]);
  }
  if (has_gpio_) {
    out.emplace_back("legs", "enable", &legs_active_);
  }
  return out;
}

std::vector<hardware_interface::CommandInterface> VectorSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> out;
  for (size_t k = 0; k < info_.joints.size(); ++k) {
    out.emplace_back(info_.joints[k].name, hardware_interface::HW_IF_POSITION, &cmd_[slot_[k]]);
  }
  if (has_gpio_) {
    out.emplace_back("legs", "enable", &legs_enable_cmd_);
  }
  if (has_estop_) {
    out.emplace_back("legs", "estop", &legs_estop_cmd_);
  }
  return out;
}

void VectorSystem::drain()
{
  for (int n = 0; n < 256; ++n) {
    auto f = bus_.receive();
    if (!f) {
      return;
    }
    if (auto s = vector::can::decode_leg_state(*f)) {
      const int l = *vector::can::leg_of(f->id);
      pos_[l * 3 + 0] = s->coxa * kDeg;
      pos_[l * 3 + 1] = s->femur * kDeg;
      pos_[l * 3 + 2] = s->tibia * kDeg;
      last_rx_[l] = Clock::now();
      seen_[l] = true;
    } else if (auto st = vector::can::decode_leg_status(*f)) {
      leg_state_[*vector::can::leg_of(f->id)] = st->state;
    }
  }
}

hardware_interface::return_type VectorSystem::read(const rclcpp::Time &, const rclcpp::Duration & period)
{
  const auto before = pos_;
  drain();
  const double dt = period.seconds();
  for (int i = 0; i < kJoints; ++i) {
    vel_[i] = dt > 0 ? (pos_[i] - before[i]) / dt : 0.0;
  }

  int active = 0;
  for (auto st : leg_state_) {
    active += st == vector::can::kActive;
  }
  legs_active_ = active == kLegs ? 1.0 : active == 0 ? 0.0 : 0.5;

  const auto now = Clock::now();
  for (int l = 0; l < kLegs; ++l) {
    if (now - last_rx_[l] > timeout_) {
      RCLCPP_ERROR(log_, "leg %s silent for more than %ld ms", kLegNames[l], timeout_.count());
      return hardware_interface::return_type::ERROR;
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type VectorSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  // NaN until the power controller has said anything: keep the legs on, no e-stop.
  // An e-stop also drops enable, so releasing it leaves the legs off until they are
  // switched on again (the gait node's ~/wake).
  const bool estop = !std::isnan(legs_estop_cmd_) && legs_estop_cmd_ >= 0.5;
  send_all(!estop && (std::isnan(legs_enable_cmd_) || legs_enable_cmd_ >= 0.5), estop);
  return hardware_interface::return_type::OK;
}

void VectorSystem::send_all(bool enable, bool estop)
{
  bus_.send(vector::can::encode_sync(sync_counter_++, vector::can::Mode::Walk, estop));
  for (int l = 0; l < kLegs; ++l) {
    vector::can::LegCmd c;
    c.coxa = cmd_[l * 3 + 0] / kDeg;
    c.femur = cmd_[l * 3 + 1] / kDeg;
    c.tibia = cmd_[l * 3 + 2] / kDeg;
    c.enable = enable;
    c.counter = cmd_counter_;
    bus_.send(vector::can::encode_leg_cmd(l, c));
  }
  cmd_counter_++;
}

}  // namespace vector_hw

PLUGINLIB_EXPORT_CLASS(vector_hw::VectorSystem, hardware_interface::SystemInterface)
