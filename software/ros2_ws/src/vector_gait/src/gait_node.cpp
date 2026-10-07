// Runs the gait at a fixed rate: cmd_vel in, 18 joint positions out to the
// forward command controller, plus odom -> base_link from the commanded motion
// (also as nav_msgs/Odometry on odom/legs, for the EKF) and foot trails for rviz. Body pose via the body_* parameters (live).
// With level:=true the IMU tilt is fed back into the body pose to keep it level.
// A cmd_vel with only linear.z set (teleop keys t/b) steps the body height up/down by
// height_step and leaves the walking command alone.
//
// Sentinel Stance (~/sentinel, ~/wake services): stop, step the feet out to sentinel_reach
// one leg at a time (wave order, so five legs carry the body), lower it to sentinel_height,
// switch the legs off through the leg_power GPIO controller; waking powers them, waits
// until they all report active, raises the body, steps the feet back in and walks again.
// Without the GPIO (Gazebo) it only does the moves.
//
// A leg that drops out while walking (fault, lost power) halts the gait: the other legs
// finish their step and stand. ~/sentinel then ~/wake power-cycles the legs, which clears
// the fault if its cause is gone. ~/estop (true) sets the e-stop flag on the bus, which
// switches every leg off at once, and puts the node in sentinel; after ~/estop (false),
// ~/wake stands it up again.
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include "control_msgs/msg/dynamic_interface_group_values.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "visualization_msgs/msg/marker_array.hpp"
#include "vector_gait/gait.hpp"
#include "vector_gait/leveler.hpp"

using namespace std::chrono_literals;

class GaitNode : public rclcpp::Node
{
public:
  GaitNode()
  : Node("gait_node")
  {
    vector::LegGeometry geo;
    geo.coxa = declare_parameter("coxa", 0.05);
    geo.femur = declare_parameter("femur", 0.08);
    geo.tibia = declare_parameter("tibia", 0.12);

    const auto mx = declare_parameter("mount_x", std::vector<double>{});
    const auto my = declare_parameter("mount_y", std::vector<double>{});
    const auto myaw = declare_parameter("mount_yaw", std::vector<double>{});
    if (mx.size() != vector::kLegs || my.size() != vector::kLegs || myaw.size() != vector::kLegs) {
      throw std::runtime_error("mount_x, mount_y and mount_yaw need 6 values each");
    }
    std::array<vector::LegMount, vector::kLegs> mounts;
    for (int i = 0; i < vector::kLegs; ++i) {
      mounts[i] = {mx[i], my[i], myaw[i]};
    }

    vector::GaitParams p;
    p.period = declare_parameter("period", p.period);
    p.ripple_period = declare_parameter("ripple_period", p.ripple_period);
    p.wave_period = declare_parameter("wave_period", p.wave_period);
    p.step_height = declare_parameter("step_height", p.step_height);
    p.body_height = declare_parameter("body_height", p.body_height);
    p.reach = declare_parameter("reach", p.reach);
    p.max_stride = declare_parameter("max_stride", p.max_stride);
    p.accel = declare_parameter("accel", p.accel);
    p.turn_accel = declare_parameter("turn_accel", p.turn_accel);
    const auto gait_name = declare_parameter("gait", std::string("tripod"));
    if (!vector::parse_gait(gait_name, p.type)) {
      throw std::runtime_error("unknown gait: " + gait_name);
    }

    rate_ = declare_parameter("rate", 200.0);
    // 0 keeps the last command until a new one arrives (teleop_twist_keyboard publishes only
    // on key presses). > 0 stops that long after the last message: ~0.6 s gives hold-to-move
    // with keyboard auto-repeat; joystick/ELRS sources stream and can use ~0.3 s.
    cmd_timeout_ = declare_parameter("cmd_timeout", 0.0);
    height_step_ = declare_parameter("height_step", 0.005);
    height_min_ = declare_parameter("height_min", -0.04);
    height_max_ = declare_parameter("height_max", 0.03);
    p.touchdown = declare_parameter("touchdown", false);
    p.probe_depth = declare_parameter("probe_depth", p.probe_depth);
    p.touch_after = declare_parameter("touch_after", 0.7);
    contact_threshold_ = declare_parameter("contact_threshold", 0.6);
    sentinel_height_ = declare_parameter("sentinel_height", 0.03);
    sentinel_reach_ = declare_parameter("sentinel_reach", 0.14);
    stand_reach_ = p.reach;
    power_timeout_ = declare_parameter("power_timeout", 10.0);
    // Off in Gazebo, where odom -> base_link comes from the simulator's ground truth.
    publish_odom_ = declare_parameter("publish_odom_tf", true);

    level_ = declare_parameter("level", false);
    leveler_ = std::make_unique<vector::Leveler>(
      declare_parameter("level_gain", 2.0), declare_parameter("level_max", 0.3));
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "imu", rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::Imu::SharedPtr msg) {
        const auto & q = msg->orientation;
        imu_roll_ = std::atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y));
        imu_pitch_ = std::asin(std::clamp(2 * (q.w * q.y - q.z * q.x), -1.0, 1.0));
        last_imu_ = now();
        have_imu_ = true;
      });

    gait_ = std::make_unique<vector::Gait>(geo, mounts, p);

    // Foot contact for touchdown. "position": the measured joints put the foot higher than
    // commanded, i.e. something is in the way (the pots on the robot). "load": femur + tibia
    // torque (Gazebo effort, leg current on the robot).
    contact_from_ = declare_parameter("contact_from", std::string("position"));
    contact_height_ = declare_parameter("contact_height", 0.003);
    geo_ = geo;
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10, [this](sensor_msgs::msg::JointState::SharedPtr msg) {on_joints(*msg);});

    for (const char * name : kPoseParams) {
      declare_parameter(name, 0.0);
    }
    update_pose();

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, [this](geometry_msgs::msg::Twist::SharedPtr msg) {
        const bool height_only = msg->linear.z != 0.0 && msg->linear.x == 0.0 &&
          msg->linear.y == 0.0 && msg->angular.z == 0.0;
        if (mode_ != Mode::Walk) {
          return;  // sitting down or getting up: ignore the keys
        }
        if (height_only) {
          height_ = std::clamp(height_ + (msg->linear.z > 0 ? height_step_ : -height_step_),
              height_min_, height_max_);
          pose_dirty_ = true;
          RCLCPP_INFO(get_logger(), "body height %+.0f mm", height_ * 1000);
          return;
        }
        cmd_ = {msg->linear.x, msg->linear.y, msg->angular.z};
        last_cmd_ = now();
      });
    joint_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("leg_controller/commands", 10);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom/legs", 10);

    power_pub_ = create_publisher<control_msgs::msg::DynamicInterfaceGroupValues>("leg_power/commands", 10);
    power_sub_ = create_subscription<control_msgs::msg::DynamicInterfaceGroupValues>(
      "leg_power/gpio_states", 10, [this](control_msgs::msg::DynamicInterfaceGroupValues::SharedPtr msg) {
        for (size_t i = 0; i < msg->interface_groups.size() && i < msg->interface_values.size(); ++i) {
          const auto & v = msg->interface_values[i];
          for (size_t k = 0; k < v.interface_names.size() && k < v.values.size(); ++k) {
            if (msg->interface_groups[i] == "legs" && v.interface_names[k] == "enable") {
              on_legs_active(v.values[k]);
            }
          }
        }
      });
    mode_pub_ = create_publisher<std_msgs::msg::String>(
      "~/mode", rclcpp::QoS(1).transient_local());
    sentinel_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/sentinel", [this](const std_srvs::srv::Trigger::Request::SharedPtr,
      std_srvs::srv::Trigger::Response::SharedPtr res) {
        res->success = mode_ == Mode::Walk || mode_ == Mode::Halted;
        res->message = res->success ? "sitting down" : "only from walking or halted";
        if (res->success) {
          set_mode(Mode::Stopping);
        }
      });
    wake_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/wake", [this](const std_srvs::srv::Trigger::Request::SharedPtr,
      std_srvs::srv::Trigger::Response::SharedPtr res) {
        res->success = mode_ == Mode::Sentinel && !estop_;
        res->message = res->success ? "waking up" : estop_ ? "e-stop engaged" : "only from sentinel";
        if (res->success) {
          power(true);
          set_mode(Mode::Powering);
        }
      });
    estop_srv_ = create_service<std_srvs::srv::SetBool>(
      "~/estop", [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
      std_srvs::srv::SetBool::Response::SharedPtr res) {
        estop(req->data);
        res->success = true;
        res->message = req->data ? "e-stop: legs off" : "released, ~/wake to stand up";
      });
    set_mode(Mode::Walk);
    tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    trail_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("foot_trails", 10);

    param_cb_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) {
        rcl_interfaces::msg::SetParametersResult res;
        res.successful = true;
        for (const auto & prm : params) {
          if (prm.get_name().rfind("body_", 0) == 0) {
            pose_dirty_ = true;
            continue;
          }
          if (prm.get_name() == "cmd_timeout") {
            cmd_timeout_ = prm.as_double();
            continue;
          }
          if (prm.get_name() == "level") {
            level_ = prm.as_bool();
            leveler_->reset();
            pose_dirty_ = true;
            continue;
          }
          if (prm.get_name() != "gait") {
            continue;
          }
          vector::GaitType t;
          if (!gait_->standing()) {
            res.successful = false;
            res.reason = "stop walking before switching gait";
          } else if (!vector::parse_gait(prm.as_string(), t)) {
            res.successful = false;
            res.reason = "use tripod, ripple or wave";
          } else if (spread_) {
            walk_type_ = t;  // sitting or getting up in wave order: the next walk uses it
          } else {
            gait_->set_type(t);
          }
        }
        return res;
      });

    last_cmd_ = now();
    last_tick_ = now();
    // ROS time, so it follows /clock in simulation.
    timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(1.0 / rate_), [this] {tick();});
    RCLCPP_INFO(get_logger(), "gait %s at %.0f Hz", gait_name.c_str(), rate_);
  }

private:
  void tick()
  {
    const auto t = now();
    const double dt = (t - last_tick_).seconds();
    last_tick_ = t;
    if (dt <= 0 || dt > 0.1) {
      return;
    }

    if (level_ && have_imu_ && (t - last_imu_).seconds() < 0.2) {
      leveler_->update(imu_roll_, imu_pitch_, dt);
      pose_dirty_ = true;
    }
    if (pose_dirty_) {
      update_pose();
    }
    if (cmd_timeout_ > 0 && (t - last_cmd_).seconds() > cmd_timeout_) {
      cmd_ = {};  // gone for good, so it can't come back if the timeout is changed later
    }
    sentinel_step(t);
    const vector::Twist2D cmd = mode_ == Mode::Walk ? cmd_ : vector::Twist2D{};
    if (!gait_->update(cmd, dt)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "foot target out of reach");
    }

    history_i_ = (history_i_ + 1) % cmd_history_.size();
    cmd_history_[history_i_] = gait_->joints();

    std_msgs::msg::Float64MultiArray out;
    out.data.reserve(3 * vector::kLegs);
    for (const auto & q : gait_->joints()) {
      out.data.push_back(q.coxa);
      out.data.push_back(q.femur);
      out.data.push_back(q.tibia);
    }
    joint_pub_->publish(out);

    // Dead reckoning from the applied command, good enough to see it walk in rviz.
    const auto & v = gait_->applied();
    x_ += (v.vx * std::cos(yaw_) - v.vy * std::sin(yaw_)) * dt;
    y_ += (v.vx * std::sin(yaw_) + v.vy * std::cos(yaw_)) * dt;
    yaw_ += v.wz * dt;

    // odom -> walking frame (x, y, yaw, hip height) -> body (pose offset and tilt)
    const auto & b = gait_->body_pose();
    const double c = std::cos(yaw_), s = std::sin(yaw_);
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = t;
    tf.header.frame_id = "odom";
    tf.child_frame_id = "base_link";
    tf.transform.translation.x = x_ + c * b.x - s * b.y;
    tf.transform.translation.y = y_ + s * b.x + c * b.y;
    tf.transform.translation.z = gait_->params().body_height + b.z;
    tf.transform.rotation = rpy_to_quat(b.roll, b.pitch, yaw_ + b.yaw);
    if (publish_odom_) {
      tf_->sendTransform(tf);
    }

    if (++tick_count_ % 4 == 0) {
      publish_trails(t, c, s);
      publish_odom(t, tf, v);
    }
  }

  // The applied velocity with a variance that grows with speed (slip, uneven ground).
  // The pose is plain dead reckoning, so the EKF should only take the twist.
  void publish_odom(const rclcpp::Time & t, const geometry_msgs::msg::TransformStamped & tf, const vector::Twist2D & v)
  {
    nav_msgs::msg::Odometry o;
    o.header.stamp = t;
    o.header.frame_id = "odom";
    o.child_frame_id = "base_link";
    o.pose.pose.position.x = tf.transform.translation.x;
    o.pose.pose.position.y = tf.transform.translation.y;
    o.pose.pose.position.z = tf.transform.translation.z;
    o.pose.pose.orientation = tf.transform.rotation;
    for (int i = 0; i < 6; ++i) {
      o.pose.covariance[i * 7] = 1e3;
    }
    o.twist.twist.linear.x = v.vx;
    o.twist.twist.linear.y = v.vy;
    o.twist.twist.angular.z = v.wz;
    const double sv = 0.01 + 0.2 * std::hypot(v.vx, v.vy), sw = 0.02 + 0.2 * std::abs(v.wz);
    o.twist.covariance[0] = o.twist.covariance[7] = sv * sv;
    o.twist.covariance[14] = o.twist.covariance[21] = o.twist.covariance[28] = 1e-4;  // no z motion or tilt rate
    o.twist.covariance[35] = sw * sw;
    odom_pub_->publish(o);
  }

  void on_joints(const sensor_msgs::msg::JointState & msg)
  {
    std::array<vector::JointAngles, vector::kLegs> meas{};
    std::array<double, vector::kLegs> load{};
    std::array<int, vector::kLegs> seen{};
    const bool efforts = msg.effort.size() == msg.name.size();
    for (size_t k = 0; k < msg.name.size() && k < msg.position.size(); ++k) {
      const auto & n = msg.name[k];
      for (int l = 0; l < vector::kLegs; ++l) {
        if (n.rfind(vector::kLegNames[l], 0) != 0) {
          continue;
        }
        const double q = msg.position[k];
        if (n.find("_coxa") != std::string::npos) {meas[l].coxa = q; seen[l]++;}
        if (n.find("_femur") != std::string::npos) {meas[l].femur = q; seen[l]++;}
        if (n.find("_tibia") != std::string::npos) {meas[l].tibia = q; seen[l]++;}
        if (efforts && n.find("_coxa") == std::string::npos) {load[l] += std::abs(msg.effort[k]);}
      }
    }
    std::array<bool, vector::kLegs> contact{};
    for (int l = 0; l < vector::kLegs; ++l) {
      if (contact_from_ == "load") {
        contact[l] = efforts && load[l] > contact_threshold_;
      } else if (seen[l] == 3) {
        // blocked: higher than the highest the foot was told to be over the last 40 ms,
        // which servo lag on the way down can't explain
        double highest = -1e9;
        for (const auto & c : cmd_history_) {
          highest = std::max(highest, vector::forward(geo_, c[l]).z);
        }
        contact[l] = vector::forward(geo_, meas[l]).z - highest > contact_height_;
      }
    }
    gait_->set_contact(contact);
  }

  enum class Mode { Walk, Stopping, Spreading, Lowering, Sentinel, Powering, Raising, Gathering, Halted };

  void set_mode(Mode m)
  {
    static const char * names[] = {"walk", "stopping", "spreading", "lowering", "sentinel", "powering", "raising",
      "gathering", "halted"};
    mode_ = m;
    mode_t_ = now();
    std_msgs::msg::String msg;
    msg.data = names[static_cast<int>(m)];
    mode_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "mode: %s", msg.data.c_str());
  }

  // The GPIO controller applies only the latest message each cycle, so every message carries
  // both interfaces: an e-stop release followed at once by ~/wake must not lose the release.
  void legs_gpio(const std::vector<std::string> & names, const std::vector<double> & values)
  {
    for (size_t i = 0; i < names.size(); ++i) {
      (names[i] == "estop" ? gpio_estop_ : gpio_enable_) = values[i];
    }
    control_msgs::msg::DynamicInterfaceGroupValues msg;
    msg.interface_groups = {"legs"};
    msg.interface_values.resize(1);
    msg.interface_values[0].interface_names = {"enable", "estop"};
    msg.interface_values[0].values = {gpio_enable_, gpio_estop_};
    power_pub_->publish(msg);
  }

  void power(bool on)
  {
    legs_were_up_ = false;
    legs_gpio({"enable"}, {on ? 1.0 : 0.0});
  }

  void on_legs_active(double active)
  {
    legs_active_ = active;
    have_power_ = true;
    if (mode_ != Mode::Walk) {
      return;
    }
    if (active > 0.999) {
      legs_were_up_ = true;
    } else if (legs_were_up_) {
      cmd_ = {};
      RCLCPP_ERROR(get_logger(), "a leg dropped out (see /diagnostics), stopped; "
        "~/sentinel then ~/wake to power-cycle the legs");
      set_mode(Mode::Halted);
    }
  }

  void estop(bool on)
  {
    estop_ = on;
    if (!on) {
      legs_gpio({"estop"}, {0.0});
      RCLCPP_WARN(get_logger(), "e-stop released, ~/wake to stand up");
      return;
    }
    legs_were_up_ = false;
    legs_gpio({"enable", "estop"}, {0.0, 1.0});
    cmd_ = {};
    // the body is on the ground by now; command the sentinel pose so waking starts from it
    spread_feet(false);
    sentinel_z_ = (sentinel_height_ - gait_->params().body_height) - (get_parameter("body_z").as_double() + height_);
    pose_dirty_ = true;
    RCLCPP_ERROR(get_logger(), "e-stop");
    set_mode(Mode::Sentinel);
  }

  bool body_at(double z) const {return std::abs(gait_->body_pose().z - z) < 1e-4;}

  // Feet out to sentinel_reach, where the hip can get down to sentinel_height within the
  // joint limits, in wave order (one leg in the air at a time). step false: set them there
  // at once, for legs that are already off.
  void spread_feet(bool step)
  {
    if (!spread_) {
      walk_type_ = gait_->params().type;
      spread_ = true;
    }
    gait_->set_type(vector::GaitType::Wave);
    gait_->set_reach(sentinel_reach_, step);
  }

  void sentinel_step(const rclcpp::Time & t)
  {
    const double down = sentinel_height_ - gait_->params().body_height;
    const double up = get_parameter("body_z").as_double() + height_;
    switch (mode_) {
      case Mode::Stopping:
        if (gait_->standing()) {
          spread_feet(true);
          set_mode(Mode::Spreading);
        }
        break;
      case Mode::Spreading:
        if (gait_->standing()) {
          sentinel_z_ = down - up;
          pose_dirty_ = true;
          set_mode(Mode::Lowering);
        }
        break;
      case Mode::Lowering:
        if (body_at(down)) {
          power(false);
          set_mode(Mode::Sentinel);
        }
        break;
      case Mode::Powering:
        if (!have_power_ || legs_active_ > 0.999) {
          sentinel_z_ = 0;
          pose_dirty_ = true;
          set_mode(Mode::Raising);
        } else if ((t - mode_t_).seconds() > power_timeout_) {
          RCLCPP_ERROR(get_logger(), "legs did not come back within %.0f s, staying down", power_timeout_);
          power(false);
          set_mode(Mode::Sentinel);
        }
        break;
      case Mode::Raising:
        if (body_at(up)) {
          gait_->set_reach(stand_reach_, true);
          set_mode(Mode::Gathering);
        }
        break;
      case Mode::Gathering:
        if (gait_->standing()) {
          gait_->set_type(walk_type_);
          spread_ = false;
          set_mode(Mode::Walk);
        }
        break;
      default:
        break;
    }
  }

  static geometry_msgs::msg::Quaternion rpy_to_quat(double roll, double pitch, double yaw)
  {
    const double cr = std::cos(roll / 2), sr = std::sin(roll / 2);
    const double cp = std::cos(pitch / 2), sp = std::sin(pitch / 2);
    const double cy = std::cos(yaw / 2), sy = std::sin(yaw / 2);
    geometry_msgs::msg::Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;
    return q;
  }

  void update_pose()
  {
    vector::BodyPose b;
    b.x = get_parameter("body_x").as_double();
    b.y = get_parameter("body_y").as_double();
    b.z = get_parameter("body_z").as_double() + height_ + sentinel_z_;
    b.roll = get_parameter("body_roll").as_double() + (level_ ? leveler_->roll() : 0.0);
    b.pitch = get_parameter("body_pitch").as_double() + (level_ ? leveler_->pitch() : 0.0);
    b.yaw = get_parameter("body_yaw").as_double();
    gait_->set_body_pose(b);
    pose_dirty_ = false;
  }

  // Last few seconds of each foot's path in the odom frame.
  void publish_trails(const rclcpp::Time & t, double c, double s)
  {
    static const std::array<std::array<float, 3>, vector::kLegs> colors = {{
      {1.0f, 0.3f, 0.3f}, {1.0f, 0.7f, 0.2f}, {1.0f, 1.0f, 0.3f},
      {0.3f, 0.6f, 1.0f}, {0.3f, 1.0f, 0.8f}, {0.7f, 0.4f, 1.0f}}};
    const double h = gait_->params().body_height;

    visualization_msgs::msg::MarkerArray arr;
    for (int l = 0; l < vector::kLegs; ++l) {
      const auto & f = gait_->feet()[l];
      geometry_msgs::msg::Point p;
      p.x = x_ + c * f.x - s * f.y;
      p.y = y_ + s * f.x + c * f.y;
      p.z = h + f.z;
      trails_[l].push_back(p);
      if (trails_[l].size() > 150) {
        trails_[l].pop_front();
      }

      visualization_msgs::msg::Marker m;
      m.header.stamp = t;
      m.header.frame_id = "odom";
      m.ns = vector::kLegNames[l];
      m.type = visualization_msgs::msg::Marker::LINE_STRIP;
      m.scale.x = 0.003;
      m.color.r = colors[l][0];
      m.color.g = colors[l][1];
      m.color.b = colors[l][2];
      m.color.a = 1.0;
      m.pose.orientation.w = 1.0;
      m.points.assign(trails_[l].begin(), trails_[l].end());
      arr.markers.push_back(m);
    }
    trail_pub_->publish(arr);
  }

  static constexpr std::array<const char *, 6> kPoseParams = {
    "body_x", "body_y", "body_z", "body_roll", "body_pitch", "body_yaw"};

  std::unique_ptr<vector::Gait> gait_;
  bool pose_dirty_ = false;
  bool publish_odom_ = true;
  bool level_ = false, have_imu_ = false;
  double imu_roll_ = 0, imu_pitch_ = 0;
  rclcpp::Time last_imu_;
  std::unique_ptr<vector::Leveler> leveler_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  unsigned tick_count_ = 0;
  std::array<std::deque<geometry_msgs::msg::Point>, vector::kLegs> trails_;
  vector::Twist2D cmd_;
  double rate_ = 200, cmd_timeout_ = 0.0;
  double height_ = 0, height_step_ = 0.005, height_min_ = -0.04, height_max_ = 0.03;
  double contact_threshold_ = 0.6, contact_height_ = 0.003;
  std::string contact_from_ = "position";
  vector::LegGeometry geo_;
  std::array<std::array<vector::JointAngles, vector::kLegs>, 8> cmd_history_{};
  size_t history_i_ = 0;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  Mode mode_ = Mode::Walk;
  rclcpp::Time mode_t_;
  double sentinel_height_ = 0.03, power_timeout_ = 10.0, sentinel_z_ = 0, legs_active_ = 1.0;
  double gpio_enable_ = 1.0, gpio_estop_ = 0.0;  // last values sent to leg_power
  double sentinel_reach_ = 0.14, stand_reach_ = 0.108;
  bool spread_ = false;  // feet at sentinel_reach, walk_type_ holds the gait to go back to
  vector::GaitType walk_type_ = vector::GaitType::Tripod;
  bool have_power_ = false, legs_were_up_ = false, estop_ = false;
  rclcpp::Publisher<control_msgs::msg::DynamicInterfaceGroupValues>::SharedPtr power_pub_;
  rclcpp::Subscription<control_msgs::msg::DynamicInterfaceGroupValues>::SharedPtr power_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr sentinel_srv_, wake_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr estop_srv_;
  double x_ = 0, y_ = 0, yaw_ = 0;
  rclcpp::Time last_cmd_, last_tick_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr trail_pub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GaitNode>());
  rclcpp::shutdown();
  return 0;
}
