// Runs the gait at a fixed rate: cmd_vel in, 18 joint positions out to the
// forward command controller, plus odom -> base_link from the commanded motion
// and foot trails for rviz. Body pose via the body_* parameters (live).
// With level:=true the IMU tilt is fed back into the body pose to keep it level.
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
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
    p.step_height = declare_parameter("step_height", p.step_height);
    p.body_height = declare_parameter("body_height", p.body_height);
    p.reach = declare_parameter("reach", p.reach);
    p.max_stride = declare_parameter("max_stride", p.max_stride);
    const auto gait_name = declare_parameter("gait", std::string("tripod"));
    if (!vector::parse_gait(gait_name, p.type)) {
      throw std::runtime_error("unknown gait: " + gait_name);
    }

    rate_ = declare_parameter("rate", 200.0);
    cmd_timeout_ = declare_parameter("cmd_timeout", 0.5);
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

    for (const char * name : kPoseParams) {
      declare_parameter(name, 0.0);
    }
    update_pose();

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, [this](geometry_msgs::msg::Twist::SharedPtr msg) {
        cmd_ = {msg->linear.x, msg->linear.y, msg->angular.z};
        last_cmd_ = now();
      });
    joint_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("leg_controller/commands", 10);
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
    const vector::Twist2D cmd = (t - last_cmd_).seconds() > cmd_timeout_ ? vector::Twist2D{} : cmd_;
    if (!gait_->update(cmd, dt)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "foot target out of reach");
    }

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
    b.z = get_parameter("body_z").as_double();
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
  double rate_ = 200, cmd_timeout_ = 0.5;
  double x_ = 0, y_ = 0, yaw_ = 0;
  rclcpp::Time last_cmd_, last_tick_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_pub_;
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
