// VL53L8CX on the nose board (CM5 I2C through the PCA9306):
//   nose/tof/points   sensor_msgs/PointCloud2, one point per valid zone, frame nose_tof
//   nose/tof/depth    sensor_msgs/Image 8x8 32FC1, metres along each zone's ray, NaN = no
//                     target; row 0 at the top, column 0 on the robot's left, looking out
// It finds the sensor on its own, also one plugged in later, and restarts it after
// repeated errors. The firmware upload takes about a second at 1 MHz.
#include <fcntl.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

extern "C" {
#include "vl53l8cx_api.h"
}

using namespace std::chrono_literals;

namespace
{
constexpr int kSide = 8;
constexpr int kZones = kSide * kSide;
// target status 5 = valid, 9 = valid but merged with a wider pulse; the rest aren't ranges
bool valid(uint8_t status) {return status == 5 || status == 9;}
}  // namespace

class TofFront : public rclcpp::Node
{
public:
  TofFront()
  : Node("tof_front")
  {
    bus_ = declare_parameter("i2c_bus", std::string("/dev/i2c-1"));
    frame_ = declare_parameter("frame_id", std::string("nose_tof"));
    rate_ = static_cast<int>(declare_parameter("rate_hz", 15));  // 8x8 goes up to 15 Hz
    fov_ = declare_parameter("fov_deg", 45.0) * M_PI / 180.0;
    // the lens flips the zone grid; check on the bench with a hand in one corner
    flip_x_ = declare_parameter("flip_x", true);
    flip_y_ = declare_parameter("flip_y", false);
    dev_.platform.fd = -1;
    points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("nose/tof/points", rclcpp::SensorDataQoS());
    depth_pub_ = create_publisher<sensor_msgs::msg::Image>("nose/tof/depth", rclcpp::SensorDataQoS());
    timer_ = create_wall_timer(5ms, [this] {tick();});
  }

  ~TofFront() override
  {
    if (ranging_) {
      vl53l8cx_stop_ranging(&dev_);
    }
    if (dev_.platform.fd >= 0) {
      close(dev_.platform.fd);
    }
  }

private:
  bool start()
  {
    if (dev_.platform.fd < 0) {
      dev_.platform.fd = open(bus_.c_str(), O_RDWR);
      if (dev_.platform.fd < 0) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 30000, "%s: %s", bus_.c_str(), std::strerror(errno));
        return false;
      }
    }
    dev_.platform.address = VL53L8CX_DEFAULT_I2C_ADDRESS;
    uint8_t alive = 0;
    if (vl53l8cx_is_alive(&dev_, &alive) || !alive) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 30000, "no VL53L8CX on %s", bus_.c_str());
      return false;
    }
    const auto t = now();
    if (vl53l8cx_init(&dev_) || vl53l8cx_set_resolution(&dev_, VL53L8CX_RESOLUTION_8X8) ||
      vl53l8cx_set_ranging_frequency_hz(&dev_, static_cast<uint8_t>(rate_)) ||
      vl53l8cx_set_target_order(&dev_, VL53L8CX_TARGET_ORDER_CLOSEST) ||
      vl53l8cx_set_ranging_mode(&dev_, VL53L8CX_RANGING_MODE_CONTINUOUS) || vl53l8cx_start_ranging(&dev_))
    {
      RCLCPP_ERROR(get_logger(), "VL53L8CX set-up failed");
      return false;
    }
    RCLCPP_INFO(get_logger(), "VL53L8CX ranging, 8x8 at %d Hz (set-up %.1f s)", rate_, (now() - t).seconds());
    return true;
  }

  void tick()
  {
    const auto t = now();
    if (!ranging_) {
      if ((t - last_try_).seconds() < 5.0) {
        return;
      }
      last_try_ = t;
      ranging_ = start();
      errors_ = 0;
      return;
    }
    uint8_t ready = 0;
    if (vl53l8cx_check_data_ready(&dev_, &ready)) {
      fail();
      return;
    }
    if (!ready) {
      return;
    }
    VL53L8CX_ResultsData r;
    if (vl53l8cx_get_ranging_data(&dev_, &r)) {
      fail();
      return;
    }
    errors_ = 0;
    publish(r, t);
  }

  void fail()
  {
    if (++errors_ >= 10) {
      RCLCPP_WARN(get_logger(), "VL53L8CX stopped answering, restarting it");
      ranging_ = false;
    }
  }

  // Zone (row, col) as published: row 0 top, col 0 left looking out
  void publish(const VL53L8CX_ResultsData & r, const rclcpp::Time & t)
  {
    const float nan = std::numeric_limits<float>::quiet_NaN();
    sensor_msgs::msg::Image depth;
    depth.header.stamp = t;
    depth.header.frame_id = frame_;
    depth.height = depth.width = kSide;
    depth.encoding = "32FC1";
    depth.step = kSide * sizeof(float);
    depth.data.resize(kZones * sizeof(float));
    auto * d = reinterpret_cast<float *>(depth.data.data());

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = depth.header;
    sensor_msgs::PointCloud2Modifier mod(cloud);
    mod.setPointCloud2FieldsByString(1, "xyz");
    mod.resize(kZones);
    sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
    int n = 0;
    const double step = fov_ / kSide;
    for (int i = 0; i < kZones; ++i) {
      int row = i / kSide, col = i % kSide;
      if (flip_x_) {
        col = kSide - 1 - col;
      }
      if (flip_y_) {
        row = kSide - 1 - row;
      }
      const bool ok = r.nb_target_detected[i] > 0 && valid(r.target_status[i]);
      const float m = ok ? r.distance_mm[i] / 1000.0f : nan;
      d[row * kSide + col] = m;
      if (!ok) {
        continue;
      }
      // ray through the zone centre; x forward, y left, z up
      const double left = (3.5 - col) * step, up = (3.5 - row) * step;
      const double dx = 1.0, dy = std::tan(left), dz = std::tan(up), norm = std::sqrt(dx * dx + dy * dy + dz * dz);
      *x = static_cast<float>(m * dx / norm);
      *y = static_cast<float>(m * dy / norm);
      *z = static_cast<float>(m * dz / norm);
      ++x, ++y, ++z, ++n;
    }
    mod.resize(n);
    depth_pub_->publish(depth);
    points_pub_->publish(cloud);
  }

  std::string bus_, frame_;
  int rate_ = 15;
  double fov_ = 0.785;
  bool flip_x_ = true, flip_y_ = false;
  VL53L8CX_Configuration dev_{};
  bool ranging_ = false;
  int errors_ = 0;
  rclcpp::Time last_try_{0, 0, RCL_ROS_TIME};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TofFront>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
