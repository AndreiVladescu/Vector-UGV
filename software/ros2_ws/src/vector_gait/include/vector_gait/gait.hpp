// Gait generator. Plain C++, no ROS.
//
// Feet are tracked in the walking frame: level, at nominal hip height, moving with
// the robot. The body can be shifted and tilted relative to it (BodyPose), which only
// changes the IK, not where the feet are. Stance feet move
// opposite to the body so they stay fixed on the ground; swing feet travel from
// lift-off to a touchdown point ahead of the neutral stance, on a sine lift.
// When the command goes to zero, legs finish their swing back to neutral and stay down.
#pragma once

#include <array>
#include <string>

#include "vector_gait/kinematics.hpp"

namespace vector
{

constexpr int kLegs = 6;
// Leg order used everywhere (CAN node ids 1..6, joint command order).
constexpr std::array<const char *, kLegs> kLegNames = {"L1", "L2", "L3", "R1", "R2", "R3"};

enum class GaitType { Tripod, Ripple, Wave };

bool parse_gait(const std::string & name, GaitType & out);

struct LegMount
{
  double x = 0, y = 0, yaw = 0;
};

// Body offset from the walking frame. Rotation order: yaw, then pitch, then roll.
struct BodyPose
{
  double x = 0, y = 0, z = 0;
  double roll = 0, pitch = 0, yaw = 0;
};

struct GaitParams
{
  GaitType type = GaitType::Tripod;
  double period = 1.0;        // s, one full cycle
  double step_height = 0.035; // m
  double body_height = 0.10;  // m, hip plane above ground
  double reach = 0.13;        // m, neutral foot distance from the hip
  double max_stride = 0.06;   // m, commands are scaled down to respect this
  double pose_speed = 0.05;   // m/s, body shift rate
  double pose_turn = 0.5;     // rad/s, body tilt rate
};

struct Twist2D
{
  double vx = 0, vy = 0, wz = 0;
};

class Gait
{
public:
  Gait(const LegGeometry & geometry, const std::array<LegMount, kLegs> & mounts, const GaitParams & params);

  // Advance by dt. Returns false if a foot target was out of reach (that leg keeps its last angles).
  bool update(const Twist2D & cmd, double dt);

  void set_type(GaitType type);
  // Target body pose; the actual pose ramps toward it at pose_speed / pose_turn.
  void set_body_pose(const BodyPose & target) {pose_target_ = target;}
  const BodyPose & body_pose() const {return pose_;}
  // Walking frame -> body frame, for a point.
  Vec3 to_body(const Vec3 & p) const;
  const GaitParams & params() const {return params_;}

  const std::array<JointAngles, kLegs> & joints() const {return joints_;}
  const std::array<Vec3, kLegs> & feet() const {return feet_;}   // walking frame
  const std::array<Vec3, kLegs> & neutral() const {return neutral_;}
  bool swinging(int leg) const {return swinging_[leg];}
  bool standing() const {return standing_;}
  // Command actually applied after stride limiting.
  const Twist2D & applied() const {return applied_;}

  double duty() const;

private:
  Twist2D limit(const Twist2D & cmd) const;
  Vec3 to_leg(int leg, const Vec3 & body) const;
  bool ramp_pose(double dt);  // true if the pose changed this step
  bool solve(int leg);

  LegGeometry geometry_;
  std::array<LegMount, kLegs> mounts_;
  GaitParams params_;

  double phase_ = 0;
  std::array<double, kLegs> offset_{};
  std::array<Vec3, kLegs> neutral_{};
  std::array<Vec3, kLegs> feet_{};
  std::array<Vec3, kLegs> liftoff_{};
  std::array<bool, kLegs> swinging_{};
  std::array<bool, kLegs> skip_{};  // leg stays planted through this swing window
  std::array<JointAngles, kLegs> joints_{};
  BodyPose pose_, pose_target_;
  Twist2D applied_;
  bool standing_ = true;
};

}  // namespace vector
