// Leg kinematics. Plain C++, no ROS.
//
// Leg frame: origin on the coxa axis, x pointing outward at coxa = 0, z up.
// Positive femur/tibia angles lift the segment. Tibia angle is relative to the femur.
#pragma once

#include <optional>

namespace vector
{

struct Vec3
{
  double x = 0, y = 0, z = 0;
};

struct LegGeometry
{
  double coxa = 0, femur = 0, tibia = 0;
};

struct JointAngles
{
  double coxa = 0, femur = 0, tibia = 0;
};

Vec3 forward(const LegGeometry & g, const JointAngles & q);

// Knee-up solution. Empty if the point is out of reach.
std::optional<JointAngles> inverse(const LegGeometry & g, const Vec3 & foot);

// Joint torques (N·m) that balance a force (N) acting on the foot, leg frame: tau = J^T f.
JointAngles joint_torques(const LegGeometry & g, const JointAngles & q, const Vec3 & force);

}  // namespace vector
