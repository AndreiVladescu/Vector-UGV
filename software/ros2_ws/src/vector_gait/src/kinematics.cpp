#include "vector_gait/kinematics.hpp"

#include <algorithm>
#include <cmath>

namespace vector
{

namespace
{
// acos that tolerates rounding at the edge of the workspace
double safe_acos(double c) {return std::acos(std::clamp(c, -1.0, 1.0));}
}  // namespace

Vec3 forward(const LegGeometry & g, const JointAngles & q)
{
  const double r = g.coxa + g.femur * std::cos(q.femur) + g.tibia * std::cos(q.femur + q.tibia);
  const double z = g.femur * std::sin(q.femur) + g.tibia * std::sin(q.femur + q.tibia);
  return {r * std::cos(q.coxa), r * std::sin(q.coxa), z};
}

std::optional<JointAngles> inverse(const LegGeometry & g, const Vec3 & foot)
{
  const double r = std::hypot(foot.x, foot.y) - g.coxa;
  const double d = std::hypot(r, foot.z);
  if (d > g.femur + g.tibia || d < std::abs(g.femur - g.tibia) || d < 1e-9) {
    return std::nullopt;
  }

  const double knee = safe_acos((g.femur * g.femur + g.tibia * g.tibia - d * d) / (2 * g.femur * g.tibia));
  const double lift = safe_acos((g.femur * g.femur + d * d - g.tibia * g.tibia) / (2 * g.femur * d));

  JointAngles q;
  q.coxa = std::atan2(foot.y, foot.x);
  q.femur = std::atan2(foot.z, r) + lift;
  q.tibia = knee - M_PI;
  return q;
}

JointAngles joint_torques(const LegGeometry & g, const JointAngles & q, const Vec3 & f)
{
  const double c = std::cos(q.coxa), s = std::sin(q.coxa);
  const double r = g.coxa + g.femur * std::cos(q.femur) + g.tibia * std::cos(q.femur + q.tibia);
  const double dr_df = -g.femur * std::sin(q.femur) - g.tibia * std::sin(q.femur + q.tibia);
  const double dz_df = g.femur * std::cos(q.femur) + g.tibia * std::cos(q.femur + q.tibia);
  const double dr_dt = -g.tibia * std::sin(q.femur + q.tibia);
  const double dz_dt = g.tibia * std::cos(q.femur + q.tibia);

  JointAngles t;
  t.coxa = -r * s * f.x + r * c * f.y;
  t.femur = dr_df * (c * f.x + s * f.y) + dz_df * f.z;
  t.tibia = dr_dt * (c * f.x + s * f.y) + dz_dt * f.z;
  return t;
}

}  // namespace vector
