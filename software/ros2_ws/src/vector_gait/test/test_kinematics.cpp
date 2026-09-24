#include <gtest/gtest.h>

#include <cmath>

#include "vector_gait/kinematics.hpp"

using vector::forward;
using vector::inverse;

namespace
{
const vector::LegGeometry kGeo{0.05, 0.08, 0.12};
}

TEST(Kinematics, RoundTripOverWorkspace)
{
  int solved = 0;
  for (double x = 0.06; x <= 0.24; x += 0.01) {
    for (double y = -0.12; y <= 0.12; y += 0.02) {
      for (double z = -0.16; z <= 0.04; z += 0.01) {
        const vector::Vec3 p{x, y, z};
        auto q = inverse(kGeo, p);
        if (!q) {
          continue;
        }
        const auto back = forward(kGeo, *q);
        EXPECT_NEAR(back.x, p.x, 1e-6);
        EXPECT_NEAR(back.y, p.y, 1e-6);
        EXPECT_NEAR(back.z, p.z, 1e-6);
        ++solved;
      }
    }
  }
  EXPECT_GT(solved, 1000);
}

TEST(Kinematics, StandingPose)
{
  // Foot 13 cm out from the hip, 10 cm below it.
  auto q = inverse(kGeo, {0.13, 0.0, -0.10});
  ASSERT_TRUE(q);
  EXPECT_NEAR(q->coxa, 0.0, 1e-6);
  EXPECT_NEAR(q->femur, 0.2523, 1e-3);
  EXPECT_NEAR(q->tibia, -1.8020, 1e-3);
}

TEST(Kinematics, OutOfReach)
{
  EXPECT_FALSE(inverse(kGeo, {0.40, 0.0, 0.0}));
  EXPECT_FALSE(inverse(kGeo, {0.05, 0.0, 0.0}));
}

TEST(Kinematics, TorqueFromVerticalLoad)
{
  // For a vertical foot force the femur torque is force x horizontal distance femur->foot,
  // and the tibia torque is force x horizontal distance knee->foot.
  const vector::Vec3 foot{0.13, 0.0, -0.10};
  auto q = inverse(kGeo, foot);
  ASSERT_TRUE(q);
  const double fz = 10.0;
  const auto t = vector::joint_torques(kGeo, *q, {0, 0, fz});
  const double knee_x = kGeo.coxa + kGeo.femur * std::cos(q->femur);
  EXPECT_NEAR(t.coxa, 0.0, 1e-12);
  EXPECT_NEAR(t.femur, fz * (foot.x - kGeo.coxa), 1e-9);
  EXPECT_NEAR(t.tibia, fz * (foot.x - knee_x), 1e-9);
}
