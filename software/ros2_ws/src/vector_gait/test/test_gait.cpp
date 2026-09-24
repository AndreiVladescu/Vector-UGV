#include <gtest/gtest.h>

#include <cmath>

#include "vector_gait/gait.hpp"

namespace
{
vector::Gait make(vector::GaitType type)
{
  const vector::LegGeometry geo{0.05, 0.08, 0.12};
  const std::array<vector::LegMount, vector::kLegs> mounts = {{
    {0.10, 0.07, 0.7854}, {0.0, 0.09, 1.5708}, {-0.10, 0.07, 2.3562},
    {0.10, -0.07, -0.7854}, {0.0, -0.09, -1.5708}, {-0.10, -0.07, -2.3562}}};
  vector::GaitParams p;
  p.type = type;
  return vector::Gait(geo, mounts, p);
}

constexpr double kDt = 0.005;

int feet_down(const vector::Gait & g)
{
  int n = 0;
  for (const auto & f : g.feet()) {
    n += std::abs(f.z + g.params().body_height) < 1e-3;
  }
  return n;
}
}  // namespace

TEST(Gait, StartsStanding)
{
  auto g = make(vector::GaitType::Tripod);
  EXPECT_TRUE(g.standing());
  EXPECT_TRUE(g.update({}, kDt));
  EXPECT_TRUE(g.standing());
}

TEST(Gait, TripodKeepsThreeFeetDown)
{
  auto g = make(vector::GaitType::Tripod);
  for (int i = 0; i < 1000; ++i) {
    ASSERT_TRUE(g.update({0.08, 0.0, 0.0}, kDt));
    int swinging = 0;
    for (int l = 0; l < vector::kLegs; ++l) {
      swinging += g.swinging(l);
    }
    EXPECT_LE(swinging, 3);
    EXPECT_GE(feet_down(g), 3);
  }
}

TEST(Gait, StanceFeetStayPutInTheWorld)
{
  // Walk and turn at once; a planted foot must not slide in the world frame.
  auto g = make(vector::GaitType::Ripple);
  double x = 0, y = 0, yaw = 0;
  std::array<bool, vector::kLegs> was_stance{};
  std::array<vector::Vec3, vector::kLegs> world{};

  for (int i = 0; i < 2000; ++i) {
    ASSERT_TRUE(g.update({0.05, 0.02, 0.2}, kDt));
    const auto & v = g.applied();
    x += (v.vx * std::cos(yaw) - v.vy * std::sin(yaw)) * kDt;
    y += (v.vx * std::sin(yaw) + v.vy * std::cos(yaw)) * kDt;
    yaw += v.wz * kDt;

    for (int l = 0; l < vector::kLegs; ++l) {
      const auto & f = g.feet()[l];
      const vector::Vec3 w{
        x + f.x * std::cos(yaw) - f.y * std::sin(yaw),
        y + f.x * std::sin(yaw) + f.y * std::cos(yaw), 0};
      const bool stance = !g.swinging(l);
      if (stance && was_stance[l]) {
        EXPECT_NEAR(w.x, world[l].x, 5e-5);
        EXPECT_NEAR(w.y, world[l].y, 5e-5);
      }
      world[l] = w;
      was_stance[l] = stance;
    }
  }
}

TEST(Gait, SettlesBackToStanding)
{
  for (auto type : {vector::GaitType::Tripod, vector::GaitType::Ripple, vector::GaitType::Wave}) {
    auto g = make(type);
    for (int i = 0; i < 600; ++i) {
      g.update({0.06, 0.0, 0.1}, kDt);
    }
    EXPECT_FALSE(g.standing());
    for (int i = 0; i < 1000 && !g.standing(); ++i) {
      g.update({}, kDt);
    }
    EXPECT_TRUE(g.standing());
    for (int l = 0; l < vector::kLegs; ++l) {
      EXPECT_NEAR(g.feet()[l].x, g.neutral()[l].x, 0.003);
      EXPECT_NEAR(g.feet()[l].y, g.neutral()[l].y, 0.003);
    }
    EXPECT_EQ(feet_down(g), vector::kLegs);
  }
}

TEST(Gait, StrideIsLimited)
{
  auto g = make(vector::GaitType::Tripod);
  g.update({5.0, 0.0, 0.0}, kDt);
  const double stride = g.applied().vx * g.params().period * g.duty();
  EXPECT_NEAR(stride, g.params().max_stride, 1e-6);
}

TEST(Gait, BodyPoseMovesBodyNotFeet)
{
  auto g = make(vector::GaitType::Tripod);
  const auto feet_before = g.feet();
  vector::BodyPose target;
  target.z = 0.02;
  target.roll = 0.1;
  target.pitch = -0.1;
  target.yaw = 0.15;
  g.set_body_pose(target);

  for (int i = 0; i < 400; ++i) {
    ASSERT_TRUE(g.update({}, kDt));
  }
  EXPECT_NEAR(g.body_pose().z, 0.02, 1e-9);
  EXPECT_NEAR(g.body_pose().roll, 0.1, 1e-9);
  EXPECT_TRUE(g.standing());

  const vector::LegGeometry geo{0.05, 0.08, 0.12};
  for (int l = 0; l < vector::kLegs; ++l) {
    // Feet did not move on the ground...
    EXPECT_NEAR(g.feet()[l].x, feet_before[l].x, 1e-12);
    EXPECT_NEAR(g.feet()[l].z, feet_before[l].z, 1e-12);
    // ...and the joint angles put each foot exactly there, seen from the tilted body.
    const auto p = vector::forward(geo, g.joints()[l]);
    const auto want = g.to_body(g.feet()[l]);
    const std::array<double, vector::kLegs> yaw = {0.7854, 1.5708, 2.3562, -0.7854, -1.5708, -2.3562};
    const std::array<double, vector::kLegs> mx = {0.10, 0.0, -0.10, 0.10, 0.0, -0.10};
    const std::array<double, vector::kLegs> my = {0.07, 0.09, 0.07, -0.07, -0.09, -0.07};
    const double bx = mx[l] + std::cos(yaw[l]) * p.x - std::sin(yaw[l]) * p.y;
    const double by = my[l] + std::sin(yaw[l]) * p.x + std::cos(yaw[l]) * p.y;
    EXPECT_NEAR(bx, want.x, 1e-9);
    EXPECT_NEAR(by, want.y, 1e-9);
    EXPECT_NEAR(p.z, want.z, 1e-9);
  }
}
