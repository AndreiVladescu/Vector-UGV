#include <gtest/gtest.h>

#include <cmath>

#include "vector_gait/gait.hpp"

namespace
{
vector::Gait make(vector::GaitType type, bool touchdown = false)
{
  const vector::LegGeometry geo{0.05, 0.08, 0.12};
  const std::array<vector::LegMount, vector::kLegs> mounts = {{
    {0.10, 0.07, 0.7854}, {0.0, 0.09, 1.5708}, {-0.10, 0.07, 2.3562},
    {0.10, -0.07, -0.7854}, {0.0, -0.09, -1.5708}, {-0.10, -0.07, -2.3562}}};
  vector::GaitParams p;
  p.type = type;
  p.touchdown = touchdown;
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
  for (int i = 0; i < 400; ++i) {  // past the acceleration ramp
    g.update({5.0, 0.0, 0.0}, kDt);
  }
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

TEST(Gait, RestartAfterStopDoesNotOverreach)
{
  // Walk, stop until standing, then set off in each direction. The first steps must not
  // push any foot further from neutral than a normal stride, and must not jump in height.
  const vector::Twist2D starts[] = {{0.12, 0, 0}, {0, 0.12, 0}, {0, 0, 0.6}, {-0.12, 0, 0}};
  for (auto type : {vector::GaitType::Tripod, vector::GaitType::Ripple, vector::GaitType::Wave}) {
    for (const auto & go : starts) {
      auto g = make(type);
      for (int i = 0; i < 700; ++i) {
        g.update({0.08, 0.0, 0.0}, kDt);
      }
      for (int i = 0; i < 2000 && !g.standing(); ++i) {
        g.update({}, kDt);
      }
      ASSERT_TRUE(g.standing());

      // a normal swing's fastest height change per tick, for this gait
      const double swing_time = g.params().period * (1 - g.duty());
      const double normal = g.params().step_height * M_PI * kDt / swing_time;
      double worst = 0, jump = 0;
      auto prev = g.feet();
      for (int i = 0; i < 600; ++i) {
        EXPECT_TRUE(g.update(go, kDt));
        for (int l = 0; l < vector::kLegs; ++l) {
          const auto & f = g.feet()[l];
          const auto & n = g.neutral()[l];
          worst = std::max(worst, std::hypot(f.x - n.x, f.y - n.y));
          jump = std::max(jump, std::abs(f.z - prev[l].z));
        }
        prev = g.feet();
      }
      EXPECT_LE(worst, g.params().max_stride + 0.002)
        << "gait " << static_cast<int>(type) << " cmd " << go.vx << "," << go.vy << "," << go.wz;
      EXPECT_LT(jump, 2.0 * normal) << "foot height jumped";
    }
  }
}

TEST(Gait, SpeedRampsUp)
{
  auto g = make(vector::GaitType::Tripod);
  g.update({0.12, 0.0, 0.0}, kDt);
  EXPECT_NEAR(g.applied().vx, 0.25 * kDt, 1e-9);
  for (int i = 0; i < 200; ++i) {
    g.update({0.12, 0.0, 0.0}, kDt);
  }
  EXPECT_NEAR(g.applied().vx, 0.12, 1e-9);
}

TEST(Gait, TouchdownStopsOnObstacleAndProbesHoles)
{
  // Leg 0 walks onto a 20 mm block, leg 1 steps into a hole, the rest are on flat ground.
  auto g = make(vector::GaitType::Tripod, true);
  const double h = -g.params().body_height, block = h + 0.020;
  for (int i = 0; i < 1200; ++i) {
    std::array<bool, vector::kLegs> contact{};
    for (int l = 0; l < vector::kLegs; ++l) {
      const double ground = l == 0 ? block : l == 1 ? -1.0 : h;
      contact[l] = g.feet()[l].z <= ground + 1e-4;
    }
    g.set_contact(contact);
    ASSERT_TRUE(g.update({0.06, 0.0, 0.0}, kDt));
  }
  // contact is seen one tick after the foot gets there, so a few mm of overshoot
  EXPECT_NEAR(g.ground_z(0), block - 0.0025, 0.0025);
  EXPECT_NEAR(g.ground_z(1), h - g.params().probe_depth, 0.003);
  EXPECT_NEAR(g.ground_z(2), h - 0.0025, 0.0025);
  EXPECT_GT(g.ground_z(0) - g.ground_z(2), 0.015);  // the block leg really stands higher
  // stance feet sit on the ground they found
  for (int l : {0, 1, 2}) {
    if (!g.swinging(l)) {
      EXPECT_NEAR(g.feet()[l].z, g.ground_z(l), 1e-9);
    }
  }
}
