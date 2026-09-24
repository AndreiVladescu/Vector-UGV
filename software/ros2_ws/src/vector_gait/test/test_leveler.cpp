#include <gtest/gtest.h>

#include "vector_gait/leveler.hpp"

TEST(Leveler, CancelsGroundTilt)
{
  vector::Leveler lev(2.0, 0.3);
  const double ground_roll = 0.08, ground_pitch = -0.12;
  for (int i = 0; i < 2000; ++i) {
    // what the IMU would read: ground tilt plus the correction already applied
    lev.update(ground_roll + lev.roll(), ground_pitch + lev.pitch(), 0.005);
  }
  EXPECT_NEAR(lev.roll(), -ground_roll, 1e-4);
  EXPECT_NEAR(lev.pitch(), -ground_pitch, 1e-4);
}

TEST(Leveler, StaysWithinLimit)
{
  vector::Leveler lev(2.0, 0.3);
  for (int i = 0; i < 5000; ++i) {
    lev.update(0.6 + lev.roll(), 0.0, 0.005);
  }
  EXPECT_DOUBLE_EQ(lev.roll(), -0.3);
}
