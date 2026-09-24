// Keeps the body level using the IMU. Plain C++, no ROS.
//
// The IMU sees ground tilt + commanded body tilt. Integrating the measured tilt back
// into the body pose drives the measurement to zero, so the body ends up level while
// the feet follow the ground.
#pragma once

namespace vector
{

class Leveler
{
public:
  Leveler(double gain, double max_angle)
  : gain_(gain), max_(max_angle) {}

  // Measured body roll/pitch relative to gravity (rad).
  void update(double roll, double pitch, double dt);
  void reset() {roll_ = pitch_ = 0;}

  // Body pose correction to add to the commanded pose.
  double roll() const {return roll_;}
  double pitch() const {return pitch_;}

private:
  double gain_, max_;
  double roll_ = 0, pitch_ = 0;
};

}  // namespace vector
