#include "vector_gait/gait.hpp"

#include <algorithm>
#include <cmath>

namespace vector
{

namespace
{
constexpr double kNeutralTolerance = 0.003;  // m
constexpr double kIdle = 1e-4;

// Phase offsets per leg, in kLegNames order: L1 L2 L3 R1 R2 R3.
constexpr std::array<double, kLegs> kTripod = {0.0, 0.5, 0.0, 0.5, 0.0, 0.5};
constexpr std::array<double, kLegs> kRipple = {1.0 / 3, 2.0 / 3, 0.0, 5.0 / 6, 1.0 / 6, 0.5};
constexpr std::array<double, kLegs> kWave = {5.0 / 6, 4.0 / 6, 3.0 / 6, 2.0 / 6, 1.0 / 6, 0.0};

double frac(double x) {return x - std::floor(x);}

bool idle(const Twist2D & c)
{
  return std::abs(c.vx) < kIdle && std::abs(c.vy) < kIdle && std::abs(c.wz) < kIdle;
}

double step_toward(double from, double to, double max_step)
{
  return from + std::clamp(to - from, -max_step, max_step);
}

// Body velocity at a point p (walking frame): v + w x p.
void point_velocity(const Twist2D & c, const Vec3 & p, double & vx, double & vy)
{
  vx = c.vx - c.wz * p.y;
  vy = c.vy + c.wz * p.x;
}
}  // namespace

bool parse_gait(const std::string & name, GaitType & out)
{
  if (name == "tripod") {out = GaitType::Tripod; return true;}
  if (name == "ripple") {out = GaitType::Ripple; return true;}
  if (name == "wave") {out = GaitType::Wave; return true;}
  return false;
}

Gait::Gait(const LegGeometry & geometry, const std::array<LegMount, kLegs> & mounts, const GaitParams & params)
: geometry_(geometry), mounts_(mounts), params_(params)
{
  for (int i = 0; i < kLegs; ++i) {
    const auto & m = mounts_[i];
    neutral_[i] = {
      m.x + params_.reach * std::cos(m.yaw),
      m.y + params_.reach * std::sin(m.yaw),
      -params_.body_height};
    feet_[i] = neutral_[i];
    solve(i);
  }
  set_type(params_.type);
}

void Gait::set_type(GaitType type)
{
  params_.type = type;
  switch (type) {
    case GaitType::Tripod: offset_ = kTripod; break;
    case GaitType::Ripple: offset_ = kRipple; break;
    case GaitType::Wave: offset_ = kWave; break;
  }
}

double Gait::duty() const
{
  switch (params_.type) {
    case GaitType::Ripple: return 2.0 / 3;
    case GaitType::Wave: return 5.0 / 6;
    default: return 0.5;
  }
}

Twist2D Gait::limit(const Twist2D & cmd) const
{
  // Largest stride any leg would take for this command.
  const double stance_time = params_.period * duty();
  double worst = 0;
  for (const auto & n : neutral_) {
    double vx, vy;
    point_velocity(cmd, n, vx, vy);
    worst = std::max(worst, std::hypot(vx, vy) * stance_time);
  }
  if (worst <= params_.max_stride) {
    return cmd;
  }
  const double k = params_.max_stride / worst;
  return {cmd.vx * k, cmd.vy * k, cmd.wz * k};
}

Vec3 Gait::to_body(const Vec3 & p) const
{
  // p_body = R^T (p - t), R = Rz(yaw) Ry(pitch) Rx(roll)
  const double dx = p.x - pose_.x, dy = p.y - pose_.y, dz = p.z - pose_.z;
  const double cy = std::cos(pose_.yaw), sy = std::sin(pose_.yaw);
  const double cp = std::cos(pose_.pitch), sp = std::sin(pose_.pitch);
  const double cr = std::cos(pose_.roll), sr = std::sin(pose_.roll);
  // undo yaw
  const double x1 = cy * dx + sy * dy, y1 = -sy * dx + cy * dy, z1 = dz;
  // undo pitch
  const double x2 = cp * x1 - sp * z1, y2 = y1, z2 = sp * x1 + cp * z1;
  // undo roll
  return {x2, cr * y2 + sr * z2, -sr * y2 + cr * z2};
}

bool Gait::ramp_pose(double dt)
{
  const double lin = params_.pose_speed * dt, ang = params_.pose_turn * dt;
  const BodyPose before = pose_;
  BodyPose & p = pose_;
  const BodyPose & t = pose_target_;
  p.x = step_toward(p.x, t.x, lin);
  p.y = step_toward(p.y, t.y, lin);
  p.z = step_toward(p.z, t.z, lin);
  p.roll = step_toward(p.roll, t.roll, ang);
  p.pitch = step_toward(p.pitch, t.pitch, ang);
  p.yaw = step_toward(p.yaw, t.yaw, ang);
  return p.x != before.x || p.y != before.y || p.z != before.z ||
         p.roll != before.roll || p.pitch != before.pitch || p.yaw != before.yaw;
}

bool Gait::solve(int leg)
{
  if (auto q = inverse(geometry_, to_leg(leg, to_body(feet_[leg])))) {
    joints_[leg] = *q;
    return true;
  }
  return false;
}

Vec3 Gait::to_leg(int leg, const Vec3 & p) const
{
  const auto & m = mounts_[leg];
  const double dx = p.x - m.x, dy = p.y - m.y;
  const double c = std::cos(m.yaw), s = std::sin(m.yaw);
  return {c * dx + s * dy, -s * dx + c * dy, p.z};
}

bool Gait::update(const Twist2D & cmd_in, double dt)
{
  const bool is_idle = idle(cmd_in);
  applied_ = is_idle ? Twist2D{} : limit(cmd_in);

  const bool pose_moving = ramp_pose(dt);

  if (is_idle && standing_) {
    // Standing still: feet stay put, only the body pose may change.
    bool ok = true;
    if (pose_moving) {
      for (int i = 0; i < kLegs; ++i) {
        ok = solve(i) && ok;
      }
    }
    return ok;
  }
  standing_ = false;

  const double d = duty();
  const double stance_time = params_.period * d;
  phase_ = frac(phase_ + dt / params_.period);

  bool ok = true;
  bool settled = is_idle;

  for (int i = 0; i < kLegs; ++i) {
    const double p = frac(phase_ + offset_[i]);
    Vec3 & f = feet_[i];
    const Vec3 & n = neutral_[i];
    const bool at_neutral = std::hypot(f.x - n.x, f.y - n.y) < kNeutralTolerance;
    const bool swing_window = p >= d;

    if (swing_window && !swinging_[i] && !skip_[i]) {
      // Start of a swing. If there's nowhere to go, keep the foot down.
      if (is_idle && at_neutral) {
        skip_[i] = true;
      } else {
        swinging_[i] = true;
        liftoff_[i] = f;
      }
    }
    if (!swing_window) {
      swinging_[i] = false;
      skip_[i] = false;
    }

    if (swinging_[i]) {
      const double s = (p - d) / (1 - d);
      const double e = s * s * (3 - 2 * s);
      double vx, vy;
      point_velocity(applied_, n, vx, vy);
      const double tx = n.x + 0.5 * vx * stance_time;
      const double ty = n.y + 0.5 * vy * stance_time;
      f.x = liftoff_[i].x + (tx - liftoff_[i].x) * e;
      f.y = liftoff_[i].y + (ty - liftoff_[i].y) * e;
      f.z = -params_.body_height + params_.step_height * std::sin(M_PI * s);
      settled = false;
    } else {
      double vx, vy;
      point_velocity(applied_, f, vx, vy);
      f.x -= vx * dt;
      f.y -= vy * dt;
      f.z = -params_.body_height;
      if (!at_neutral) {
        settled = false;
      }
    }

    ok = solve(i) && ok;
  }

  standing_ = settled;
  return ok;
}

}  // namespace vector
