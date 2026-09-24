// Offline check of the gait against the servos, no ROS needed.
//
//   gait_report <legs.yaml> [--mass kg] [--period s] [--step m] [--stride m] [--com-x m] [--com-y m]
//
// Runs each gait for a few cycles and prints joint speeds, how close joints get to their
// limits, the static stability margin and the static joint torques from carrying the robot.
// Torques assume the weight is shared by the feet on the ground (least-squares split that
// keeps the robot balanced); dynamics and friction are ignored, so treat them as a floor.
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

#include "vector_gait/gait.hpp"

using vector::kLegs;

namespace
{

constexpr double kG = 9.81;
constexpr double kDt = 0.001;
constexpr double kMg996rStall = 11.0;  // kg·cm at 6 V, datasheet stall

double to_kgcm(double nm) {return nm / kG * 100.0;}
double deg(double rad) {return rad * 180.0 / M_PI;}

struct Limits
{
  double lo[3], hi[3], velocity;
};

struct Pt
{
  double x, y;
};

double cross(const Pt & o, const Pt & a, const Pt & b)
{
  return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
}

// Signed distance from c to the convex hull of pts (positive inside).
double stability_margin(std::vector<Pt> pts, const Pt & c)
{
  if (pts.size() < 3) {
    return -std::numeric_limits<double>::infinity();
  }
  std::sort(pts.begin(), pts.end(), [](const Pt & a, const Pt & b) {
      return a.x < b.x || (a.x == b.x && a.y < b.y);
    });
  std::vector<Pt> h(2 * pts.size());
  size_t k = 0;
  for (size_t i = 0; i < pts.size(); ++i) {
    while (k >= 2 && cross(h[k - 2], h[k - 1], pts[i]) <= 0) {k--;}
    h[k++] = pts[i];
  }
  for (size_t i = pts.size() - 1, t = k + 1; i-- > 0; ) {
    while (k >= t && cross(h[k - 2], h[k - 1], pts[i]) <= 0) {k--;}
    h[k++] = pts[i];
  }
  h.resize(k - 1);  // counter-clockwise, last point repeats the first

  double margin = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < h.size(); ++i) {
    const Pt & a = h[i];
    const Pt & b = h[(i + 1) % h.size()];
    const double len = std::hypot(b.x - a.x, b.y - a.y);
    margin = std::min(margin, cross(a, b, c) / len);
  }
  return margin;
}

// Vertical foot forces that carry weight w with the centre of mass at c:
// minimum-norm solution of sum(F) = w, sum(F x) = w cx, sum(F y) = w cy.
std::vector<double> split_weight(const std::vector<Pt> & feet, const Pt & c, double w)
{
  double m[3][3] = {};
  for (const auto & p : feet) {
    const double r[3] = {1, p.x, p.y};
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        m[i][j] += r[i] * r[j];
      }
    }
  }
  const double b[3] = {w, w * c.x, w * c.y};
  const double det =
    m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
    m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
    m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
  if (std::abs(det) < 1e-12) {
    return std::vector<double>(feet.size(), w / feet.size());
  }
  // lambda = m^-1 b (Cramer), F_i = [1 x_i y_i] . lambda
  double lam[3];
  for (int col = 0; col < 3; ++col) {
    double a[3][3];
    std::memcpy(a, m, sizeof(a));
    for (int r = 0; r < 3; ++r) {a[r][col] = b[r];}
    lam[col] = (a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1]) -
      a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0]) +
      a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0])) / det;
  }
  std::vector<double> f;
  for (const auto & p : feet) {
    f.push_back(lam[0] + lam[1] * p.x + lam[2] * p.y);
  }
  return f;
}

struct Scenario
{
  const char * name;
  vector::GaitType gait;
  vector::Twist2D cmd;
};

struct Result
{
  vector::Twist2D applied;
  double speed[3] = {0, 0, 0};
  double margin[3] = {1e9, 1e9, 1e9};
  double stability = 1e9;
  double torque[3] = {0, 0, 0};
  int ik_fail = 0;
};

Result run(
  const Scenario & sc, const vector::LegGeometry & geo, const std::array<vector::LegMount, kLegs> & mounts,
  vector::GaitParams params, const Limits & lim, double mass, const Pt & com)
{
  params.type = sc.gait;
  vector::Gait g(geo, mounts, params);
  Result r;

  const int warmup = static_cast<int>(params.period / kDt);
  const int steps = warmup + static_cast<int>(2 * params.period / kDt);
  auto prev = g.joints();

  for (int n = 0; n < std::max(steps, 1); ++n) {
    if (!g.update(sc.cmd, kDt)) {
      r.ik_fail++;
    }
    const auto & q = g.joints();
    if (n >= warmup || n == 0) {
      r.applied = g.applied();
      for (int l = 0; l < kLegs; ++l) {
        const double a[3] = {q[l].coxa, q[l].femur, q[l].tibia};
        const double b[3] = {prev[l].coxa, prev[l].femur, prev[l].tibia};
        for (int j = 0; j < 3; ++j) {
          if (n > 0) {
            r.speed[j] = std::max(r.speed[j], std::abs(a[j] - b[j]) / kDt);
          }
          r.margin[j] = std::min(r.margin[j], std::min(a[j] - lim.lo[j], lim.hi[j] - a[j]));
        }
      }

      std::vector<Pt> feet;
      std::vector<int> legs;
      for (int l = 0; l < kLegs; ++l) {
        if (!g.swinging(l)) {
          feet.push_back({g.feet()[l].x, g.feet()[l].y});
          legs.push_back(l);
        }
      }
      r.stability = std::min(r.stability, stability_margin(feet, com));
      const auto f = split_weight(feet, com, mass * kG);
      for (size_t i = 0; i < legs.size(); ++i) {
        const auto t = vector::joint_torques(geo, q[legs[i]], {0, 0, f[i]});
        r.torque[0] = std::max(r.torque[0], std::abs(t.coxa));
        r.torque[1] = std::max(r.torque[1], std::abs(t.femur));
        r.torque[2] = std::max(r.torque[2], std::abs(t.tibia));
      }
    }
    prev = q;
  }
  return r;
}

double arg(int argc, char ** argv, const char * name, double fallback)
{
  for (int i = 2; i + 1 < argc; ++i) {
    if (std::strcmp(argv[i], name) == 0) {
      return std::stod(argv[i + 1]);
    }
  }
  return fallback;
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 2) {
    std::fprintf(stderr,
      "usage: gait_report <legs.yaml> [--mass kg] [--period s] [--step m] [--stride m] [--com-x m] [--com-y m]\n");
    return 2;
  }

  const YAML::Node y = YAML::LoadFile(argv[1]);
  vector::LegGeometry geo{y["coxa"].as<double>(), y["femur"].as<double>(), y["tibia"].as<double>()};
  std::array<vector::LegMount, kLegs> mounts;
  for (int l = 0; l < kLegs; ++l) {
    const auto m = y["mounts"][vector::kLegNames[l]];
    mounts[l] = {m["x"].as<double>(), m["y"].as<double>(), m["yaw"].as<double>()};
  }
  Limits lim;
  const char * joints[3] = {"coxa", "femur", "tibia"};
  for (int j = 0; j < 3; ++j) {
    lim.lo[j] = y["limits"][joints[j]][0].as<double>();
    lim.hi[j] = y["limits"][joints[j]][1].as<double>();
  }
  lim.velocity = y["limits"]["velocity"].as<double>();

  vector::GaitParams p;
  p.reach = y["stand"]["reach"].as<double>();
  p.body_height = y["stand"]["height"].as<double>();
  p.period = arg(argc, argv, "--period", p.period);
  p.step_height = arg(argc, argv, "--step", p.step_height);
  p.max_stride = arg(argc, argv, "--stride", p.max_stride);
  const double mass = arg(argc, argv, "--mass", 2.5);
  const Pt com{arg(argc, argv, "--com-x", 0.0), arg(argc, argv, "--com-y", 0.0)};

  using vector::GaitType;
  const Scenario scenarios[] = {
    {"stand", GaitType::Tripod, {0, 0, 0}},
    {"tripod fwd", GaitType::Tripod, {1, 0, 0}},
    {"tripod side", GaitType::Tripod, {0, 1, 0}},
    {"tripod turn", GaitType::Tripod, {0, 0, 5}},
    {"tripod fwd+turn", GaitType::Tripod, {1, 0, 2}},
    {"ripple fwd", GaitType::Ripple, {1, 0, 0}},
    {"wave fwd", GaitType::Wave, {1, 0, 0}},
  };

  std::printf("mass %.2f kg, CoM (%.0f, %.0f) mm, period %.2f s, step %.0f mm, stride %.0f mm\n",
    mass, com.x * 1e3, com.y * 1e3, p.period, p.step_height * 1e3, p.max_stride * 1e3);
  std::printf("legs: coxa %.0f  femur %.0f  tibia %.0f mm, reach %.0f mm, hip height %.0f mm\n\n",
    geo.coxa * 1e3, geo.femur * 1e3, geo.tibia * 1e3, p.reach * 1e3, p.body_height * 1e3);
  std::printf("%-16s %-18s %-19s %-19s %-9s %-13s %s\n", "", "speed", "joint speed rad/s",
    "limit margin deg", "stability", "torque kg.cm", "");
  std::printf("%-16s %-18s %-19s %-19s %-9s %-13s %s\n", "scenario", "mm/s  mm/s  deg/s",
    "coxa femur tibia", "coxa femur tibia", "mm", "femur tibia", "notes");

  bool any_warning = false;
  for (const auto & sc : scenarios) {
    const Result r = run(sc, geo, mounts, p, lim, mass, com);
    std::string notes;
    const double fastest = std::max({r.speed[0], r.speed[1], r.speed[2]});
    if (fastest > lim.velocity) {notes += "too fast; ";}
    if (std::min({r.margin[0], r.margin[1], r.margin[2]}) < 0) {
      notes += "past joint limit; ";
    } else if (deg(std::min({r.margin[0], r.margin[1], r.margin[2]})) < 5) {
      notes += "near joint limit; ";
    }
    if (r.stability < 0.01) {notes += r.stability < 0 ? "tips over; " : "barely stable; ";}
    const double worst = to_kgcm(std::max(r.torque[1], r.torque[2]));
    if (worst > kMg996rStall) {
      notes += "over stall torque; ";
    } else if (worst > 0.5 * kMg996rStall) {
      notes += "over 50% of stall; ";
    }
    if (r.ik_fail) {notes += "out of reach " + std::to_string(r.ik_fail) + "x; ";}
    any_warning |= !notes.empty();

    std::printf("%-16s %5.0f %5.0f %5.0f   %5.1f %5.1f %5.1f   %5.1f %5.1f %5.1f   %7.1f   %5.1f %5.1f   %s\n",
      sc.name, r.applied.vx * 1e3, r.applied.vy * 1e3, deg(r.applied.wz),
      r.speed[0], r.speed[1], r.speed[2],
      deg(r.margin[0]), deg(r.margin[1]), deg(r.margin[2]),
      r.stability * 1e3, to_kgcm(r.torque[1]), to_kgcm(r.torque[2]), notes.c_str());
  }
  std::printf("\nlimits: joint speed %.1f rad/s, MG996R stall ~%.0f kg.cm at 6 V (aim for under half)\n",
    lim.velocity, kMg996rStall);
  return any_warning ? 1 : 0;
}
