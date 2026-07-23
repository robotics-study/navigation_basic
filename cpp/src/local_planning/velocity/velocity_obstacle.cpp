#include "navigation/local_planning/velocity/velocity_obstacle.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "navigation/local_planning/geometry.hpp"

namespace navigation::local_planning {

namespace {

double dist_pts(const core::Point& a, const core::Point& b) {
  return std::hypot(a.x - b.x, a.y - b.y);
}

double det2(const core::Point& a, const core::Point& b) { return a.x * b.y - a.y * b.x; }

double dot2(const core::Point& a, const core::Point& b) { return a.x * b.x + a.y * b.y; }

core::Point sub2(const core::Point& a, const core::Point& b) {
  return core::Point{a.x - b.x, a.y - b.y};
}

core::Point unit2(const core::Point& v) {
  double n = std::hypot(v.x, v.y);
  if (n < kEps) return core::Point{0.0, 0.0};
  return core::Point{v.x / n, v.y / n};
}

// direction (RVO2's Line.direction) is normal rotated -90 degrees, so that
// feasibility det(direction, v - point) >= 0 matches dot(v - point, normal) >= 0.
core::Point direction_of(const core::Point& normal) { return core::Point{normal.y, -normal.x}; }

core::Point normal_of(const core::Point& direction) {
  return core::Point{-direction.y, direction.x};
}

struct Lp1Result {
  bool ok = false;
  core::Point point;
};

// RVO2 linearProgram1, ported to the (point, normal) representation: the 1D
// sub-problem of optimizing along line `line_no` subject to the max-speed
// circle and every earlier line. Returns ok=false on an infeasible (empty)
// interval -- never raises.
Lp1Result lp1(const std::vector<HalfPlane>& half_planes, int line_no,
             const core::Point& opt_velocity, double max_speed, bool direction_opt) {
  const auto& [point, normal] = half_planes[static_cast<size_t>(line_no)];
  core::Point direction = direction_of(normal);
  double dot_product = dot2(point, direction);
  double discriminant = dot_product * dot_product + max_speed * max_speed - dot2(point, point);
  if (discriminant < 0.0) return {false, point};
  double sqrt_discriminant = std::sqrt(discriminant);
  double t_left = -dot_product - sqrt_discriminant;
  double t_right = -dot_product + sqrt_discriminant;
  for (int i = 0; i < line_no; ++i) {
    const auto& [p_i, n_i] = half_planes[static_cast<size_t>(i)];
    core::Point d_i = direction_of(n_i);
    double denominator = det2(direction, d_i);
    double numerator = det2(d_i, sub2(point, p_i));
    if (std::fabs(denominator) <= kEps) {
      if (numerator < 0.0) return {false, point};
      continue;
    }
    double t = numerator / denominator;
    if (denominator >= 0.0) {
      t_right = std::min(t_right, t);
    } else {
      t_left = std::max(t_left, t);
    }
    if (t_left > t_right) return {false, point};
  }
  double t;
  if (direction_opt) {
    t = dot2(opt_velocity, direction) > 0.0 ? t_right : t_left;
  } else {
    t = std::max(t_left, std::min(t_right, dot2(direction, sub2(opt_velocity, point))));
  }
  return {true, core::Point{point.x + t * direction.x, point.y + t * direction.y}};
}

struct Lp2Result {
  core::Point velocity;
  int fail_index = 0;
};

// RVO2 linearProgram2: incrementally re-optimize onto each violated line's 1D
// sub-problem. fail_index == half_planes.size() means every line was
// satisfied.
Lp2Result lp2(const std::vector<HalfPlane>& half_planes, const core::Point& opt_velocity,
             double max_speed, bool direction_opt) {
  core::Point result;
  if (direction_opt) {
    result = core::Point{opt_velocity.x * max_speed, opt_velocity.y * max_speed};
  } else if (std::hypot(opt_velocity.x, opt_velocity.y) > max_speed) {
    core::Point u = unit2(opt_velocity);
    result = core::Point{u.x * max_speed, u.y * max_speed};
  } else {
    result = opt_velocity;
  }
  for (int i = 0; i < static_cast<int>(half_planes.size()); ++i) {
    const auto& [point, normal] = half_planes[static_cast<size_t>(i)];
    if (dot2(sub2(result, point), normal) < 0.0) {
      core::Point saved = result;
      Lp1Result r = lp1(half_planes, i, opt_velocity, max_speed, direction_opt);
      if (!r.ok) return {saved, i};
      result = r.point;
    }
  }
  return {result, static_cast<int>(half_planes.size())};
}

}  // namespace

core::Point preferred_velocity(const core::Pose& pose, const core::Pose& goal, double max_speed) {
  double dx = goal.x - pose.x, dy = goal.y - pose.y;
  double dist = std::hypot(dx, dy);
  if (dist < kEps) return core::Point{0.0, 0.0};
  double speed = std::min(max_speed, dist);
  return core::Point{dx / dist * speed, dy / dist * speed};
}

core::VelocityCommand velocity_to_command(const core::Point& v_new, double theta, double max_omega,
                                          double heading_gain) {
  double speed = std::hypot(v_new.x, v_new.y);
  if (speed < kEps) return core::VelocityCommand{0.0, 0.0};
  double desired = std::atan2(v_new.y, v_new.x);
  double theta_err = wrap_to_pi(desired - theta);
  return heading_command(theta_err, heading_gain, speed, max_omega);
}

Cone truncated_vo_cone(const core::Point& rel_pos, double combined_radius,
                       const core::Point& apex_vel, double tau) {
  double px = rel_pos.x, py = rel_pos.y;
  double dist = std::hypot(px, py);
  if (dist <= combined_radius + kEps) {
    // Already overlapping: every relative velocity leads to (deeper)
    // penetration, so the forbidden region is the whole plane.
    return Cone{apex_vel,        core::Point{1.0, 0.0}, -1.0, dist, combined_radius,
               tau, true, core::Point{1.0, 0.0}, core::Point{1.0, 0.0}};
  }
  double ux = px / dist, uy = py / dist;
  double sin_half = combined_radius / dist;
  double cos_half = std::sqrt(std::max(0.0, 1.0 - sin_half * sin_half));
  core::Point left{ux * cos_half - uy * sin_half, ux * sin_half + uy * cos_half};
  core::Point right{ux * cos_half + uy * sin_half, -ux * sin_half + uy * cos_half};
  return Cone{apex_vel, core::Point{ux, uy}, cos_half, dist, combined_radius, tau, false, left,
             right};
}

bool in_velocity_obstacle(const core::Point& v, const Cone& cone) {
  if (cone.full) return true;
  double wx = v.x - cone.apex.x, wy = v.y - cone.apex.y;
  double wlen = std::hypot(wx, wy);
  if (wlen < kEps) return false;  // relative rest never collides
  double wproj = wx * cone.axis.x + wy * cone.axis.y;
  if (wproj <= 0.0) return false;  // moving away from the obstacle
  double cos_ang = wproj / wlen;
  if (cos_ang < cone.cos_half) return false;  // outside the cone's angular span
  // tau-truncation (near-plane approximation): the gap must close within tau.
  if (wproj < (cone.dist - cone.radius) / cone.tau) return false;
  return true;
}

std::vector<double> cone_to_constraint(const Cone& cone) {
  return {cone.apex.x, cone.apex.y, cone.left.x, cone.left.y, cone.right.x, cone.right.y};
}

std::vector<double> halfplane_to_constraint(const HalfPlane& plane) {
  const auto& [point, normal] = plane;
  return {point.x, point.y, normal.x, normal.y};
}

core::Point rvo_apex(const core::Point& v_self, const core::Point& v_other, double reciprocity) {
  return core::Point{(1.0 - reciprocity) * v_other.x + reciprocity * v_self.x,
                     (1.0 - reciprocity) * v_other.y + reciprocity * v_self.y};
}

std::vector<core::Point> sample_reachable_velocities(const core::Point& v_pref, double max_speed,
                                                      int speed_samples, int angle_samples) {
  double speed = std::hypot(v_pref.x, v_pref.y);
  core::Point v0 = speed > max_speed
                       ? core::Point{v_pref.x / speed * max_speed, v_pref.y / speed * max_speed}
                       : v_pref;
  std::vector<core::Point> out;
  out.reserve(static_cast<size_t>(1 + (speed_samples + 1) * angle_samples));
  out.push_back(v0);
  for (int si = 0; si <= speed_samples; ++si) {
    double s = max_speed * static_cast<double>(si) / static_cast<double>(speed_samples);
    for (int ai = 0; ai < angle_samples; ++ai) {
      double ang = 2.0 * M_PI * static_cast<double>(ai) / static_cast<double>(angle_samples);
      out.push_back(core::Point{s * std::cos(ang), s * std::sin(ang)});
    }
  }
  return out;
}

std::vector<DynamicObstacle> static_obstacles(core::ObstacleQuery& space, const core::Point& center,
                                              double sensor_radius, double obstacle_radius) {
  std::vector<DynamicObstacle> out;
  for (const core::Point& p : space.occupied_within(center, sensor_radius)) {
    out.push_back(DynamicObstacle{p, core::Point{0.0, 0.0}, obstacle_radius});
  }
  return out;
}

VelocitySelection select_sampled_velocity(
    const core::Point& v_pref, const std::vector<DynamicObstacle>& obstacles, const core::Point& pos,
    double agent_radius, double neighbor_dist, double time_horizon, double max_speed,
    int speed_samples, int angle_samples,
    const std::function<core::Point(const DynamicObstacle&)>& apex_of) {
  std::vector<Cone> cones;
  for (const DynamicObstacle& o : obstacles) {
    if (dist_pts(o.position, pos) < neighbor_dist + o.radius) {
      core::Point rel_pos{o.position.x - pos.x, o.position.y - pos.y};
      cones.push_back(
          truncated_vo_cone(rel_pos, agent_radius + o.radius, apex_of(o), time_horizon));
    }
  }
  std::vector<core::Point> candidates =
      sample_reachable_velocities(v_pref, max_speed, speed_samples, angle_samples);
  core::Point best = candidates[0];
  double best_cost = std::numeric_limits<double>::infinity();
  for (const core::Point& v : candidates) {  // fixed traversal order -- see sample_reachable_velocities
    bool violated = false;
    for (const Cone& c : cones) {
      if (in_velocity_obstacle(v, c)) {
        violated = true;
        break;
      }
    }
    double cost = dist_pts(v, v_pref) + (violated ? kPenalty : 0.0);
    if (cost < best_cost) {  // strict <: first candidate (v_pref) wins ties
      best_cost = cost;
      best = v;
    }
  }
  VelocitySelection result;
  result.velocity = best;
  result.constraints.reserve(cones.size());
  for (const Cone& c : cones) result.constraints.push_back(cone_to_constraint(c));
  return result;
}

HalfPlane orca_half_plane(const core::Point& rel_pos, const core::Point& rel_vel,
                         const core::Point& v_self, double combined_radius, double tau, double dt) {
  double px = rel_pos.x, py = rel_pos.y;
  double vx = rel_vel.x, vy = rel_vel.y;
  double dist_sq = px * px + py * py;
  double r = combined_radius;
  double r_sq = r * r;
  core::Point normal;
  core::Point u;
  if (dist_sq > r_sq) {  // not currently colliding
    double inv_tau = 1.0 / tau;
    double wx = vx - inv_tau * px, wy = vy - inv_tau * py;
    double w_len_sq = wx * wx + wy * wy;
    double dot1 = wx * px + wy * py;
    if (dot1 < 0.0 && dot1 * dot1 > r_sq * w_len_sq) {
      // In front of the truncated cutoff circle: project onto it.
      double w_len = std::sqrt(w_len_sq);
      double unit_wx = wx / w_len, unit_wy = wy / w_len;
      normal = core::Point{unit_wx, unit_wy};
      u = core::Point{(r * inv_tau - w_len) * unit_wx, (r * inv_tau - w_len) * unit_wy};
    } else {
      // Project onto one of the two tangent legs.
      double leg = std::sqrt(dist_sq - r_sq);
      double dirx, diry;
      if (px * wy - py * wx > 0.0) {
        dirx = (px * leg - py * r) / dist_sq;
        diry = (px * r + py * leg) / dist_sq;
      } else {
        // RVO2's true right-leg tangent is the negation of the naive mirror
        // of the left-leg formula (Agent.cpp computeNewVelocity): without
        // this flip the derived half-plane normal points INTO the collision
        // cone instead of away from it (verified by the ORCA linear-program
        // unit test).
        dirx = -(px * leg + py * r) / dist_sq;
        diry = (px * r - py * leg) / dist_sq;
      }
      double dot2_ = vx * dirx + vy * diry;
      u = core::Point{dot2_ * dirx - vx, dot2_ * diry - vy};
      normal = core::Point{-diry, dirx};
    }
  } else {  // already colliding: project onto the cutoff circle of the control tick
    double inv_dt = 1.0 / dt;
    double wx = vx - inv_dt * px, wy = vy - inv_dt * py;
    double w_len = std::hypot(wx, wy);
    double unit_wx, unit_wy;
    if (w_len < kEps) {
      unit_wx = 1.0;
      unit_wy = 0.0;  // degenerate exact-alignment fallback
    } else {
      unit_wx = wx / w_len;
      unit_wy = wy / w_len;
    }
    normal = core::Point{unit_wx, unit_wy};
    u = core::Point{(r * inv_dt - w_len) * unit_wx, (r * inv_dt - w_len) * unit_wy};
  }
  core::Point point{v_self.x + 0.5 * u.x, v_self.y + 0.5 * u.y};
  return {point, normal};
}

LinearProgram2DResult linear_program_2d(const std::vector<HalfPlane>& half_planes,
                                        const core::Point& v_pref, double max_speed) {
  Lp2Result r = lp2(half_planes, v_pref, max_speed, false);
  LinearProgram2DResult out;
  out.ok = r.fail_index == static_cast<int>(half_planes.size());
  out.velocity = r.velocity;
  out.fail_index = r.fail_index;
  return out;
}

core::Point linear_program_3d(const std::vector<HalfPlane>& half_planes, int begin_line,
                              const core::Point& v_pref, double max_speed) {
  std::vector<HalfPlane> prefix(half_planes.begin(),
                                half_planes.begin() + begin_line);
  Lp2Result r0 = lp2(prefix, v_pref, max_speed, false);
  core::Point result = r0.velocity;
  double distance = 0.0;
  for (int i = begin_line; i < static_cast<int>(half_planes.size()); ++i) {
    const auto& [point, normal] = half_planes[static_cast<size_t>(i)];
    core::Point direction = direction_of(normal);
    if (dot2(sub2(result, point), normal) < -distance) {
      std::vector<HalfPlane> proj_lines;
      for (int j = 0; j < i; ++j) {
        const auto& [p_j, n_j] = half_planes[static_cast<size_t>(j)];
        core::Point d_j = direction_of(n_j);
        double denominator = det2(direction, d_j);
        core::Point new_point;
        if (std::fabs(denominator) <= kEps) {
          if (dot2(direction, d_j) > 0.0) continue;
          new_point = core::Point{0.5 * (point.x + p_j.x), 0.5 * (point.y + p_j.y)};
        } else {
          double t = det2(d_j, sub2(point, p_j)) / denominator;
          new_point = core::Point{point.x + t * direction.x, point.y + t * direction.y};
        }
        core::Point new_dir = unit2(core::Point{d_j.x - direction.x, d_j.y - direction.y});
        proj_lines.push_back({new_point, normal_of(new_dir)});
      }
      Lp2Result cand = lp2(proj_lines, normal, max_speed, true);
      // A failure here means even the direction-optimizing 1D sub-problem is
      // over-constrained -- RVO2 treats this as float noise around an
      // already-feasible point and keeps the prior `result` rather than
      // raising (this function's hot-path contract).
      if (cand.fail_index == static_cast<int>(proj_lines.size())) {
        result = cand.velocity;
      }
      distance = -dot2(sub2(result, point), normal);
    }
  }
  return result;
}

VelocityObstaclePlanner::VelocityObstaclePlanner(core::ParamSet params)
    : core::ObstacleLocalPlanner(std::move(params)),
      max_speed_(params_.get_float("max_speed")),
      max_omega_(params_.get_float("max_omega")),
      heading_gain_(params_.get_float("heading_gain")),
      agent_radius_(params_.get_float("agent_radius")),
      neighbor_dist_(params_.get_float("neighbor_dist")),
      time_horizon_(params_.get_float("time_horizon")),
      obstacle_radius_(params_.get_float("obstacle_radius")) {}

core::VelocityCommand VelocityObstaclePlanner::compute_command(core::ObstacleQuery& space,
                                                                const core::RobotState& state,
                                                                const core::LocalTask& task,
                                                                double dt,
                                                                core::TraceRecorder* recorder) {
  return command_with_neighbors(space, state, task, {}, dt, recorder);
}

core::VelocityCommand VelocityObstaclePlanner::command_with_neighbors(
    core::ObstacleQuery& space, const core::RobotState& state, const core::LocalTask& task,
    const std::vector<DynamicObstacle>& neighbors, double dt, core::TraceRecorder* recorder) {
  double x = state.pose.x, y = state.pose.y, theta = state.pose.theta;
  std::vector<DynamicObstacle> statics =
      static_obstacles(space, core::Point{x, y}, neighbor_dist_, obstacle_radius_);
  core::Point v_pref = preferred_velocity(state.pose, task.goal, max_speed_);
  VelocitySelection sel = select_velocity(v_pref, neighbors, statics, state, dt);
  if (recorder != nullptr) {
    recorder->velocity_obstacle(
        core::Pose{x, y, theta}, sel.constraints,
        core::TraceRecorder::EventData{{"pref_vx", v_pref.x},
                                       {"pref_vy", v_pref.y},
                                       {"new_vx", sel.velocity.x},
                                       {"new_vy", sel.velocity.y}});
  }
  return velocity_to_command(sel.velocity, theta, max_omega_, heading_gain_);
}

}  // namespace navigation::local_planning
