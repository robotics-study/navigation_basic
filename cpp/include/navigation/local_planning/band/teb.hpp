#pragma once

#include <set>
#include <string>
#include <vector>

#include "navigation/core/planner.hpp"

// Timed Elastic Band (Rösmann, Feiten, Wösch, Hoffmann & Bertram, ROBOTIK
// 2012; extended in Rösmann et al., Robotics and Autonomous Systems
// 88:142-153, 2017, DOI 10.1016/j.robot.2016.11.007): augments an Elastic-
// Bands-style pose chain with a per-segment time interval dT_i and optimizes
// both jointly against a multi-objective soft-constraint cost (reference
// tracking, obstacle clearance, velocity/acceleration limits, time-
// optimality, and a nonholonomic two-pose-arc constraint) via a fixed number
// of damped gradient-descent steps -- no external solver, so every iteration,
// clamp, and floating-point accumulation order is spelled out explicitly and
// mirrored bit-for-bit from the Python port.
namespace navigation::local_planning {

class TebPlanner final : public core::ObstacleLocalPlanner {
 public:
  explicit TebPlanner(core::ParamSet params);

  std::string name() const override { return "teb"; }
  std::set<core::Capability> required_capabilities() const override {
    return {core::Capability::OBSTACLE_QUERY};
  }
  bool requires_reference_path() const override { return true; }
  void reset() override {
    poses_.clear();
    dts_.clear();
    progress_index_ = 0;
  }

  core::VelocityCommand compute_command(core::ObstacleQuery& space, const core::RobotState& state,
                                        const core::LocalTask& task, double dt,
                                        core::TraceRecorder* recorder) override;

 private:
  // Walks the reference path forward from `origin` (already projected onto
  // path[start_index]->path[start_index+1]) up to arc-length `horizon_`,
  // returning the local-goal pose and the clipped polyline (used both to
  // anchor the path-attraction cost and, on re-init, to resample the initial
  // band). If the remaining path is shorter than horizon_, the local goal is
  // the final goal itself (inheriting its theta) rather than an interpolated
  // point.
  std::pair<core::Pose, std::vector<core::Point>> clip(const std::vector<core::Point>& path,
                                                        int start_index, const core::Point& origin,
                                                        double goal_theta) const;
  // Re-initializes the band from the clipped reference path, resampled at
  // v_max*dt_ref spacing. dT_i = ell_i/(0.5*v_max) is a deliberately
  // conservative (slow) initial guess -- the time-optimality term then pulls
  // it down during optimization -- rather than an attempt at the true
  // achievable time.
  std::pair<std::vector<core::Pose>, std::vector<double>> init_band(
      const std::vector<core::Point>& clip_points, double local_goal_theta) const;
  // Forward while-loop (mirrors Elastic Bands' overlap maintenance): splits
  // an over-long interval with a wrap-aware midpoint pose, merges an
  // under-short interval into its neighbor. Never touches dts[i] when i+1 is
  // out of range, which keeps the merge from ever deleting the fixed
  // local-goal pose at index n-1.
  void resize(std::vector<core::Pose>& poses, std::vector<double>& dts) const;
  // One damped gradient-descent iteration: accumulates every cost term's
  // gradient in the fixed order (a) path -> (b) obstacle -> (c) velocity ->
  // (d) acceleration -> (f) kinematics -> (e) time, each with i ascending,
  // then applies clamped updates (positions/theta first, then every dT) --
  // this order, not just the final result, is part of the cross-language
  // determinism contract.
  void gradient_step(std::vector<core::Pose>& poses, std::vector<double>& dts,
                     const std::vector<core::Point>& anchors, core::ObstacleQuery& space) const;
  // Re-evaluates every cost term (no gradient) from the final optimized
  // state -- kept as a separate pass, gated by the recorder check at the call
  // site, so the hot solver loop never pays for a scalar it only needs when
  // tracing is on.
  double total_cost(const std::vector<core::Pose>& poses, const std::vector<double>& dts,
                    const std::vector<core::Point>& anchors, core::ObstacleQuery& space) const;
  void emit_band(core::TraceRecorder& recorder, const std::vector<core::Pose>& poses,
                const std::vector<double>& dts, int iterations, double total_cost) const;

  double w_path_;
  double w_obstacle_;
  double w_velocity_;
  double w_acceleration_;
  double w_time_;
  double w_kinematics_;
  double v_max_;
  double omega_max_;
  double a_max_;
  double min_obstacle_dist_;
  double dt_ref_;
  double dt_min_;
  double horizon_;
  int iterations_;
  double step_alpha_;
  double max_step_xy_;
  double max_step_theta_;
  double max_step_dt_;
  int max_poses_;
  double reinit_distance_;

  // Band state: poses_[0] is always overwritten to the executed robot pose
  // and poses_.back() to the current local goal every tick; empty = no band
  // (first tick / after reset()), forcing re-initialization.
  std::vector<core::Pose> poses_;
  std::vector<double> dts_;
  // Reference-path segment cursor for the monotonic forward projection
  // (advance_progress_index) -- distinct from the band's own internal
  // warm-start cursor, which tracks the band's first edge, not the path's.
  int progress_index_ = 0;
};

}  // namespace navigation::local_planning
