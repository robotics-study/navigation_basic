#pragma once

#include <functional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "navigation/core/capabilities.hpp"
#include "navigation/core/params.hpp"
#include "navigation/core/planner.hpp"
#include "navigation/core/trace.hpp"
#include "navigation/core/types.hpp"

// Shared geometry/LP machinery for the velocity-obstacle family: VO/RVO/ORCA
// choose the agent's *next absolute velocity* directly in velocity space
// instead of DWA's command-space rollout scoring.
//
// Fiorini & Shiller 1998 (VO, truncated cone) / van den Berg, Lin & Manocha
// 2008 (RVO, reciprocal apex) / van den Berg, Guy, Lin & Manocha 2011 (ORCA,
// half-plane + linear program) all reduce to: build one forbidden region per
// nearby obstacle, then pick the admissible velocity closest to a
// goal-seeking preferred velocity.
namespace navigation::local_planning {

// Numerical guard for divisions by a near-zero vector length / distance --
// not a tunable parameter (no meaningful physical unit), so a module
// constant rather than a config value.
constexpr double kEps = 1e-9;
// Added to an infeasible (VO/RVO-cone-violating) candidate's cost so any
// feasible candidate always outranks it, however close the infeasible one
// sits to v_pref. Not tunable (only its relative dominance over real costs
// matters).
constexpr double kPenalty = 1e6;

struct DynamicObstacle {
  core::Point position;
  core::Point velocity;
  double radius = 0.0;
};

// Truncated velocity obstacle (Fiorini & Shiller 1998): the set of (absolute)
// velocities that, held for `tau` seconds, put the agent inside `radius` of
// the obstacle. `left`/`right` are unit boundary rays from `apex` along the
// two tangents; `full` marks an already-overlapping pair (radius >= dist),
// whose VO is the entire velocity plane.
struct Cone {
  core::Point apex;
  core::Point axis;
  double cos_half = 0.0;
  double dist = 0.0;
  double radius = 0.0;
  double tau = 0.0;
  bool full = false;
  core::Point left;
  core::Point right;
};

// ORCA half-plane: feasible region is {v : dot(v - point, normal) >= 0}, normal unit.
using HalfPlane = std::pair<core::Point, core::Point>;

// Goal-seeking velocity: full speed toward the goal, tapering to a stop once
// within max_speed meters of it so the episode settles into REACHED instead
// of orbiting/overshooting (RVO2's goal heuristic).
core::Point preferred_velocity(const core::Pose& pose, const core::Pose& goal, double max_speed);

// Differential-drive projection of a velocity-space target onto (v, omega),
// reusing the reactive family's turn-in-place-then-drive law: the chosen
// speed itself becomes the law's speed cap, cos-gated by the heading error.
core::VelocityCommand velocity_to_command(const core::Point& v_new, double theta,
                                          double max_omega, double heading_gain);

Cone truncated_vo_cone(const core::Point& rel_pos, double combined_radius,
                       const core::Point& apex_vel, double tau);

bool in_velocity_obstacle(const core::Point& v, const Cone& cone);

std::vector<double> cone_to_constraint(const Cone& cone);
std::vector<double> halfplane_to_constraint(const HalfPlane& plane);

// RVO apex (van den Berg et al. 2008): shift the VO's apex from the other
// agent's velocity toward the midpoint of both velocities so each side
// absorbs half the avoidance effort. reciprocity=0 recovers plain VO (other
// bears all responsibility), 1 collapses the cone onto self.
core::Point rvo_apex(const core::Point& v_self, const core::Point& v_other, double reciprocity);

// Deterministic polar candidate grid for VO/RVO (no RNG): speed-outer /
// angle-inner traversal so py/cpp/TS scoring and tie-breaking stay
// bit-identical. Candidate 0 is v_pref itself (clamped to max_speed) so a
// fully unobstructed tick costs exactly 0 and wins every tie.
std::vector<core::Point> sample_reachable_velocities(const core::Point& v_pref, double max_speed,
                                                      int speed_samples, int angle_samples);

// Occupied cells within `sensor_radius` folded into velocity-0 obstacles so a
// single VO/RVO/ORCA code path handles static walls and moving agents alike.
std::vector<DynamicObstacle> static_obstacles(core::ObstacleQuery& space, const core::Point& center,
                                              double sensor_radius, double obstacle_radius);

// `_select_velocity`'s return value: the chosen next absolute velocity plus
// the trace constraints (cones or half-planes) that produced it.
struct VelocitySelection {
  core::Point velocity;
  std::vector<std::vector<double>> constraints;
};

// Candidate-grid velocity selection shared by VO and RVO: the two differ only
// in how each obstacle's cone apex is placed (`apex_of`), so this owns the
// common cone-building / scan / cost-with-penalty loop (Fiorini & Shiller
// 1998 eq. for VO; van den Berg et al. 2008 for the reciprocal apex),
// avoiding a duplicated scan between vo.cpp and rvo.cpp.
VelocitySelection select_sampled_velocity(
    const core::Point& v_pref, const std::vector<DynamicObstacle>& obstacles, const core::Point& pos,
    double agent_radius, double neighbor_dist, double time_horizon, double max_speed,
    int speed_samples, int angle_samples,
    const std::function<core::Point(const DynamicObstacle&)>& apex_of);

// ORCA line for one obstacle (van den Berg et al. 2011; RVO2 reference
// implementation's Agent::computeNewVelocity). `rel_pos` = other - self,
// `rel_vel` = self - other (both absolute). Returns (point, normal) with
// feasible region `dot(v - point, normal) >= 0`.
HalfPlane orca_half_plane(const core::Point& rel_pos, const core::Point& rel_vel,
                         const core::Point& v_self, double combined_radius, double tau, double dt);

struct LinearProgram2DResult {
  bool ok = false;
  core::Point velocity;
  int fail_index = 0;
};

// RVO2 linearProgram2 wrapper: fail_index == half_planes.size() iff every
// line was satisfied (ok == true).
LinearProgram2DResult linear_program_2d(const std::vector<HalfPlane>& half_planes,
                                        const core::Point& v_pref, double max_speed);

// RVO2 linearProgram3: over-constrained fallback, minimizing total
// penetration across every line from `begin_line` on. Always returns a
// Point -- never raises (hot path). Re-derives (rather than receives) the
// prefix result already satisfied before `begin_line`, since that prefix
// computation is deterministic and bit-identical to what linear_program_2d
// already performed.
core::Point linear_program_3d(const std::vector<HalfPlane>& half_planes, int begin_line,
                              const core::Point& v_pref, double max_speed);

// Template Method base for VO/RVO/ORCA: every tick, gather neighbor + static
// obstacles, compute a goal-seeking preferred velocity, and delegate the
// actual avoidance strategy to `select_velocity`. The three algorithms differ
// only in that one method.
class VelocityObstaclePlanner : public core::ObstacleLocalPlanner {
 public:
  explicit VelocityObstaclePlanner(core::ParamSet params);

  std::set<core::Capability> required_capabilities() const override {
    return {core::Capability::OBSTACLE_QUERY};
  }

  // ABC entry point: static-avoidance-only mode (no neighbors) so this
  // planner remains a drop-in ObstacleLocalPlanner for the single-robot
  // simulator; the multi-agent harness calls command_with_neighbors directly.
  core::VelocityCommand compute_command(core::ObstacleQuery& space, const core::RobotState& state,
                                        const core::LocalTask& task, double dt,
                                        core::TraceRecorder* recorder) override;

  core::VelocityCommand command_with_neighbors(core::ObstacleQuery& space,
                                               const core::RobotState& state,
                                               const core::LocalTask& task,
                                               const std::vector<DynamicObstacle>& neighbors,
                                               double dt, core::TraceRecorder* recorder);

 protected:
  double max_speed_;
  double max_omega_;
  double heading_gain_;
  double agent_radius_;
  double neighbor_dist_;
  double time_horizon_;
  double obstacle_radius_;

  // Chooses the next absolute velocity plus the trace constraints (cones or
  // half-planes) that produced it. `neighbors` and `statics` are kept
  // separate (rather than pre-merged) because ORCA needs a different time
  // horizon for each (moving agents vs. static-cell discs); VO/RVO treat both
  // uniformly and simply concatenate them.
  virtual VelocitySelection select_velocity(const core::Point& v_pref,
                                            const std::vector<DynamicObstacle>& neighbors,
                                            const std::vector<DynamicObstacle>& statics,
                                            const core::RobotState& state, double dt) = 0;
};

}  // namespace navigation::local_planning
