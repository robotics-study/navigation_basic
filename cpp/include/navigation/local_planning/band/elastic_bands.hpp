#pragma once

#include <set>
#include <string>
#include <utility>
#include <vector>

#include "navigation/core/planner.hpp"

// Elastic Bands (Quinlan & Khatib, "Elastic bands: connecting path planning
// and control," ICRA 1993, DOI 10.1109/ROBOT.1993.291936): represents a
// corridor of free space around a reference path as a chain of bubbles --
// world-space discs sized to local clearance -- and deforms the chain every
// tick under an internal contraction force (keeps it taut) and an external
// repulsion force (keeps it off obstacles), then drives toward a lookahead
// point on the deformed polyline instead of tracking a fixed discrete path.
namespace navigation::local_planning {

class ElasticBandsPlanner final : public core::ObstacleLocalPlanner {
 public:
  explicit ElasticBandsPlanner(core::ParamSet params);

  std::string name() const override { return "elastic_bands"; }
  std::set<core::Capability> required_capabilities() const override {
    return {core::Capability::OBSTACLE_QUERY};
  }
  bool requires_reference_path() const override { return true; }
  void reset() override {
    centers_.clear();
    radii_.clear();
  }

  core::VelocityCommand compute_command(core::ObstacleQuery& space, const core::RobotState& state,
                                        const core::LocalTask& task, double dt,
                                        core::TraceRecorder* recorder) override;

 private:
  double clearance(core::ObstacleQuery& space, const core::Point& p) const;
  // Builds the initial band: the reference path resampled at even arc-length
  // spacing, anchored at the robot's position and the goal, then repaired with
  // repair_iterations of deformation so a bubble that starts inside an
  // obstacle (rho=0) gets a chance to push itself out before regular per-tick
  // deformation begins.
  void initialize(core::ObstacleQuery& space, const core::LocalTask& task, const core::Point& robot_xy);
  // One Jacobi pass over interior bubbles: every force is computed from a
  // snapshot of the pass-start band, then every displacement is applied as a
  // batch afterward, so visitation order never changes what force any bubble
  // sees (order-independent, and keeps py/C++ numerically identical).
  void deform_once(core::ObstacleQuery& space);
  // Overlap maintenance + post-maintenance validity check. Returns false
  // (broken) if a gap can't be bridged within max_bubbles, a bridging midpoint
  // itself sits below rho_min, or an interior bubble survives below rho_min.
  bool maintain(core::ObstacleQuery& space);
  void emit_band(core::TraceRecorder& recorder, double broken) const;
  // Serializes the band exactly as it stands (broken=1), discards it (the next
  // tick re-initializes), and returns the zero command.
  core::VelocityCommand on_broken(core::TraceRecorder* recorder);

  double k_contraction_;
  double k_repulsion_;
  double rho_max_;
  double rho_influence_;
  double rho_min_;
  double step_size_;
  int deform_iterations_;
  int repair_iterations_;
  double repair_step_;
  double overlap_factor_;
  int max_bubbles_;
  double bubble_spacing_;
  double lookahead_distance_;
  double heading_gain_;
  double v_max_;
  double omega_max_;

  std::vector<core::Point> centers_;
  std::vector<double> radii_;
};

}  // namespace navigation::local_planning
