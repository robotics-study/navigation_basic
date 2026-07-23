#pragma once

#include <optional>
#include <random>
#include <set>
#include <string>
#include <vector>

#include "navigation/core/planner.hpp"
#include "navigation/local_planning/predictive/rollout.hpp"

// Model Predictive Path Integral control (Williams, Aldrich & Theodorou, ICRA
// 2016, DOI 10.1109/ICRA.2016.7487277; Williams et al., IEEE T-RO
// 34(6):1603-1622, 2018, DOI 10.1109/TRO.2018.2865891): each tick draws K
// Gaussian-perturbed control sequences, rolls each out, scores it with the same
// shared receding-horizon cost J(U) MPC uses, and updates the nominal sequence by
// a softmax importance-weighted average of the sampled perturbations -- a
// derivative-free optimizer where MPC descends a finite-difference gradient.
// Executes only the first control u_0 and re-optimizes next tick.
//
// MPPI is a stochastic planner: C++ and Python share the trace event types and
// the algorithm, not the exact numeric stream. C++ draws its Gaussian noise from
// std::mt19937 (its own stream) and gives behavioral parity only, matching the
// sampling-planner precedent where C++ RRT uses std::mt19937 while Python uses
// numpy default_rng. Box-Muller (not std::normal_distribution) mirrors the shape
// of Python's noise so the algorithms stay behaviorally identical.
namespace navigation::local_planning {

class MppiPlanner final : public core::ObstacleLocalPlanner {
 public:
  explicit MppiPlanner(core::ParamSet params);

  std::string name() const override { return "mppi"; }
  std::set<core::Capability> required_capabilities() const override {
    return {core::Capability::OBSTACLE_QUERY};
  }
  // Reseed the RNG and drop the nominal sequence so an episode replays
  // identically from a fixed seed (the accel clamp reads RobotState.v, so there
  // is no extra velocity state to reset).
  void reset() override {
    controls_.clear();
    rng_.seed(static_cast<std::mt19937::result_type>(seed_));
    spare_.reset();
  }

  core::VelocityCommand compute_command(core::ObstacleQuery& space, const core::RobotState& state,
                                        const core::LocalTask& task, double dt,
                                        core::TraceRecorder* recorder) override;

 private:
  // Left-shift the previous nominal Û and duplicate its last control (the
  // executed u_0 is dropped), so this tick starts where the last horizon left
  // off. Cold start (empty controls_) seeds H zero controls.
  std::vector<Control> warm_start() const;
  // One standard normal via Box-Muller over the mt19937 uniform stream; two
  // normals per uniform pair, the second cached in spare_. `1.0 - unit(rng)`
  // maps the uniform to (0, 1] so log(0) never occurs.
  double gaussian();
  void emit_band(core::TraceRecorder& recorder, const core::Pose& s0,
                 const std::vector<core::Pose>& traj, double h, double min_cost) const;

  int horizon_;
  int num_samples_;
  double temperature_;
  double sigma_v_;
  double sigma_omega_;
  double w_goal_;
  double w_obstacle_;
  double w_control_;
  double min_obstacle_dist_;
  double v_max_;
  double omega_max_;
  double a_max_;
  double footprint_radius_;
  int seed_;

  // Nominal control sequence Û carried across ticks for warm-starting; empty =
  // cold start (first tick / after reset()), seeded with zeros.
  std::vector<Control> controls_;
  std::mt19937 rng_;
  std::uniform_real_distribution<double> unit_{0.0, 1.0};
  std::optional<double> spare_;
};

}  // namespace navigation::local_planning
