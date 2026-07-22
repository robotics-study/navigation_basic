#pragma once

#include <set>
#include <string>
#include <vector>

#include "navigation/core/planner.hpp"

// Vector Field Histogram (Borenstein & Koren 1991): polar-histogram reactive
// obstacle avoidance. Each tick bins nearby obstacles by bearing into a
// circular histogram, smooths it, finds the free "valleys" below a density
// threshold, and steers toward whichever valley best serves the goal
// direction. No search, no memory -- each tick recomputes the histogram from
// scratch, so the planner is entirely stateless (reset() stays the base
// no-op).
namespace navigation::local_planning {

class VfhPlanner final : public core::ObstacleLocalPlanner {
 public:
  explicit VfhPlanner(core::ParamSet params);
  std::string name() const override { return "vfh"; }
  std::set<core::Capability> required_capabilities() const override {
    return {core::Capability::OBSTACLE_QUERY};
  }
  core::VelocityCommand compute_command(core::ObstacleQuery& space, const core::RobotState& state,
                                        const core::LocalTask& task, double dt,
                                        core::TraceRecorder* recorder) override;

 private:
  // (start_sector, end_sector, width): a maximal circular run of sectors whose
  // smoothed histogram value is below threshold. `end_sector` is reached from
  // `start_sector` by walking +1 (mod n) `width` times -- the run may wrap
  // past sector n-1 back to 0, so the two borders are not necessarily
  // start <= end.
  struct Valley {
    int start;
    int end;
    int width;
  };

  int sector_of(double angle) const;
  double sector_center(double index) const;
  std::vector<double> build_histogram(core::ObstacleQuery& space, double x, double y) const;
  std::vector<double> smooth(const std::vector<double>& h) const;
  std::vector<Valley> find_valleys(const std::vector<bool>& below) const;
  static bool valley_contains(const Valley& v, int k, int n);
  static int circular_dist(int a, int b, int n);
  int nearest_sector_gap(const Valley& v, int k_target) const;
  double candidate_direction(const Valley& v, int k_target, double goal_dir) const;

  int num_sectors_;
  double window_radius_;
  double threshold_;
  int smoothing_window_;
  int wide_valley_sectors_;
  double h_m_;
  double k_omega_;
  double max_speed_;
  double max_omega_;
  double delta_;  // 2*pi / num_sectors_
};

}  // namespace navigation::local_planning
