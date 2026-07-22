#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "../demos/demo_common.hpp"
#include "navigation/core/params.hpp"
#include "navigation/core/types.hpp"
#include "navigation/local_planning/simulation.hpp"
#include "navigation/local_planning/tracking/pure_pursuit.hpp"
#include "navigation/maps/loader.hpp"
#include "navigation/maps/occupancy_grid.hpp"
#include "test_util.hpp"

using namespace navigation;
using core::LocalTask;
using core::ParamSet;
using core::Point;
using core::RobotState;
using local_planning::PurePursuitPlanner;
using local_planning::SimConfig;
using local_planning::SimStatus;
using local_planning::simulate;

namespace {

constexpr const char* kAlgo = "pure_pursuit";

std::string config_path() { return test::repo_path("configs/local_planning/pure_pursuit.yaml"); }

double dist_to_polyline(const Point& p, const std::vector<Point>& path) {
  double best = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i + 1 < path.size(); ++i) {
    const Point& a = path[i];
    const Point& b = path[i + 1];
    double dx = b.x - a.x, dy = b.y - a.y;
    double seg_len_sq = dx * dx + dy * dy;
    double t = seg_len_sq < 1e-12
                   ? 0.0
                   : std::max(0.0, std::min(1.0, ((p.x - a.x) * dx + (p.y - a.y) * dy) / seg_len_sq));
    double cx = a.x + t * dx, cy = a.y + t * dy;
    best = std::min(best, std::hypot(p.x - cx, p.y - cy));
  }
  return best;
}

struct RealSetup {
  ParamSet params;
  RobotState start;
  LocalTask task;
  SimConfig config;
};

RealSetup real_setup() {
  ParamSet params = ParamSet::from_yaml(config_path());
  maps::Scenario sc =
      maps::load_scenario(test::repo_path("maps/scenarios/open01_s2.yaml"));
  RobotState start{core::Pose{sc.start.x, sc.start.y, sc.start_theta}, 0.0, 0.0};
  LocalTask task{core::Pose{sc.goal.x, sc.goal.y, sc.goal_theta}, sc.reference_path};
  SimConfig config{params.get_float("control_dt"),     params.get_int("max_steps"),
                   params.get_float("goal_tolerance"), params.get_float("footprint_radius"),
                   params.get_int("stall_window"),     params.get_float("stall_distance")};
  return RealSetup{std::move(params), start, std::move(task), config};
}

}  // namespace

// (a) follows the open01_s2 S-curve to the goal, bounded cross-track error -------
TEST(PurePursuit, FollowsSCurveToGoalWithBoundedCrossTrackError) {
  RealSetup setup = real_setup();
  auto map = maps::load_map(test::repo_path("maps/grid/open01.yaml"));
  auto& grid = demo::as_grid(*map);
  PurePursuitPlanner planner(setup.params);

  auto result = simulate(planner, grid, setup.start, setup.task, setup.config, nullptr);

  ASSERT_EQ(result.status, SimStatus::REACHED);
  EXPECT_TRUE(result.success);
  double max_cross_track = 0.0;
  for (const auto& pose : result.trajectory) {
    max_cross_track =
        std::max(max_cross_track, dist_to_polyline(Point{pose.x, pose.y}, setup.task.reference_path));
  }
  EXPECT_LT(max_cross_track, setup.params.get_float("lookahead_distance"));
}

// (b) missing reference_path is rejected at assembly time, not mid-tick ----------
TEST(PurePursuit, RunLocalRejectsScenarioWithoutReferencePath) {
  std::string trace_path = test::write_temp("pp_no_path_trace.jsonl", "");
  std::vector<std::string> argv_storage = {
      "demo_pure_pursuit",
      "--map",
      test::repo_path("maps/grid/open01.yaml"),
      "--scenario",
      test::repo_path("maps/scenarios/open01_s1.yaml"),
      "--params",
      config_path(),
      "--trace",
      trace_path,
  };
  std::vector<char*> argv;
  for (auto& s : argv_storage) argv.push_back(s.data());

  EXPECT_THROW(
      demo::run_local(static_cast<int>(argv.size()), argv.data(), kAlgo,
                      [](const ParamSet& p) { return PurePursuitPlanner(p); }),
      std::runtime_error);
}

// (c) lookahead_distance = 0 fails load-time range validation --------------------
TEST(PurePursuit, ZeroLookaheadDistanceRejectedAtLoadTime) {
  std::string bad = test::write_temp(
      "pure_pursuit.yaml",
      "algorithm: pure_pursuit\ncategory: local_planning\nparams:\n"
      "  - {name: lookahead_distance, type: float, default: 0.0, min: 0.01, max: 100, "
      "description: ld}\n"
      "  - {name: max_speed, type: float, default: 0.8, min: 0.01, max: 10, description: ms}\n"
      "  - {name: max_omega, type: float, default: 1.5, min: 0.01, max: 20, description: mo}\n"
      "  - {name: slow_radius, type: float, default: 0.5, min: 0.001, max: 100, description: sr}\n"
      "  - {name: control_dt, type: float, default: 0.1, min: 0.001, max: 1, description: dt}\n"
      "  - {name: max_steps, type: int, default: 1000, min: 1, max: 100000, description: n}\n"
      "  - {name: goal_tolerance, type: float, default: 0.3, min: 0.01, max: 5, description: gt}\n"
      "  - {name: footprint_radius, type: float, default: 0.2, min: 0.01, max: 5, description: fp}\n"
      "  - {name: stall_window, type: int, default: 20, min: 1, max: 100000, description: sw}\n"
      "  - {name: stall_distance, type: float, default: 0.05, min: 0.0, max: 5, "
      "description: sd}\n");
  EXPECT_THROW(ParamSet::from_yaml(bad), std::runtime_error);
}

// (behavior) reset() leaves no progress_index_ cursor leak across reruns ---------
TEST(PurePursuit, RerunAfterImplicitResetIsDeterministic) {
  // simulate() calls planner.reset() itself at the top of every episode -- this
  // proves that call actually clears progress_index_ rather than the planner
  // silently resuming from wherever the first run's cursor stopped.
  RealSetup setup = real_setup();
  auto map = maps::load_map(test::repo_path("maps/grid/open01.yaml"));
  auto& grid = demo::as_grid(*map);
  PurePursuitPlanner planner(setup.params);

  auto first = simulate(planner, grid, setup.start, setup.task, setup.config, nullptr);
  auto second = simulate(planner, grid, setup.start, setup.task, setup.config, nullptr);

  EXPECT_EQ(second.status, first.status);
  EXPECT_EQ(second.steps, first.steps);
  EXPECT_NEAR(second.distance_traveled, first.distance_traveled, 1e-9);
  ASSERT_EQ(second.trajectory.size(), first.trajectory.size());
  for (size_t i = 0; i < first.trajectory.size(); ++i) {
    EXPECT_NEAR(first.trajectory[i].x, second.trajectory[i].x, 1e-9);
    EXPECT_NEAR(first.trajectory[i].y, second.trajectory[i].y, 1e-9);
    EXPECT_NEAR(first.trajectory[i].theta, second.trajectory[i].theta, 1e-9);
  }
}
