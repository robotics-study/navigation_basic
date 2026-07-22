#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "../demos/demo_common.hpp"
#include "navigation/core/params.hpp"
#include "navigation/core/types.hpp"
#include "navigation/local_planning/simulation.hpp"
#include "navigation/local_planning/tracking/stanley.hpp"
#include "navigation/maps/loader.hpp"
#include "navigation/maps/occupancy_grid.hpp"
#include "test_util.hpp"

using namespace navigation;
using core::LocalTask;
using core::ParamSet;
using core::Point;
using core::RobotState;
using local_planning::SimConfig;
using local_planning::SimStatus;
using local_planning::simulate;
using local_planning::StanleyPlanner;

namespace {

std::string config_path() { return test::repo_path("configs/local_planning/stanley.yaml"); }

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

RealSetup real_setup(const std::string& scenario_path) {
  ParamSet params = ParamSet::from_yaml(config_path());
  maps::Scenario sc = maps::load_scenario(scenario_path);
  RobotState start{core::Pose{sc.start.x, sc.start.y, sc.start_theta}, 0.0, 0.0};
  LocalTask task{core::Pose{sc.goal.x, sc.goal.y, sc.goal_theta}, sc.reference_path};
  SimConfig config{params.get_float("control_dt"),     params.get_int("max_steps"),
                   params.get_float("goal_tolerance"), params.get_float("footprint_radius"),
                   params.get_int("stall_window"),     params.get_float("stall_distance")};
  return RealSetup{std::move(params), start, std::move(task), config};
}

}  // namespace

// (a) follows the open01_s2 S-curve to the goal with a tight terminal crosstrack error ----
TEST(Stanley, FollowsSCurveToGoalWithSmallTerminalCrossTrackError) {
  RealSetup setup = real_setup(test::repo_path("maps/scenarios/open01_s2.yaml"));
  auto map = maps::load_map(test::repo_path("maps/grid/open01.yaml"));
  auto& grid = demo::as_grid(*map);
  StanleyPlanner planner(setup.params);

  auto result = simulate(planner, grid, setup.start, setup.task, setup.config, nullptr);

  ASSERT_EQ(result.status, SimStatus::REACHED);
  EXPECT_TRUE(result.success);
  const core::Pose& terminal = result.trajectory.back();
  double terminal_cross_track =
      dist_to_polyline(Point{terminal.x, terminal.y}, setup.task.reference_path);
  EXPECT_LT(terminal_cross_track, 0.5);
}

// (b) open01_s3 offset start converges: crosstrack error shrinks toward the path ----
TEST(Stanley, OffsetStartConvergesTowardReferencePath) {
  RealSetup setup = real_setup(test::repo_path("maps/scenarios/open01_s3.yaml"));
  auto map = maps::load_map(test::repo_path("maps/grid/open01.yaml"));
  auto& grid = demo::as_grid(*map);
  StanleyPlanner planner(setup.params);

  auto result = simulate(planner, grid, setup.start, setup.task, setup.config, nullptr);

  ASSERT_EQ(result.status, SimStatus::REACHED);
  std::vector<double> errors;
  errors.reserve(result.trajectory.size());
  for (const auto& pose : result.trajectory) {
    errors.push_back(dist_to_polyline(Point{pose.x, pose.y}, setup.task.reference_path));
  }
  double initial_offset = errors.front();
  EXPECT_GT(initial_offset, 1.0);
  double tail_min = *std::min_element(errors.end() - 5, errors.end());
  EXPECT_LT(tail_min, initial_offset * 0.1);
  bool found_monotone_run = false;
  for (size_t i = 0; i + 3 < errors.size(); ++i) {
    if (errors[i] >= errors[i + 1] && errors[i + 1] >= errors[i + 2] &&
        errors[i + 2] >= errors[i + 3]) {
      found_monotone_run = true;
      break;
    }
  }
  EXPECT_TRUE(found_monotone_run);
}

// (c) low-speed singularity: v near 0 with a large crosstrack error still -----------
// yields a finite command (k_soft prevents the arctan argument from blowing up) -----
TEST(Stanley, LowSpeedLargeCrossTrackErrorYieldsFiniteCommand) {
  ParamSet params = ParamSet::from_yaml(config_path());
  auto map = maps::load_map(test::repo_path("maps/grid/open01.yaml"));
  auto& grid = demo::as_grid(*map);
  StanleyPlanner planner(params);
  // Straight path along +x; robot sits far off to the side (large e) but at
  // its own goal (remaining ~= 0), so the speed profile drives v to ~0.
  LocalTask task{core::Pose{5.0, 5.0, 0.0}, {Point{0.0, 0.0}, Point{10.0, 0.0}}};
  RobotState state{core::Pose{5.0, 5.0, 0.0}, 0.0, 0.0};

  core::VelocityCommand cmd =
      planner.compute_command(grid, state, task, params.get_float("control_dt"), nullptr);

  EXPECT_TRUE(std::isfinite(cmd.v));
  EXPECT_TRUE(std::isfinite(cmd.omega));
  EXPECT_LE(std::fabs(cmd.omega), params.get_float("max_omega") + 1e-9);
}

// (c2) degenerate paths: duplicated waypoints / a single point stay finite ---------
TEST(Stanley, DegenerateReferencePathsYieldFiniteCommands) {
  // A duplicated waypoint makes a zero-length segment (no tangent direction)
  // and a single-point path has no segment at all -- the planner must fall
  // back to a finite command instead of dividing by zero or indexing past
  // the path.
  ParamSet params = ParamSet::from_yaml(config_path());
  auto map = maps::load_map(test::repo_path("maps/grid/open01.yaml"));
  auto& grid = demo::as_grid(*map);
  const double dt = params.get_float("control_dt");

  {
    StanleyPlanner planner(params);
    LocalTask task{core::Pose{9.0, 9.0, 0.0},
                   {Point{1.0, 1.0}, Point{1.0, 1.0}, Point{1.0, 1.0}}};
    RobotState state{core::Pose{1.0, 1.2, 0.0}, 0.0, 0.0};
    core::VelocityCommand cmd = planner.compute_command(grid, state, task, dt, nullptr);
    EXPECT_TRUE(std::isfinite(cmd.v));
    EXPECT_TRUE(std::isfinite(cmd.omega));
  }
  {
    StanleyPlanner planner(params);
    LocalTask task{core::Pose{5.0, 5.0, 0.0}, {Point{5.0, 5.0}}};
    RobotState state{core::Pose{1.0, 1.0, 0.0}, 0.0, 0.0};
    core::VelocityCommand cmd = planner.compute_command(grid, state, task, dt, nullptr);
    EXPECT_TRUE(std::isfinite(cmd.v));
    EXPECT_TRUE(std::isfinite(cmd.omega));
    // The fallback tangent aims at the path's end, so from below-left of the
    // point the commanded turn is toward it (counterclockwise, positive).
    EXPECT_GT(cmd.omega, 0.0);
  }
}

// (d) max_steer above the declared range (1.55) fails load-time validation ---------
TEST(Stanley, MaxSteerAboveRangeRejectedAtLoadTime) {
  std::string bad = test::write_temp(
      "stanley.yaml",
      "algorithm: stanley\ncategory: local_planning\nparams:\n"
      "  - {name: k_gain, type: float, default: 2.5, min: 0.01, max: 100, description: k}\n"
      "  - {name: k_soft, type: float, default: 0.5, min: 0.001, max: 10, description: ks}\n"
      "  - {name: wheelbase, type: float, default: 0.3, min: 0.01, max: 10, description: L}\n"
      "  - {name: max_steer, type: float, default: 2.0, min: 0.01, max: 1.55, description: ms}\n"
      "  - {name: max_speed, type: float, default: 0.8, min: 0.01, max: 10, description: v}\n"
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

// (behavior) reset() leaves no progress_index_ cursor leak across reruns ------------
TEST(Stanley, RerunAfterImplicitResetIsDeterministic) {
  // simulate() calls planner.reset() itself at the top of every episode -- this
  // proves that call actually clears progress_index_ rather than the planner
  // silently resuming from wherever the first run's cursor stopped.
  RealSetup setup = real_setup(test::repo_path("maps/scenarios/open01_s2.yaml"));
  auto map = maps::load_map(test::repo_path("maps/grid/open01.yaml"));
  auto& grid = demo::as_grid(*map);
  StanleyPlanner planner(setup.params);

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
