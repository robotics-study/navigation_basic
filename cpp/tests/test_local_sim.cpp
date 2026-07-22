#include <cmath>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "navigation/core/planner.hpp"
#include "navigation/core/trace.hpp"
#include "navigation/core/types.hpp"
#include "navigation/local_planning/geometry.hpp"
#include "navigation/local_planning/simulation.hpp"
#include "navigation/maps/occupancy_grid.hpp"
#include "test_util.hpp"

using namespace navigation;
using core::Capability;
using core::LocalTask;
using core::ObstacleLocalPlanner;
using core::ObstacleQuery;
using core::ParamSet;
using core::Pose;
using core::RobotState;
using core::TraceRecorder;
using core::VelocityCommand;
using local_planning::SimConfig;
using local_planning::SimResult;
using local_planning::SimStatus;
using local_planning::simulate;

namespace {

// --- minimal script planners: fixtures that live inside the test file, not
// production algorithms, so they relax no visibility on the real planners -----

// Constant forward speed, no turning -- exercises integrate_unicycle's straight
// branch (omega == 0) and the REACHED / COLLISION / TIMEOUT termination paths.
class StraightPlanner : public ObstacleLocalPlanner {
 public:
  explicit StraightPlanner(double v) : ObstacleLocalPlanner(ParamSet{}), v_(v) {}
  std::string name() const override { return "straight_script"; }
  std::set<Capability> required_capabilities() const override {
    return {Capability::OBSTACLE_QUERY};
  }
  VelocityCommand compute_command(ObstacleQuery&, const RobotState&, const LocalTask&, double,
                                  TraceRecorder*) override {
    return VelocityCommand{v_, 0.0};
  }

 private:
  double v_;
};

// Constant (v, omega) every tick -- the executed trajectory must lie exactly on
// the closed-form arc's circle (radius v/omega).
class ConstantArcPlanner : public ObstacleLocalPlanner {
 public:
  ConstantArcPlanner(double v, double omega) : ObstacleLocalPlanner(ParamSet{}), v_(v), omega_(omega) {}
  std::string name() const override { return "arc_script"; }
  std::set<Capability> required_capabilities() const override {
    return {Capability::OBSTACLE_QUERY};
  }
  VelocityCommand compute_command(ObstacleQuery&, const RobotState&, const LocalTask&, double,
                                  TraceRecorder*) override {
    return VelocityCommand{v_, omega_};
  }

 private:
  double v_;
  double omega_;
};

// Never moves -- net displacement over any window is exactly 0, so this drives
// the STALLED path deterministically.
class ZeroPlanner : public ObstacleLocalPlanner {
 public:
  ZeroPlanner() : ObstacleLocalPlanner(ParamSet{}) {}
  std::string name() const override { return "zero_script"; }
  std::set<Capability> required_capabilities() const override {
    return {Capability::OBSTACLE_QUERY};
  }
  VelocityCommand compute_command(ObstacleQuery&, const RobotState&, const LocalTask&, double,
                                  TraceRecorder*) override {
    return VelocityCommand{0.0, 0.0};
  }
};

// Turns for the first `turn_ticks_` calls (mutating `ticks_`), then drives
// straight. Used to prove simulate() calls planner.reset() at episode start:
// re-running the SAME instance twice must not let the first episode's counter
// leak into the second.
class CountingPlanner : public ObstacleLocalPlanner {
 public:
  explicit CountingPlanner(int turn_ticks)
      : ObstacleLocalPlanner(ParamSet{}), turn_ticks_(turn_ticks) {}
  std::string name() const override { return "counting_script"; }
  std::set<Capability> required_capabilities() const override {
    return {Capability::OBSTACLE_QUERY};
  }
  void reset() override { ticks_ = 0; }
  VelocityCommand compute_command(ObstacleQuery&, const RobotState&, const LocalTask&, double,
                                  TraceRecorder*) override {
    bool turning = ticks_ < turn_ticks_;
    ++ticks_;
    return turning ? VelocityCommand{0.3, 0.8} : VelocityCommand{0.5, 0.0};
  }

 private:
  int turn_ticks_;
  int ticks_ = 0;
};

// Large open grid (no interior obstacles) so straight/arc runs never collide
// unless the test deliberately places a wall.
maps::OccupancyGrid2D open_grid(int n, double resolution = 0.5) {
  std::vector<bool> free_cells(static_cast<size_t>(n) * n, true);
  return maps::OccupancyGrid2D(n, n, resolution, 0.0, 0.0, free_cells);
}

// A conservative default SimConfig: stall_window larger than any test's
// max_steps so STALLED never fires as a side effect in tests that aren't
// exercising it.
SimConfig base_config(double control_dt, int max_steps, double goal_tolerance,
                      double footprint_radius = 0.1) {
  return SimConfig{control_dt, max_steps, goal_tolerance, footprint_radius, max_steps + 1, 1e-9};
}

}  // namespace

// ① straight command -> REACHED, time_to_goal == steps * dt --------------------

TEST(LocalSim, StraightReachesGoalWithExactTimeToGoal) {
  auto grid = open_grid(40);
  RobotState start{Pose{2.0, 2.0, 0.0}, 0.0, 0.0};  // clear of the map boundary
  LocalTask task{Pose{3.0, 2.0, 0.0}, {}};
  SimConfig config = base_config(/*dt=*/0.1, /*max_steps=*/50, /*goal_tolerance=*/0.05);
  StraightPlanner planner(/*v=*/0.5);

  SimResult r = simulate(planner, grid, start, task, config, nullptr);

  ASSERT_EQ(r.status, SimStatus::REACHED);
  EXPECT_TRUE(r.success);
  EXPECT_EQ(r.steps, 20);  // 0.5 m/s * 0.1 s * 20 ticks = 1.0 m == goal distance
  EXPECT_DOUBLE_EQ(r.time_to_goal, r.steps * config.control_dt);
}

// ② driving straight into a wall -> COLLISION within a couple of ticks ---------

TEST(LocalSim, WallAheadCausesCollision) {
  std::vector<std::string> rows(8, std::string(10, '.'));
  for (auto& row : rows) row[3] = '#';  // vertical wall at col 3 -> x in [1.5, 2.0)
  auto grid = test::make_grid(rows);
  RobotState start{Pose{1.2, 1.0, 0.0}, 0.0, 0.0};
  LocalTask task{Pose{4.0, 1.0, 0.0}, {}};
  SimConfig config = base_config(/*dt=*/0.1, /*max_steps=*/50, /*goal_tolerance=*/0.01,
                                 /*footprint_radius=*/0.15);
  StraightPlanner planner(/*v=*/1.0);

  SimResult r = simulate(planner, grid, start, task, config, nullptr);

  ASSERT_EQ(r.status, SimStatus::COLLISION);
  EXPECT_FALSE(r.success);
  EXPECT_LE(r.steps, 3);
  // The colliding tick's pose is never appended to the executed trajectory.
  EXPECT_EQ(r.trajectory.size(), static_cast<size_t>(r.steps));
}

// ③ zero command sustained -> STALLED -------------------------------------------

TEST(LocalSim, ZeroCommandStalls) {
  auto grid = open_grid(40);
  RobotState start{Pose{2.0, 2.0, 0.0}, 0.0, 0.0};
  LocalTask task{Pose{15.0, 15.0, 0.0}, {}};
  SimConfig config{/*control_dt=*/0.1, /*max_steps=*/50, /*goal_tolerance=*/0.05,
                   /*footprint_radius=*/0.1, /*stall_window=*/3, /*stall_distance=*/0.01};
  ZeroPlanner planner;

  SimResult r = simulate(planner, grid, start, task, config, nullptr);

  ASSERT_EQ(r.status, SimStatus::STALLED);
  EXPECT_FALSE(r.success);
  EXPECT_EQ(r.steps, config.stall_window);  // displacement is 0 from tick 1 on
  EXPECT_DOUBLE_EQ(r.distance_traveled, 0.0);
}

// ④ goal unreachable within the budget -> TIMEOUT --------------------------------

TEST(LocalSim, ExhaustsBudgetWithoutReachingOrStalling) {
  auto grid = open_grid(200, /*resolution=*/1.0);  // 200m x 200m, plenty of room
  RobotState start{Pose{100.0, 100.0, 0.0}, 0.0, 0.0};
  LocalTask task{Pose{-500.0, -500.0, 0.0}, {}};  // far outside the map -> unreachable
  SimConfig config = base_config(/*dt=*/0.1, /*max_steps=*/5, /*goal_tolerance=*/0.05);
  StraightPlanner planner(/*v=*/0.5);  // base_config() sets stall_window > max_steps: no stall

  SimResult r = simulate(planner, grid, start, task, config, nullptr);

  ASSERT_EQ(r.status, SimStatus::TIMEOUT);
  EXPECT_FALSE(r.success);
  EXPECT_EQ(r.steps, config.max_steps);
  EXPECT_DOUBLE_EQ(r.time_to_goal, config.max_steps * config.control_dt);
}

// ⑤ constant (v, omega) traces the closed-form arc's circle ----------------------

TEST(LocalSim, ConstantArcCommandTracesExactCircle) {
  auto grid = open_grid(400, /*resolution=*/0.25);  // 100m x 100m: room for a 1m-radius loop
  const double v = 1.0, omega = 1.0;
  const double radius = v / omega;
  RobotState start{Pose{50.0, 50.0, 0.0}, 0.0, 0.0};
  // Turning left (omega > 0) from theta0=0: the arc's center is at (x0, y0 + radius).
  const double cx = 50.0, cy = 50.0 + radius;
  LocalTask task{Pose{-1000.0, -1000.0, 0.0}, {}};  // unreachable -> run the full budget
  SimConfig config = base_config(/*dt=*/0.05, /*max_steps=*/40, /*goal_tolerance=*/0.01);
  ConstantArcPlanner planner(v, omega);

  SimResult r = simulate(planner, grid, start, task, config, nullptr);

  ASSERT_EQ(r.status, SimStatus::TIMEOUT);
  for (const Pose& p : r.trajectory) {
    double d2 = (p.x - cx) * (p.x - cx) + (p.y - cy) * (p.y - cy);
    EXPECT_NEAR(std::sqrt(d2), radius, 1e-9);
  }
}

// ⑥ min_clearance / distance_traveled hand-check ---------------------------------

TEST(LocalSim, MinClearanceAndDistanceTraveledMatchHandComputation) {
  // 20m x 20m grid, resolution 1.0: one occupied cell, everything else free.
  // Occupied cell (row=10, col=15) sits at world center (15.5, 9.5) -- see
  // OccupancyGrid2D::cell_to_world.
  std::vector<bool> free_cells(20 * 20, true);
  free_cells[static_cast<size_t>(10) * 20 + 15] = false;
  maps::OccupancyGrid2D grid(20, 20, /*resolution=*/1.0, 0.0, 0.0, free_cells);

  // theta=0 keeps y exactly 9.5 (row 10) for the whole run -- distance_to_nearest
  // is a cell-center-to-cell-center query, so every tick's clearance is exactly
  // |15 - col| cells away from the obstacle at (row 10, col 15).
  RobotState start{Pose{10.5, 9.5, 0.0}, 0.0, 0.0};  // cell (row 10, col 10)
  LocalTask task{Pose{-1000.0, -1000.0, 0.0}, {}};    // unreachable -> run the full budget
  SimConfig config = base_config(/*dt=*/1.0, /*max_steps=*/3, /*goal_tolerance=*/0.01);
  StraightPlanner planner(/*v=*/1.0);  // 1 m/tick -> exactly one cell per tick

  SimResult r = simulate(planner, grid, start, task, config, nullptr);

  ASSERT_EQ(r.status, SimStatus::TIMEOUT);
  // clearance sequence (cells from col 15): start col10 -> 5, tick1 col11 -> 4,
  // tick2 col12 -> 3, tick3 col13 -> 2. Monotonically decreasing -> min is the
  // last tick's value.
  EXPECT_DOUBLE_EQ(r.min_clearance, 2.0);
  EXPECT_DOUBLE_EQ(r.distance_traveled, 3.0);  // 3 ticks * 1 m/tick, exact (sin/cos(0))
}

// ⑦ recorder emits the expected events; a null recorder emits nothing -----------

TEST(LocalSim, RecorderEmitsEventsAndNullRecorderIsSafe) {
  auto grid = open_grid(40);
  RobotState start{Pose{2.0, 2.0, 0.0}, 0.0, 0.0};  // clear of the map boundary
  LocalTask task{Pose{3.0, 2.0, 0.0}, {}};
  SimConfig config = base_config(/*dt=*/0.1, /*max_steps=*/50, /*goal_tolerance=*/0.05);

  StraightPlanner null_planner(0.5);
  EXPECT_NO_THROW(simulate(null_planner, grid, start, task, config, nullptr));

  StraightPlanner traced_planner(0.5);
  std::ostringstream os;
  TraceRecorder rec(os);
  SimResult r = simulate(traced_planner, grid, start, task, config, &rec);
  ASSERT_TRUE(r.success);

  std::string trace = os.str();
  EXPECT_NE(trace.find("\"robot_moved\""), std::string::npos);
  EXPECT_NE(trace.find("\"path_found\""), std::string::npos);
  EXPECT_NE(trace.find("\"planning_finished\""), std::string::npos);
}

// ⑧ reset() is called at episode start: re-running the same instance is deterministic

TEST(LocalSim, ResetMakesRerunOfSameInstanceIdentical) {
  auto grid = open_grid(60);
  RobotState start{Pose{5.0, 5.0, 0.0}, 0.0, 0.0};
  LocalTask task{Pose{8.0, 6.0, 0.0}, {}};
  SimConfig config = base_config(/*dt=*/0.1, /*max_steps=*/60, /*goal_tolerance=*/0.05);
  CountingPlanner planner(/*turn_ticks=*/4);

  SimResult r1 = simulate(planner, grid, start, task, config, nullptr);
  SimResult r2 = simulate(planner, grid, start, task, config, nullptr);

  EXPECT_EQ(r1.status, r2.status);
  EXPECT_EQ(r1.steps, r2.steps);
  EXPECT_DOUBLE_EQ(r1.distance_traveled, r2.distance_traveled);
  ASSERT_EQ(r1.trajectory.size(), r2.trajectory.size());
  for (size_t i = 0; i < r1.trajectory.size(); ++i) {
    EXPECT_DOUBLE_EQ(r1.trajectory[i].x, r2.trajectory[i].x);
    EXPECT_DOUBLE_EQ(r1.trajectory[i].y, r2.trajectory[i].y);
    EXPECT_DOUBLE_EQ(r1.trajectory[i].theta, r2.trajectory[i].theta);
  }
}

// integrate_unicycle: straight branch (omega ~ 0) -------------------------------

TEST(LocalSim, IntegrateUnicycleStraightBranch) {
  Pose p = local_planning::integrate_unicycle(Pose{0.0, 0.0, 0.0}, VelocityCommand{2.0, 0.0}, 0.5);
  EXPECT_DOUBLE_EQ(p.x, 1.0);
  EXPECT_DOUBLE_EQ(p.y, 0.0);
  EXPECT_DOUBLE_EQ(p.theta, 0.0);
}

// integrate_unicycle: exact-arc branch matches the closed form directly --------

TEST(LocalSim, IntegrateUnicycleArcBranchMatchesClosedForm) {
  double v = 1.0, omega = 0.5, dt = 0.2, theta0 = 0.3;
  Pose p = local_planning::integrate_unicycle(Pose{1.0, 2.0, theta0}, VelocityCommand{v, omega}, dt);
  double theta1 = theta0 + omega * dt;
  double expected_x = 1.0 + (v / omega) * (std::sin(theta1) - std::sin(theta0));
  double expected_y = 2.0 - (v / omega) * (std::cos(theta1) - std::cos(theta0));
  EXPECT_NEAR(p.x, expected_x, 1e-12);
  EXPECT_NEAR(p.y, expected_y, 1e-12);
  EXPECT_NEAR(p.theta, local_planning::wrap_to_pi(theta1), 1e-12);
}
