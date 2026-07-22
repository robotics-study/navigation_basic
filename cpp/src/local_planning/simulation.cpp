#include "navigation/local_planning/simulation.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>

#include "navigation/local_planning/geometry.hpp"

namespace navigation::local_planning {

core::Pose integrate_unicycle(const core::Pose& pose, const core::VelocityCommand& cmd,
                              double dt) {
  // Below this |omega| the arc radius v/omega blows up numerically before the
  // path shape actually straightens out; a straight-line step at 1e-9 rad/s
  // is indistinguishable from the arc solution to any solver's precision.
  if (std::abs(cmd.omega) < 1e-9) {
    return core::Pose{pose.x + cmd.v * dt * std::cos(pose.theta),
                      pose.y + cmd.v * dt * std::sin(pose.theta), pose.theta};
  }
  double theta_next = pose.theta + cmd.omega * dt;
  double x = pose.x + (cmd.v / cmd.omega) * (std::sin(theta_next) - std::sin(pose.theta));
  double y = pose.y - (cmd.v / cmd.omega) * (std::cos(theta_next) - std::cos(pose.theta));
  return core::Pose{x, y, wrap_to_pi(theta_next)};
}

SimResult simulate(core::LocalPlanner<core::ObstacleQuery>& planner, core::ObstacleQuery& space,
                   const core::RobotState& start, const core::LocalTask& task,
                   const SimConfig& config, core::TraceRecorder* recorder) {
  auto t_start = std::chrono::steady_clock::now();

  planner.reset();
  const core::Footprint footprint{config.footprint_radius};

  SimResult result;
  result.trajectory.push_back(start.pose);
  double min_clearance = space.distance_to_nearest(core::Point{start.pose.x, start.pose.y});
  double distance_traveled = 0.0;

  SimStatus status = SimStatus::TIMEOUT;
  int steps = 0;

  if (space.is_collision(footprint, start.pose)) {
    status = SimStatus::COLLISION;
  } else {
    core::RobotState state = start;
    for (steps = 1; steps <= config.max_steps; ++steps) {
      core::VelocityCommand cmd =
          planner.compute_command(space, state, task, config.control_dt, recorder);
      core::Pose pose = integrate_unicycle(state.pose, cmd, config.control_dt);

      if (recorder) {
        recorder->robot_moved(
            pose, core::TraceRecorder::EventData{{"v", cmd.v}, {"omega", cmd.omega}});
      }

      if (space.is_collision(footprint, pose)) {
        status = SimStatus::COLLISION;
        break;
      }

      min_clearance =
          std::min(min_clearance, space.distance_to_nearest(core::Point{pose.x, pose.y}));
      distance_traveled += std::hypot(pose.x - state.pose.x, pose.y - state.pose.y);
      result.trajectory.push_back(pose);

      if (std::hypot(pose.x - task.goal.x, pose.y - task.goal.y) <= config.goal_tolerance) {
        status = SimStatus::REACHED;
        break;
      }

      if (steps >= config.stall_window) {
        const core::Pose& past =
            result.trajectory[static_cast<size_t>(steps - config.stall_window)];
        if (std::hypot(pose.x - past.x, pose.y - past.y) < config.stall_distance) {
          status = SimStatus::STALLED;
          break;
        }
      }

      state = core::RobotState{pose, cmd.v, cmd.omega};
    }
    if (steps > config.max_steps) steps = config.max_steps;  // loop ran out -> TIMEOUT
  }

  result.status = status;
  result.success = (status == SimStatus::REACHED);
  result.steps = steps;
  result.time_to_goal = steps * config.control_dt;
  result.distance_traveled = distance_traveled;
  result.min_clearance = min_clearance;

  if (recorder) {
    if (result.success) recorder->path_found(result.trajectory);
    double runtime_sec =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();
    std::map<std::string, double> metrics{
        {"time_to_goal", result.time_to_goal},
        {"distance_traveled", result.distance_traveled},
        {"min_clearance", result.min_clearance},
        {"steps", static_cast<double>(result.steps)},
        {"runtime_sec", runtime_sec},
        {"collided", status == SimStatus::COLLISION ? 1.0 : 0.0},
        {"stalled", status == SimStatus::STALLED ? 1.0 : 0.0},
    };
    recorder->planning_finished(result.success, metrics);
  }
  return result;
}

}  // namespace navigation::local_planning
