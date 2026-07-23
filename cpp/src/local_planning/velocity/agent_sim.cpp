#include "navigation/local_planning/velocity/agent_sim.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>

namespace navigation::local_planning {

namespace {

double pairwise_min_clearance(const std::vector<core::RobotState>& states,
                              const std::vector<AgentSpec>& specs) {
  double best = std::numeric_limits<double>::infinity();
  size_t n = states.size();
  for (size_t i = 0; i < n; ++i) {
    for (size_t j = i + 1; j < n; ++j) {
      double gap = std::hypot(states[i].pose.x - states[j].pose.x,
                              states[i].pose.y - states[j].pose.y) -
                   specs[i].radius - specs[j].radius;
      if (gap < best) best = gap;
    }
  }
  return best;
}

}  // namespace

std::vector<AgentResult> simulate_agents(const std::vector<VelocityObstaclePlanner*>& planners,
                                         const std::vector<AgentSpec>& specs,
                                         core::ObstacleQuery& space, const SimConfig& config,
                                         core::TraceRecorder* recorder) {
  const size_t n = specs.size();
  const core::Footprint footprint{config.footprint_radius};
  std::vector<core::RobotState> states;
  states.reserve(n);
  for (const AgentSpec& spec : specs) states.push_back(spec.start);
  std::vector<core::Point> world_vel(n, core::Point{0.0, 0.0});
  std::vector<std::vector<core::Pose>> trajectories(n);
  for (size_t k = 0; k < n; ++k) trajectories[k].push_back(specs[k].start.pose);
  std::vector<bool> reached(n, false);
  std::vector<size_t> planner_indices;
  for (size_t k = 0; k < n; ++k) {
    if (!specs[k].scripted_velocity.has_value()) planner_indices.push_back(k);
  }

  double min_pair_clearance = pairwise_min_clearance(states, specs);
  SimStatus terminal = SimStatus::TIMEOUT;
  int steps = config.max_steps;

  for (int step = 1; step <= config.max_steps; ++step) {
    // 1) snapshot every body before this tick's motion.
    std::vector<DynamicObstacle> snapshot;
    snapshot.reserve(n);
    for (size_t k = 0; k < n; ++k) {
      snapshot.push_back(DynamicObstacle{
          core::Point{states[k].pose.x, states[k].pose.y}, world_vel[k], specs[k].radius});
    }
    // 2) compute every command against that fixed snapshot, index order.
    std::vector<std::optional<core::VelocityCommand>> commands(n);
    for (size_t k : planner_indices) {
      std::vector<DynamicObstacle> neighbors;
      neighbors.reserve(n > 0 ? n - 1 : 0);
      for (size_t j = 0; j < n; ++j) {
        if (j != k) neighbors.push_back(snapshot[j]);
      }
      core::LocalTask task{specs[k].goal, {}};
      VelocityObstaclePlanner* planner = planners[k];
      assert(planner != nullptr && "agent has no planner but is not scripted");
      core::TraceRecorder* rec = (k == 0) ? recorder : nullptr;
      commands[k] =
          planner->command_with_neighbors(space, states[k], task, neighbors, config.control_dt, rec);
    }
    // 3) integrate every body only after every command is known.
    std::vector<core::RobotState> new_states = states;
    for (size_t k = 0; k < n; ++k) {
      if (specs[k].scripted_velocity.has_value()) {
        core::Point v = *specs[k].scripted_velocity;
        double x = states[k].pose.x, y = states[k].pose.y;
        core::Pose new_pose{x + v.x * config.control_dt, y + v.y * config.control_dt,
                            std::atan2(v.y, v.x)};
        world_vel[k] = v;
        new_states[k] = core::RobotState{new_pose, std::hypot(v.x, v.y), 0.0};
      } else {
        const core::VelocityCommand& cmd = *commands[k];
        core::Pose new_pose = integrate_unicycle(states[k].pose, cmd, config.control_dt);
        world_vel[k] = core::Point{cmd.v * std::cos(new_pose.theta), cmd.v * std::sin(new_pose.theta)};
        new_states[k] = core::RobotState{new_pose, cmd.v, cmd.omega};
      }
    }
    states = new_states;
    // 4) trace + trajectory bookkeeping for every body.
    for (size_t k = 0; k < n; ++k) {
      trajectories[k].push_back(states[k].pose);
      if (recorder != nullptr) {
        recorder->robot_moved(
            states[k].pose,
            core::TraceRecorder::EventData{{"v", states[k].v}, {"omega", states[k].omega}},
            static_cast<int>(k));
      }
    }
    // 5) termination judgement.
    double tick_clearance = pairwise_min_clearance(states, specs);
    min_pair_clearance = std::min(min_pair_clearance, tick_clearance);
    bool collided = tick_clearance < 0.0;
    if (!collided) {
      for (size_t k = 0; k < n; ++k) {
        if (space.is_collision(footprint, states[k].pose)) {
          collided = true;
          break;
        }
      }
    }
    if (collided) {
      terminal = SimStatus::COLLISION;
      steps = step;
      break;
    }
    for (size_t k : planner_indices) {
      if (!reached[k] &&
          std::hypot(states[k].pose.x - specs[k].goal.x, states[k].pose.y - specs[k].goal.y) <=
              config.goal_tolerance) {
        reached[k] = true;
      }
    }
    bool all_reached =
        std::all_of(planner_indices.begin(), planner_indices.end(), [&](size_t k) { return reached[k]; });
    if (all_reached) {
      terminal = SimStatus::REACHED;
      steps = step;
      break;
    }
    std::vector<size_t> still_active;
    for (size_t k : planner_indices) {
      if (!reached[k]) still_active.push_back(k);
    }
    if (!still_active.empty() && step >= config.stall_window) {
      bool all_stalled = true;
      for (size_t k : still_active) {
        const core::Pose& past = trajectories[k][static_cast<size_t>(step - config.stall_window)];
        double moved = std::hypot(states[k].pose.x - past.x, states[k].pose.y - past.y);
        if (moved >= config.stall_distance) {
          all_stalled = false;
          break;
        }
      }
      if (all_stalled) {
        terminal = SimStatus::STALLED;
        steps = step;
        break;
      }
    }
    steps = step;
  }

  std::vector<AgentResult> results;
  results.reserve(n);
  for (size_t k = 0; k < n; ++k) {
    results.push_back(AgentResult{reached[k] ? SimStatus::REACHED : terminal, steps,
                                  trajectories[k], min_pair_clearance});
  }
  return results;
}

}  // namespace navigation::local_planning
