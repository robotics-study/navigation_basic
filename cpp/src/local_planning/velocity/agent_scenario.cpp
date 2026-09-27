#include "navigation/local_planning/velocity/agent_scenario.hpp"

#include <filesystem>

#include "navigation/core/yaml.hpp"

namespace navigation::local_planning {
namespace fs = std::filesystem;
using core::YamlNode;

namespace {

// Resolve a path referenced inside a yaml file relative to that file's
// directory -- duplicated from maps/loader.cpp's `resolve()` rather than
// shared, since algorithm-family code depends on core only, never on the
// maps module (dependency direction: this loader is a peer of maps::load_scenario,
// not a client of it).
std::string resolve(const std::string& base_file, const std::string& ref) {
  fs::path p(ref);
  if (p.is_absolute()) return fs::weakly_canonical(p).string();
  fs::path dir = fs::path(base_file).parent_path();
  return fs::weakly_canonical(dir / p).string();
}

}  // namespace

AgentScenario load_agent_scenario(const std::string& path) {
  YamlNode root = core::parse_yaml_file(path);
  AgentScenario scenario;
  scenario.map_path = resolve(path, root.at("map").as_string());
  for (const YamlNode& entry : root.at("agents").seq) {
    const YamlNode& start_xy = entry.at("start");
    const YamlNode& goal_xy = entry.at("goal");
    double theta = entry.has("theta") ? entry.at("theta").as_double() : 0.0;
    core::RobotState start{
        core::Pose{start_xy.seq.at(0).as_double(), start_xy.seq.at(1).as_double(), theta}, 0.0,
        0.0};
    core::Pose goal{goal_xy.seq.at(0).as_double(), goal_xy.seq.at(1).as_double(), 0.0};
    double radius = entry.at("radius").as_double();
    std::optional<core::Point> scripted_velocity;
    if (entry.has("scripted_velocity")) {
      const YamlNode& sv = entry.at("scripted_velocity");
      scripted_velocity = core::Point{sv.seq.at(0).as_double(), sv.seq.at(1).as_double()};
    }
    scenario.agents.push_back(AgentSpec{start, goal, radius, scripted_velocity});
  }
  return scenario;
}

}  // namespace navigation::local_planning
