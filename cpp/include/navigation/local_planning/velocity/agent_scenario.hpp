#pragma once

#include <string>
#include <vector>

#include "navigation/local_planning/velocity/agent_sim.hpp"

// Multi-body scenario loader for the velocity-obstacle family.
//
// `maps/loader.hpp`'s `load_scenario`/`Scenario` are single-robot by contract
// (they reject an `agents:` key), so a genuinely different schema -- N
// bodies, each with its own start/goal/radius and an optional scripted
// (non-cooperative) velocity -- gets its own family-owned loader instead of
// extending that one. Depends on the yaml reader + core::types + this
// family's own AgentSpec only; the static grid referenced by `map:` is
// loaded separately by the caller via maps::load_map, exactly like the
// single-robot demo flow does.
namespace navigation::local_planning {

struct AgentScenario {
  std::string map_path;
  std::vector<AgentSpec> agents;
};

AgentScenario load_agent_scenario(const std::string& path);

}  // namespace navigation::local_planning
