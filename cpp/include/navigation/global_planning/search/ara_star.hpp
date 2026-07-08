#pragma once

#include <set>
#include <string>

#include "navigation/core/planner.hpp"

namespace navigation::global_planning {

// ARA* (Likhachev, Gordon & Thrun 2003): Anytime Repairing A*. A sequence of
// weighted-A* searches (f = g + eps*h) with decreasing eps that reuse prior work
// — states improved after expansion go to INCONS instead of back into OPEN, and
// each eps-iteration reopens INCONS union OPEN with recomputed keys. Emits an
// improved, eps-bounded path per iteration; the last (eps -> eps_final) is
// optimal when eps_final == 1.
class AraStar final : public core::DiscretePlanner {
 public:
  explicit AraStar(core::ParamSet params) : core::DiscretePlanner(std::move(params)) {}
  std::string name() const override { return "ara_star"; }
  std::set<core::Capability> required_capabilities() const override {
    return {core::Capability::DISCRETE_SPACE};
  }
  core::PlanResult<core::Cell> plan(core::DiscreteSpace<core::Cell>& space, const core::Cell& start,
                                    const core::Cell& goal, core::TraceRecorder* recorder) override;
};

}  // namespace navigation::global_planning
