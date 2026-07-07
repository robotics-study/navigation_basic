#pragma once

#include <set>
#include <string>

#include "navigation/core/planner.hpp"

namespace navigation::global_planning {

// LQR-RRT* (Perez, Platt, Konidaris, Kaelbling & Lozano-Pérez 2012): RRT* whose
// extension heuristics — the nearest-neighbour distance metric and the steering
// primitive — are DERIVED automatically from a Linear-Quadratic Regulator instead
// of designed by hand. The dynamics are linearised and a quadratic cost
// J = integral(x^T Q x + u^T R u) dt is chosen; the LQR solution then supplies
//   * the metric dist(a,b) = (a-b)^T S (a-b), S the Riccati cost-to-go matrix, and
//   * steering u = -K (x - ref), K = (R + B^T P B)^-1 B^T P A, integrated forward.
// It sits between geometric RRT* (2011, straight edges) and kinodynamic RRT*
// (2013, exact fixed-final-state OBVP): the LQR gives a cheap, general *feedback*
// extension at the price of being asymptotic rather than an exact two-point solve.
//
// This planner OWNS the same 2D double integrator as this repo's kinodynamic RRT*
// (state (x, y, vx, vy), control = acceleration) so the two compare on one
// benchmark, and depends only on the SamplingSpace capability, queried on the
// (x, y) projection for collision checking. S and K are state-independent for this
// LTI system, solved once via a 2x2 discrete Riccati (DARE) iteration. Nodes are
// lifted to REST states so the LQR feedback regulates onto them exactly (the closed
// loop A-BK is Hurwitz), keeping every stored edge a real, feasible trajectory and
// the RRT* rewiring exact.
class LqrRrtStarPlanner final : public core::SamplingPlanner {
 public:
  explicit LqrRrtStarPlanner(core::ParamSet params)
      : core::SamplingPlanner(std::move(params)) {}
  std::string name() const override { return "lqr_rrt_star"; }
  std::set<core::Capability> required_capabilities() const override {
    return {core::Capability::SAMPLING_SPACE};
  }
  core::PlanResult<core::Point> plan(core::SamplingSpace<core::Point>& space,
                                     const core::Point& start, const core::Point& goal,
                                     core::TraceRecorder* recorder) override;
};

}  // namespace navigation::global_planning
