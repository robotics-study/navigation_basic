#include "demo_common.hpp"
#include "navigation/global_planning/sampling/kinodynamic_rrt_star.hpp"

int main(int argc, char** argv) {
  try {
    demo::Args args = demo::parse_args(argc, argv);
    auto params = navigation::core::ParamSet::from_yaml(args.params);
    navigation::global_planning::KinodynamicRrtStarPlanner planner(params);
    // Binds the grid as a SamplingSpace<Point>&; the planner owns its double-integrator
    // dynamics and queries only (x, y) validity, so the sampling demo flow applies.
    return demo::run_sampling(args, params, planner);
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << "\n";
    return 1;
  }
}
