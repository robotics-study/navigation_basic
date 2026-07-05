#include "demo_common.hpp"
#include "nav_study/global_planning/rrt_star.hpp"

int main(int argc, char** argv) {
  try {
    demo::Args args = demo::parse_args(argc, argv);
    auto params = nav_study::core::ParamSet::from_yaml(args.params);
    nav_study::global_planning::RrtStarPlanner planner(params);
    return demo::run_sampling(args, params, planner);
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << "\n";
    return 1;
  }
}
