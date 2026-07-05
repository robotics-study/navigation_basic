#include "demo_common.hpp"
#include "nav_study/global_planning/astar.hpp"

int main(int argc, char** argv) {
  try {
    demo::Args args = demo::parse_args(argc, argv);
    auto params = nav_study::core::ParamSet::from_yaml(args.params);
    nav_study::global_planning::AstarPlanner planner(params);
    return demo::run_discrete(args, params, planner);
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << "\n";
    return 1;
  }
}
