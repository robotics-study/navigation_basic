#include "demo_common.hpp"
#include "navigation/global_planning/search/theta_star.hpp"

int main(int argc, char** argv) {
  try {
    demo::Args args = demo::parse_args(argc, argv);
    auto params = navigation::core::ParamSet::from_yaml(args.params);
    navigation::global_planning::ThetaStarPlanner planner(params);
    return demo::run_discrete(args, params, planner);
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << "\n";
    return 1;
  }
}
