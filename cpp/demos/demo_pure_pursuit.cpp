#include "demo_common.hpp"
#include "navigation/local_planning/tracking/pure_pursuit.hpp"

int main(int argc, char** argv) {
  try {
    return demo::run_local(argc, argv, "pure_pursuit", [](const navigation::core::ParamSet& p) {
      return navigation::local_planning::PurePursuitPlanner(p);
    });
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << "\n";
    return 1;
  }
}
