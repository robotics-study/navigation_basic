#include "demo_common.hpp"
#include "navigation/local_planning/predictive/mpc.hpp"

int main(int argc, char** argv) {
  try {
    return demo::run_local(argc, argv, "mpc", [](const navigation::core::ParamSet& p) {
      return navigation::local_planning::MpcPlanner(p);
    });
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << "\n";
    return 1;
  }
}
