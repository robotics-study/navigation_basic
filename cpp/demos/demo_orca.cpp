#include "demo_common.hpp"
#include "navigation/local_planning/velocity/orca.hpp"

int main(int argc, char** argv) {
  try {
    return demo::run_agents(argc, argv, "orca", [](const navigation::core::ParamSet& p) {
      return navigation::local_planning::Orca(p);
    });
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << "\n";
    return 1;
  }
}
