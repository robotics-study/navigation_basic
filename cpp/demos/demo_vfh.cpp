#include "demo_common.hpp"
#include "navigation/local_planning/reactive/vfh.hpp"

int main(int argc, char** argv) {
  try {
    return demo::run_local(argc, argv, "vfh", [](const navigation::core::ParamSet& p) {
      return navigation::local_planning::VfhPlanner(p);
    });
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << "\n";
    return 1;
  }
}
