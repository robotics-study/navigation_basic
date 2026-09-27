#include "demo_common.hpp"
#include "navigation/local_planning/velocity/vo.hpp"

int main(int argc, char** argv) {
  try {
    return demo::run_agents(argc, argv, "vo", [](const navigation::core::ParamSet& p) {
      return navigation::local_planning::Vo(p);
    });
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << "\n";
    return 1;
  }
}
