#include "demo_common.hpp"
#include "navigation/local_planning/band/elastic_bands.hpp"

int main(int argc, char** argv) {
  try {
    return demo::run_local(argc, argv, "elastic_bands", [](const navigation::core::ParamSet& p) {
      return navigation::local_planning::ElasticBandsPlanner(p);
    });
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << "\n";
    return 1;
  }
}
