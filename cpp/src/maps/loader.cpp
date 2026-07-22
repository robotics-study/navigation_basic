#include "navigation/maps/loader.hpp"

#include <filesystem>
#include <stdexcept>

#include "navigation/core/yaml.hpp"
#include "navigation/maps/occupancy_grid.hpp"
#include "navigation/maps/pgm.hpp"

namespace navigation::maps {
namespace fs = std::filesystem;
using core::YamlNode;

namespace {

// Resolve a path referenced inside a yaml file relative to that file's directory.
std::string resolve(const std::string& base_file, const std::string& ref) {
  fs::path p(ref);
  if (p.is_absolute()) return fs::weakly_canonical(p).string();
  fs::path dir = fs::path(base_file).parent_path();
  return fs::weakly_canonical(dir / p).string();
}

}  // namespace

std::unique_ptr<core::MapBase> load_map(const std::string& path, unsigned seed, int connectivity) {
  YamlNode root = core::parse_yaml_file(path);
  std::string type = root.at("type").as_string();
  if (type != "occupancy_grid") {
    throw std::runtime_error("load_map: unsupported map type '" + type + "'");
  }

  std::string image = resolve(path, root.at("image").as_string());
  double resolution = root.at("resolution").as_double();
  const YamlNode& origin = root.at("origin");
  double ox = origin.seq.at(0).as_double();
  double oy = origin.seq.at(1).as_double();
  double occupied_thresh = root.at("occupied_thresh").as_double();
  double free_thresh = root.at("free_thresh").as_double();

  PgmImage img = load_pgm(image);
  return std::make_unique<OccupancyGrid2D>(OccupancyGrid2D::from_image(
      img, resolution, ox, oy, occupied_thresh, free_thresh, connectivity, seed));
}

Scenario load_scenario(const std::string& path) {
  YamlNode root = core::parse_yaml_file(path);
  // Multi-agent scenarios (an `agents` field) belong to the multi_agent category,
  // not this single-agent loader; reject them rather than silently ignoring.
  if (root.has("agents")) {
    throw std::runtime_error("load_scenario: multi-agent scenarios are out of scope");
  }
  Scenario sc;
  sc.map_path = resolve(path, root.at("map").as_string());
  const YamlNode& start = root.at("start");
  const YamlNode& goal = root.at("goal");
  sc.start = {start.seq.at(0).as_double(), start.seq.at(1).as_double()};
  sc.goal = {goal.seq.at(0).as_double(), goal.seq.at(1).as_double()};
  // Optional headings (radians, world) — backward compatible; default 0.
  if (root.has("start_theta")) sc.start_theta = root.at("start_theta").as_double();
  if (root.has("goal_theta")) sc.goal_theta = root.at("goal_theta").as_double();
  // Optional reference path (world points), tracking planners only. Backward
  // compatible: absent -> empty vector, same as an unset field elsewhere here.
  if (root.has("reference_path")) {
    for (const YamlNode& pt : root.at("reference_path").seq) {
      sc.reference_path.push_back({pt.seq.at(0).as_double(), pt.seq.at(1).as_double()});
    }
  }
  return sc;
}

}  // namespace navigation::maps
