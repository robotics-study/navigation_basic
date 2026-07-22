#include "navigation/local_planning/reactive/vfh.hpp"

#include <algorithm>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <utility>

#include "navigation/local_planning/geometry.hpp"
#include "navigation/local_planning/reactive/steering.hpp"

namespace navigation::local_planning {

namespace {

// VFH bins bearings into sector 0..n-1 over a full turn, so it needs [0, 2pi)
// rather than the (-pi, pi] range `wrap_to_pi` (shared with PF/PP) provides --
// kept local since no other reactive/tracking planner needs a 2pi range.
double wrap_2pi(double angle) {
  double wrapped = std::fmod(angle, 2.0 * M_PI);
  if (wrapped < 0.0) wrapped += 2.0 * M_PI;
  return wrapped;
}

}  // namespace

VfhPlanner::VfhPlanner(core::ParamSet params)
    : core::ObstacleLocalPlanner(std::move(params)),
      num_sectors_(params_.get_int("num_sectors")),
      window_radius_(params_.get_float("window_radius")),
      threshold_(params_.get_float("threshold")),
      smoothing_window_(params_.get_int("smoothing_window")),
      wide_valley_sectors_(params_.get_int("wide_valley_sectors")),
      h_m_(params_.get_float("h_m")),
      k_omega_(params_.get_float("k_omega")),
      max_speed_(params_.get_float("max_speed")),
      max_omega_(params_.get_float("max_omega")),
      delta_(2.0 * M_PI / static_cast<double>(num_sectors_)) {
  // param_schema.json has no "must be odd" constraint, so this is enforced
  // here at construction (load time, not the compute_command hot path) -- an
  // even window has no unambiguous center sector for the moving average.
  if (smoothing_window_ % 2 == 0) {
    throw std::runtime_error("param error: 'smoothing_window' must be odd, got " +
                             std::to_string(smoothing_window_));
  }
}

int VfhPlanner::sector_of(double angle) const {
  int k = static_cast<int>(wrap_2pi(angle) / delta_);
  return std::min(k, num_sectors_ - 1);  // fp guard: angle ~ 2pi rounds up to n
}

double VfhPlanner::sector_center(double index) const {
  // `index` need not be reduced mod n first -- wrap_to_pi normalizes any angle
  // congruent mod 2*pi to the same (-pi, pi] value.
  return wrap_to_pi((index + 0.5) * delta_);
}

std::vector<double> VfhPlanner::build_histogram(core::ObstacleQuery& space, double x, double y) const {
  std::vector<double> h(static_cast<size_t>(num_sectors_), 0.0);
  for (const core::Point& o : space.occupied_within(core::Point{x, y}, window_radius_)) {
    const double beta = std::atan2(o.y - y, o.x - x);
    const int k = sector_of(beta);
    const double d = std::hypot(o.x - x, o.y - y);
    // c^2*(a - b*d) with a=1, b=1/window_radius, c=1 (Borenstein & Koren 1991
    // eq. 5) for a known/static map: normalized so d=0 -> m=1 and
    // d=window_radius -> m=0, near obstacles dominate the vote.
    const double m = (1.0 - d / window_radius_) * (1.0 - d / window_radius_);
    h[static_cast<size_t>(k)] += m;
  }
  return h;
}

std::vector<double> VfhPlanner::smooth(const std::vector<double>& h) const {
  const int n = num_sectors_;
  const int half = smoothing_window_ / 2;
  std::vector<double> out(static_cast<size_t>(n), 0.0);
  for (int k = 0; k < n; ++k) {
    double total = 0.0;
    for (int j = -half; j <= half; ++j) {
      const int idx = ((k + j) % n + n) % n;
      total += h[static_cast<size_t>(idx)];
    }
    out[static_cast<size_t>(k)] = total / static_cast<double>(smoothing_window_);
  }
  return out;
}

std::vector<VfhPlanner::Valley> VfhPlanner::find_valleys(const std::vector<bool>& below) const {
  const int n = num_sectors_;
  if (std::all_of(below.begin(), below.end(), [](bool b) { return b; })) {
    return {Valley{0, n - 1, n}};
  }
  if (std::none_of(below.begin(), below.end(), [](bool b) { return b; })) {
    return {};
  }
  // Rotate the circular scan to start right after a known-False sector, so a
  // single linear pass never has to special-case a run that wraps n-1->0.
  int cut = 0;
  for (int i = 0; i < n; ++i) {
    if (!below[static_cast<size_t>(i)]) {
      cut = i;
      break;
    }
  }
  std::vector<Valley> valleys;
  std::optional<int> run_start;
  int run_end = -1;
  for (int i = 0; i < n; ++i) {
    const int idx = (cut + 1 + i) % n;
    if (below[static_cast<size_t>(idx)]) {
      if (!run_start.has_value()) run_start = idx;
      run_end = idx;
    } else if (run_start.has_value()) {
      const int width = ((run_end - *run_start) % n + n) % n + 1;
      valleys.push_back(Valley{*run_start, run_end, width});
      run_start.reset();
    }
  }
  return valleys;
}

bool VfhPlanner::valley_contains(const Valley& v, int k, int n) {
  return ((k - v.start) % n + n) % n < v.width;
}

int VfhPlanner::circular_dist(int a, int b, int n) {
  const int fwd = ((a - b) % n + n) % n;
  const int bwd = ((b - a) % n + n) % n;
  return std::min(fwd, bwd);
}

int VfhPlanner::nearest_sector_gap(const Valley& v, int k_target) const {
  // Selection key (Borenstein & Koren 1991 §IV): the valley whose nearest
  // BORDER sits closest to k_target in sector count -- not the valley whose
  // resulting steering direction happens to have the smallest angle to the
  // goal. The two differ for a valley classified narrow only because its
  // width fell just under wide_valley_sectors: its "steer to center" rule can
  // aim far from the goal even though its border sits right next to
  // k_target, while a true one-sector sliver directly on the goal bearing
  // would otherwise out-rank it by angle alone despite being farther (and a
  // worse gap to thread).
  if (valley_contains(v, k_target, num_sectors_)) return 0;
  return std::min(circular_dist(v.start, k_target, num_sectors_),
                  circular_dist(v.end, k_target, num_sectors_));
}

double VfhPlanner::candidate_direction(const Valley& v, int k_target, double goal_dir) const {
  const int n = num_sectors_;
  // Half the wide-valley threshold also doubles as the minimum border
  // standoff a target bearing needs before "steer straight at the goal" is
  // trusted: a target sector right next to a blocked neighbor is one
  // position update away from being outside the valley altogether (the
  // obstacle border does not move with the robot's discretization), so it
  // gets the same border-hugging treatment as a target outside the valley.
  const int margin = std::min(wide_valley_sectors_ / 2, (v.width - 1) / 2);
  if (valley_contains(v, k_target, n) &&
      std::min(circular_dist(v.start, k_target, n), circular_dist(v.end, k_target, n)) >= margin) {
    return goal_dir;
  }
  if (v.width >= wide_valley_sectors_) {
    // Wide (Borenstein & Koren 1991 §IV): don't aim for the far-away full
    // width center -- steer just inside the border nearest the goal bearing,
    // by half the wide-valley threshold, so the path hugs the opening
    // closest to the target instead of detouring through the valley's
    // middle.
    const int offset = wide_valley_sectors_ / 2;
    const int idx = circular_dist(v.start, k_target, n) <= circular_dist(v.end, k_target, n)
                        ? v.start + offset
                        : v.end - offset;
    return sector_center(static_cast<double>(idx));
  }
  // Narrow: aim for the valley's own center.
  const double center_idx = v.start + (v.width - 1) / 2.0;
  return sector_center(center_idx);
}

core::VelocityCommand VfhPlanner::compute_command(core::ObstacleQuery& space, const core::RobotState& state,
                                           const core::LocalTask& task, double /*dt*/,
                                           core::TraceRecorder* recorder) {
  const double x = state.pose.x, y = state.pose.y, theta = state.pose.theta;
  std::vector<double> h = build_histogram(space, x, y);
  std::vector<double> smoothed = smooth(h);
  std::vector<bool> below(static_cast<size_t>(num_sectors_));
  for (int k = 0; k < num_sectors_; ++k) {
    below[static_cast<size_t>(k)] = smoothed[static_cast<size_t>(k)] < threshold_;
  }
  std::vector<Valley> valleys = find_valleys(below);
  // A run narrower than the smoothing kernel is finer than the histogram's
  // own resolution (the moving average can't distinguish it from smoothing
  // ripple around the threshold), so it is discarded as noise rather than
  // trusted as a real gap -- unless it is literally the only opening, in
  // which case reporting it is still better than declaring the sectors
  // blocked outright.
  std::vector<Valley> wide_enough;
  for (const Valley& v : valleys) {
    if (v.width >= smoothing_window_) wide_enough.push_back(v);
  }
  if (!wide_enough.empty()) valleys = wide_enough;

  const double goal_dir = std::atan2(task.goal.y - y, task.goal.x - x);

  if (valleys.empty()) {
    // Every sector is at/above threshold: no admissible heading exists.
    // heading_command's cos-gate wouldn't fire on its own (theta_err=0 is
    // possible), so this stops the robot explicitly and lets the
    // simulator's stall detector end the episode.
    if (recorder) {
      recorder->histogram_updated(
          state.pose, smoothed,
          core::TraceRecorder::EventData{{"threshold", threshold_},
                                         {"target_direction", goal_dir},
                                         {"selected_direction", goal_dir}});
    }
    return core::VelocityCommand{0.0, 0.0};
  }

  const int k_target = sector_of(goal_dir);
  std::vector<double> candidates;
  std::vector<double> costs;
  std::vector<int> gaps;
  candidates.reserve(valleys.size());
  costs.reserve(valleys.size());
  gaps.reserve(valleys.size());
  for (const Valley& v : valleys) {
    const double d = candidate_direction(v, k_target, goal_dir);
    candidates.push_back(d);
    costs.push_back(std::abs(wrap_to_pi(d - goal_dir)));
    gaps.push_back(nearest_sector_gap(v, k_target));
  }
  // Tie-break equally-near valleys by width (wider = more clearance to work
  // with) rather than by raw scan order -- two valleys can sit at the same
  // sector distance from k_target while one is a much safer, roomier gap.
  size_t selected = 0;
  for (size_t i = 1; i < valleys.size(); ++i) {
    const bool better = gaps[i] < gaps[selected] ||
                        (gaps[i] == gaps[selected] && valleys[i].width > valleys[selected].width);
    if (better) selected = i;
  }
  const double theta_sel = candidates[selected];

  if (recorder) {
    recorder->histogram_updated(
        state.pose, smoothed,
        core::TraceRecorder::EventData{
            {"threshold", threshold_}, {"target_direction", goal_dir}, {"selected_direction", theta_sel}});
    for (size_t i = 0; i < candidates.size(); ++i) {
      const double direction = candidates[i];
      const core::Point probe{x + window_radius_ * std::cos(direction),
                              y + window_radius_ * std::sin(direction)};
      recorder->candidate_evaluated(
          probe, costs[i],
          core::TraceRecorder::EventData{{"direction", direction},
                                         {"selected", i == selected ? 1.0 : 0.0}});
    }
  }

  const double h_sel = smoothed[static_cast<size_t>(sector_of(theta_sel))];
  const double v_eff = max_speed_ * (1.0 - std::min(h_sel, h_m_) / h_m_);
  return heading_command(wrap_to_pi(theta_sel - theta), k_omega_, v_eff, max_omega_);
}

}  // namespace navigation::local_planning
