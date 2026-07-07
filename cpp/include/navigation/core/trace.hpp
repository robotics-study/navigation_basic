#pragma once

#include <chrono>
#include <map>
#include <ostream>
#include <string>
#include <vector>

#include "navigation/core/params.hpp"
#include "navigation/core/types.hpp"

namespace navigation::core {

// Emits step-by-step trace events as JSON Lines (spec/trace_schema.json). seq
// starts at 0 and increments per event; t is seconds since planning_started.
// A null TraceRecorder* is never dereferenced by planners (hot-path guard at the
// call site), so tracing is zero-cost when off.
class TraceRecorder {
 public:
  explicit TraceRecorder(std::ostream& os);

  void planning_started(const std::string& algorithm, const std::string& map_path,
                        const std::map<std::string, ParamValue>& params);
  void planning_finished(bool success, const std::map<std::string, double>& metrics);

  // `data` (spec/trace_schema.json) carries optional algorithm-specific extra info
  // that viz may read or ignore; an empty map is omitted so unused traces are
  // byte-identical. Kept numeric (like `metrics`) so both language recorders mirror.
  using EventData = std::map<std::string, double>;

  template <class State>
  void node_expanded(const State& s, const EventData& data = {}) {
    ev_state("node_expanded", to_trace(s), nullptr, ptr(data));
  }
  template <class State>
  void node_expanded(const State& s, double cost, const EventData& data = {}) {
    ev_state("node_expanded", to_trace(s), &cost, ptr(data));
  }
  template <class State>
  void sample_drawn(const State& s, const EventData& data = {}) {
    ev_state("sample_drawn", to_trace(s), nullptr, ptr(data));
  }
  template <class State>
  void candidate_evaluated(const State& s, double cost, const EventData& data = {}) {
    ev_state("candidate_evaluated", to_trace(s), &cost, ptr(data));
  }
  // Dynamic replanning (D* Lite): the robot's new executed cell, and a cell newly
  // sensed as blocked (revealed obstacle). No cost field.
  template <class State>
  void robot_moved(const State& s) {
    ev_state("robot_moved", to_trace(s), nullptr, nullptr);
  }
  template <class State>
  void obstacle_revealed(const State& s) {
    ev_state("obstacle_revealed", to_trace(s), nullptr, nullptr);
  }
  template <class State>
  void edge_added(const State& s, const State& parent, const EventData& data = {}) {
    ev_edge("edge_added", to_trace(s), to_trace(parent), nullptr, ptr(data));
  }
  template <class State>
  void edge_added(const State& s, const State& parent, double cost, const EventData& data = {}) {
    ev_edge("edge_added", to_trace(s), to_trace(parent), &cost, ptr(data));
  }
  template <class State>
  void rewire(const State& s, const State& parent, const EventData& data = {}) {
    ev_edge("rewire", to_trace(s), to_trace(parent), nullptr, ptr(data));
  }
  template <class State>
  void path_found(const std::vector<State>& path) {
    std::vector<std::vector<double>> flat;
    flat.reserve(path.size());
    for (const State& s : path) flat.push_back(to_trace(s));
    ev_path(flat);
  }

 private:
  static const EventData* ptr(const EventData& d) { return d.empty() ? nullptr : &d; }
  void ev_state(const char* event, const std::vector<double>& s, const double* cost,
                const EventData* data);
  void ev_edge(const char* event, const std::vector<double>& s, const std::vector<double>& parent,
               const double* cost, const EventData* data);
  void ev_path(const std::vector<std::vector<double>>& path);
  void begin_event(const char* event);
  void end_event();

  std::ostream& os_;
  long long seq_ = 0;
  std::chrono::steady_clock::time_point t0_;
};

}  // namespace navigation::core
