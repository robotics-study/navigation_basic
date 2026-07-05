#include "navigation/core/trace.hpp"

#include <cmath>
#include <cstdio>

namespace navigation::core {
namespace {

// JSON numbers: emit integral values without a decimal point (so cells read as
// [row, col] ints) and others with enough precision to round-trip coordinates.
void write_num(std::ostream& os, double v) {
  if (std::isfinite(v) && std::abs(v) < 9e15 && v == std::floor(v)) {
    os << static_cast<long long>(v);
  } else {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.10g", v);
    os << buf;
  }
}

void write_str(std::ostream& os, const std::string& s) {
  os << '"';
  for (char c : s) {
    switch (c) {
      case '"': os << "\\\""; break;
      case '\\': os << "\\\\"; break;
      case '\n': os << "\\n"; break;
      case '\t': os << "\\t"; break;
      case '\r': os << "\\r"; break;
      default: os << c;
    }
  }
  os << '"';
}

void write_array(std::ostream& os, const std::vector<double>& a) {
  os << '[';
  for (size_t i = 0; i < a.size(); ++i) {
    if (i) os << ',';
    write_num(os, a[i]);
  }
  os << ']';
}

}  // namespace

TraceRecorder::TraceRecorder(std::ostream& os) : os_(os), t0_(std::chrono::steady_clock::now()) {}

void TraceRecorder::begin_event(const char* event) {
  double t = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0_).count();
  os_ << "{\"seq\":" << seq_++ << ",\"t\":";
  write_num(os_, t);
  os_ << ",\"event\":\"" << event << '"';
}

void TraceRecorder::end_event() { os_ << "}\n"; }

void TraceRecorder::planning_started(const std::string& algorithm, const std::string& map_path,
                                     const std::map<std::string, ParamValue>& params) {
  t0_ = std::chrono::steady_clock::now();
  begin_event("planning_started");
  os_ << ",\"algorithm\":";
  write_str(os_, algorithm);
  os_ << ",\"map\":";
  write_str(os_, map_path);
  os_ << ",\"params\":{";
  bool first = true;
  for (const auto& [k, v] : params) {
    if (!first) os_ << ',';
    first = false;
    write_str(os_, k);
    os_ << ':';
    std::visit(
        [&](const auto& val) {
          using T = std::decay_t<decltype(val)>;
          if constexpr (std::is_same_v<T, std::string>) {
            write_str(os_, val);
          } else if constexpr (std::is_same_v<T, bool>) {
            os_ << (val ? "true" : "false");
          } else {
            write_num(os_, static_cast<double>(val));
          }
        },
        v);
  }
  os_ << '}';
  end_event();
}

void TraceRecorder::planning_finished(bool success, const std::map<std::string, double>& metrics) {
  begin_event("planning_finished");
  os_ << ",\"success\":" << (success ? "true" : "false") << ",\"metrics\":{";
  bool first = true;
  for (const auto& [k, v] : metrics) {
    if (!first) os_ << ',';
    first = false;
    write_str(os_, k);
    os_ << ':';
    write_num(os_, v);
  }
  os_ << '}';
  end_event();
}

void TraceRecorder::ev_state(const char* event, const std::vector<double>& s, const double* cost) {
  begin_event(event);
  os_ << ",\"state\":";
  write_array(os_, s);
  if (cost) {
    os_ << ",\"cost\":";
    write_num(os_, *cost);
  }
  end_event();
}

void TraceRecorder::ev_edge(const char* event, const std::vector<double>& s,
                            const std::vector<double>& parent, const double* cost) {
  begin_event(event);
  os_ << ",\"state\":";
  write_array(os_, s);
  os_ << ",\"parent\":";
  write_array(os_, parent);
  if (cost) {
    os_ << ",\"cost\":";
    write_num(os_, *cost);
  }
  end_event();
}

void TraceRecorder::ev_path(const std::vector<std::vector<double>>& path) {
  begin_event("path_found");
  os_ << ",\"path\":[";
  for (size_t i = 0; i < path.size(); ++i) {
    if (i) os_ << ',';
    write_array(os_, path[i]);
  }
  os_ << ']';
  end_event();
}

}  // namespace navigation::core
