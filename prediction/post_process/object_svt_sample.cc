#include "onboard/prediction/post_process/object_svt_sample.h"

#include <cmath>
#include <optional>

namespace qcraft {
namespace prediction {

namespace {
const double kEpsilon = 1e-6;

std::vector<SvtState> Sample(double s_init, double v_init, double t_init,
                             double max_length, double a, double ds) {
  double sample_length = max_length;
  bool stopped = false;
  if (a < -kEpsilon) {
    // Determine whether stop before max_length;
    const double stop_length = v_init * v_init * 0.5 / std::fabs(a);
    if (stop_length < max_length) {
      sample_length = stop_length;
      stopped = true;
    }
  }

  if (a < kEpsilon && std::fabs(v_init) < kEpsilon) {
    // Deceleration or zero acc when v_init is around zero, it's already
    // stopped.
    sample_length = 0.0;
    stopped = true;
  }

  // Approximate a sample size.
  const int sample_size = FloorToInt(sample_length / ds) + 1;
  std::vector<SvtState> states;
  states.reserve(sample_size);
  states.push_back(SvtState({
      .s = s_init,
      .v = v_init,
      .t = t_init,
  }));

  // Deal with const v sampling.
  std::optional<double> inv_a = std::nullopt;
  if (std::fabs(a) > kEpsilon) {
    inv_a = 1.0 / a;
  }

  double sampled_length = 0.0;
  bool reach_sample_length = false;
  while (sampled_length < sample_length) {
    const auto& prev_state = states.back();
    // In case sampled s exceeds sample_length.
    const double remain_length = sample_length - sampled_length;
    if (ds > remain_length) {
      reach_sample_length = true;
      ds = remain_length;
    }
    double in_sqrt = 2.0 * a * ds + prev_state.v * prev_state.v;
    in_sqrt = std::fabs(in_sqrt) > kEpsilon ? in_sqrt : 0.0;
    const double next_state_v = std::sqrt(in_sqrt);
    const double dt = inv_a.has_value()
                          ? (next_state_v - prev_state.v) * (*inv_a)
                          : ds / v_init;
    states.push_back(SvtState({
        .s = prev_state.s + ds,
        .v = next_state_v,
        .t = prev_state.t + dt,
    }));
    sampled_length = states.back().s - states.front().s;
    if (reach_sample_length) {
      break;
    }
  }
  if (stopped) {
    // Fill stationary states till planner time horizon.
    while (states.back().t < planner::kTrajectoryTimeHorizon) {
      const auto& prev_state = states.back();
      states.push_back(SvtState({
          .s = prev_state.s,
          .v = 0.0,
          .t = std::min<double>(prev_state.t + /*dt=*/0.1,
                                planner::kTrajectoryTimeHorizon),
      }));
    }
  }
  return states;
}

}  // namespace

std::vector<SvtState> SampleDp(absl::Span<const SvtNode> nodes,
                               const DpEdgeInfo& edge_info, double ds) {
  const auto& start_node = nodes[edge_info.start_index.value()];
  const auto& max_length = edge_info.length;
  return Sample(start_node.s, start_node.v, start_node.t, max_length,
                edge_info.a, ds);
}

std::vector<SvtState> SampleEdge(absl::Span<const SvtNode> nodes,
                                 const SvtEdge& edge, double ds) {
  const auto& start_node = nodes[edge.start_index.value()];
  const auto& end_node = nodes[edge.end_index.value()];
  const double max_length = end_node.s - start_node.s;
  return Sample(start_node.s, start_node.v, start_node.t, max_length, edge.a,
                ds);
}

}  // namespace prediction
}  // namespace qcraft
