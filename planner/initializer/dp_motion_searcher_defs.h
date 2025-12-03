#ifndef ONBOARD_PLANNER_INITIALIZER_DP_MOTION_SEARCHER_DEFS_H_
#define ONBOARD_PLANNER_INITIALIZER_DP_MOTION_SEARCHER_DEFS_H_

#include <algorithm>
#include <array>
#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/hash/hash.h"

#include "onboard/math/util.h"
#include "onboard/planner/initializer/motion_graph.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/proto/trajectory_point.pb.h"
namespace qcraft::planner {

// DP search params
// TODO(xiangjun): move to planner config.
// If vehicle speed less than this threshold, we can set a stationary motion
// for it.
inline constexpr double kCanSetToZeroSpeed = 1.0;
// If vehicle speed less than this threshold and search failed, we can set a
// stationary motion for it.
inline constexpr double kSearchFailedCanSetToZeroSpeed = 1.5;
inline constexpr double kCanSetToZeroTrajLength = 4.0;
inline constexpr std::array<double, 9> kAccelerationSamplePoints = {
    -4.0, -3.0, -2.0, -1.0, -0.5, 0.0, 0.5, 1.0, 1.5};
inline constexpr double kDpDiscreteSpeedSampleStep = 3.0;  // m/s
inline constexpr double kDpDiscreteTimeSampleStep = 2.5;   // second.
inline constexpr double kMinSpeedForFinalCost = 3.0;
inline constexpr int kConstVelSampleLayerSizeThreshold = 5;

// Discrete motion samples on geometry node, used to get opt_motion_edge.
struct DpMotionSample {
  int v_discrete;  // discrete speed step
  int t_discrete;  // discrete time step
  DpMotionSample(double v, double t, double traj_horizon)
      : v_discrete(std::max(0, RoundToInt(v / kDpDiscreteSpeedSampleStep))),
        t_discrete(
            std::clamp(RoundToInt(t / kDpDiscreteTimeSampleStep), 0,
                       CeilToInt(traj_horizon / kDpDiscreteTimeSampleStep))) {}

  friend bool operator==(const DpMotionSample& lhs, const DpMotionSample& rhs) {
    return lhs.v_discrete == rhs.v_discrete && lhs.t_discrete == rhs.t_discrete;
  }

  template <typename H>
  friend H AbslHashValue(H h, const DpMotionSample& ms) {
    return H::combine(std::move(h), ms.v_discrete, ms.t_discrete);
  }
};

// Candidate trajectory info.
struct TrajInfo {
  MotionEdgeIndex idx;
  double total_cost;
  std::vector<double> feature_costs;
  std::vector<ApolloTrajectoryPointProto> traj_points;
};

struct BestEdgeInfo {
  MotionEdgeIndex idx;
  double total_cost = 0.0;
  bool is_created_stationary_motion = false;
};

enum class CollisionConfiguration {
  NONE = 0,
  FRONT = 1,  // At least one corner of the considered obstacle is beyond the
              // front bumper of AV right before collision.
  LEFT = 2,   // All corners of the obstacle is behind the front bumper of AV
              // and at least one corner is beyond the rear bumper of AV and on
              // the left of AV right before collision.
  RIGHT = 3,  // All corners of the obstacle is behind the front bumper of AV
              // and at least one corner is beyond the rear bumper of AV and
              // on the right of AV right before collision.
  BACK = 4,   // All corners of the obstacle is behind the rear bumper of AV
              // right before the collision.
};
struct CollisionConfigurationInfo {
  int time_idx = 0;
  CollisionConfiguration collision_config = CollisionConfiguration::NONE;
};

using IgnoreTrajMap =
    absl::flat_hash_map<std::string, CollisionConfigurationInfo>;

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_INITIALIZER_DP_MOTION_SEARCHER_DEFS_H_
