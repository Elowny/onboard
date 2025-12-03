#ifndef ONBOARD_PREDICTION_PREDICTION_UTIL_H_
#define ONBOARD_PREDICTION_PREDICTION_UTIL_H_

// IWYU pragma: no_include <google/protobuf/repeated_ptr_field.h>
// IWYU pragma: no_include <boost/container/detail/std_fwd.hpp>

#include <algorithm>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/status/statusor.h"
#include "absl/types/span.h"

#include "onboard/maps/proto/lane_path.pb.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/geometry/segment2d.h"
#include "onboard/math/segment_matcher/segment_matcher_kdtree.h"
#include "onboard/math/vec.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/prediction/container/object_history.h"
#include "onboard/prediction/object_prediction.h"
#include "onboard/prediction/predicted_trajectory.h"
#include "onboard/prediction/prediction_defs.h"
#include "onboard/prediction/proto/prediction_common.pb.h"
#include "onboard/proto/autonomy_state.pb.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/proto/perception/fusion/object.pb.h"
#include "onboard/proto/prediction.pb.h"
#include "onboard/proto/trajectory.pb.h"

namespace qcraft {
namespace prediction {
// Get map elements inside the region box sorted by distance.

template <class T>
std::vector<const T*> GetElementsInBox2d(const std::vector<const T*>& elements,
                                         const Box2d& region_box,
                                         const int max_num) {
  std::vector<const T*> intersected_eles;
  intersected_eles.reserve(elements.size());
  for (const auto* ele : elements) {
#if (defined(__ARM_NEON) && defined(__J5__))
    if (region_box.HasOverlapWithPoints(ele->points_smooth)) {
      intersected_eles.push_back(ele);
    }
#else
    for (int i = 1; i < ele->points_smooth.size(); ++i) {
      const auto& pt = ele->points_smooth[i];
      const auto& prev_pt = ele->points_smooth[i - 1];
      if (region_box.HasOverlap(Segment2d(prev_pt, pt)) ||
          region_box.IsPointIn(pt)) {
        intersected_eles.push_back(ele);
        break;
      }
    }
#endif
  }
  if (intersected_eles.size() <= max_num) {
    return intersected_eles;
  }

  std::vector<std::pair<const T*, double>> dist_pairs;
  dist_pairs.reserve(intersected_eles.size());
  for (const auto* ele : intersected_eles) {
#ifdef __ARM_NEON
    float64x2_t _min_dist = vdupq_n_f64(std::numeric_limits<double>::max());
    float64x2_t _center_x = vdupq_n_f64(region_box.center().x());
    float64x2_t _center_y = vdupq_n_f64(region_box.center().y());
    size_t count = ele->points_smooth.size() >> 1;
    for (size_t i = 0; i < count; ++i) {
      float64x2x2_t _ptx2 = vld2q_f64(&(ele->points_smooth[2 * i].x()));
      float64x2_t _x = vsubq_f64(_ptx2.val[0], _center_x);
      float64x2_t _y = vsubq_f64(_ptx2.val[1], _center_y);
      _x = vmulq_f64(_x, _x);
      _y = vmulq_f64(_y, _y);
      float64x2_t _dist = vsqrtq_f64(vaddq_f64(_x, _y));
      _min_dist = vminq_f64(_min_dist, _dist);
    }
    double min_dist = vminvq_f64(_min_dist);
    if (1 == ele->points_smooth.size() % 2) {
      const auto& pt = ele->points_smooth[ele->points_smooth.size() - 1];
      min_dist = std::min<double>(min_dist, pt.DistanceTo(region_box.center()));
    }
#else

    double min_dist = std::numeric_limits<double>::max();
    for (const auto& pt : ele->points_smooth) {
      min_dist = std::min<double>(min_dist, pt.DistanceTo(region_box.center()));
    }
#endif
    dist_pairs.push_back({ele, min_dist});
  }

  std::sort(
      dist_pairs.begin(), dist_pairs.end(),
      [](const auto& lhs, const auto& rhs) { return lhs.second < rhs.second; });
  intersected_eles.clear();
  for (int i = 0; i < max_num; ++i) {
    intersected_eles.push_back(dist_pairs[i].first);
  }
  return intersected_eles;
}

// Get polygon-like map elements inside the region box sorted by distance.
template <class T>
std::vector<const T*> GetPolygonElementsInBox2d(
    const std::vector<const T*>& elements, const Box2d& region_box,
    const int max_num) {
  std::vector<const T*> intersected_eles;
  intersected_eles.reserve(elements.size());

  for (const auto* ele : elements) {
    for (const auto& pt : ele->polygon_smooth.points()) {
      if (region_box.IsPointIn(pt)) {
        intersected_eles.push_back(ele);
        break;
      }
    }
  }

  if (intersected_eles.size() <= max_num) {
    return intersected_eles;
  }

  std::vector<std::pair<const T*, double>> dist_pairs;
  dist_pairs.reserve(intersected_eles.size());
  for (const auto* ele : intersected_eles) {
    double min_dist = std::numeric_limits<double>::max();
    for (const auto& pt : ele->polygon_smooth.points()) {
      min_dist = std::min<double>(min_dist, pt.DistanceTo(region_box.center()));
    }
    dist_pairs.push_back({ele, min_dist});
  }
  std::sort(
      dist_pairs.begin(), dist_pairs.end(),
      [](const auto& lhs, const auto& rhs) { return lhs.second < rhs.second; });
  intersected_eles.clear();
  for (int i = 0; i < max_num; ++i) {
    intersected_eles.push_back(dist_pairs[i].first);
  }
  return intersected_eles;
}

ObjectProto LerpObjectProto(const ObjectProto& a, const ObjectProto& b,
                            double alpha);

bool AllLanesFoundInSemanticMap(
    const mapping::LanePathProto& lane_path_proto,
    const planner::PlannerSemanticMapManager& semantic_map_manager);

std::vector<ObjectProto> ResampleObjectProtos(
    absl::Span<const ObjectProto> objs, double current_ts, double time_step,
    int max_steps);

// Maybe vulnerable road users
inline bool MaybeVRU(ObjectType type) {
  return (type == OT_PEDESTRIAN || type == OT_MOTORCYCLIST ||
          type == OT_UNKNOWN_MOVABLE || type == OT_UNKNOWN_STATIC ||
          type == OT_CYCLIST);
}

ObjectType GuessType(const ObjectHistory& obj);

// Test if a trajectory proto represents a stationary trajectory.
inline bool IsStationaryTrajectory(const PredictedTrajectoryProto& traj) {
  return traj.type() == PredictionType::PT_STATIONARY;
}
// Test if a trajectory represents a stationary trajectory.
inline bool IsStationaryTrajectory(
    const prediction::PredictedTrajectory& traj) {
  return traj.type() == PredictionType::PT_STATIONARY;
}

// Test if a prediction proto represents a stationary trajectory.
inline bool IsStationaryPrediction(const ObjectPredictionProto& pred) {
  return pred.trajectories().size() == 1 &&
         IsStationaryTrajectory(pred.trajectories(0));
}
// Test if a prediction represents a stationary trajectory.
inline bool IsStationaryPrediction(const prediction::ObjectPrediction& pred) {
  return pred.trajectories().size() == 1 &&
         IsStationaryTrajectory(pred.trajectories()[0]);
}

/* Function: InstantPredictionForNewObject()
 * -------------------------------------------------------------
 * This function takes in an ObjectProto by reference. Information needed can
 * be extracted from the ObjectProto and the predicted trajectory will be
 * written into the ObjectPredictionProto and returned. When the input proto
 * is missing some information, a Status of UNKNOWN/NOT_FOUND will be returned
 * instead.
 */
absl::StatusOr<ObjectPredictionProto> InstantPredictionForNewObject(
    const ObjectProto& object, double prediction_time);

absl::StatusOr<ObjectPrediction> InstantObjectPredictionForNewObject(
    const ObjectProto& object, double prediction_time);

// Truncate trajectory
inline void TruncateTrajectoryBySize(
    int new_size, std::vector<PredictedTrajectoryPoint>* mutable_points) {
  mutable_points->resize(
      std::min(static_cast<int>(mutable_points->size()), new_size));
}

std::unique_ptr<SegmentMatcherKdtree> TrajectoryProtoToSegmentMatcherKdtree(
    const TrajectoryProto& trajectory);

// NOTE(zuowei): Should maintain consistency of `DynamicSwitchPlanTask` in
// Planner.
bool MustReceiveHDMapForPrediction(const AutonomyStateProto& autonomy_state);

std::pair<double, double> QueryDistanceToLeftAndRightAvLaneBoundary(
    const planner::DrivePassage& drive_passage, const double& s,
    const double& l);

bool IsOnlineMapMode(const AutonomyStateProto* autonomy_state);

Box2d GetRegionBox(const Vec2d& pos, const double heading,
                   double detection_region_front,
                   double detection_region_behind, double detection_half_width);

std::vector<double> GetActNetScanBoxInfo(ObjectType type, double speed);

Box2d GetDynamicRegionBox(const Vec2d& ref_position,
                          const ObjectMotionHistory& agent_history,
                          double heading);

std::map<std::string, FeatureScaleConfig> BuildModelScaleParamMap(
    const ModelScaleParam& scale_param);

}  // namespace prediction
}  // namespace qcraft

#endif  // ONBOARD_PREDICTION_PREDICTION_UTIL_H_
