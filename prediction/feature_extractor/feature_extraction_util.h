#ifndef ONBOARD_PREDICTION_FEATURE_EXTRACTOR_FEATURE_EXTRACTION_UTIL_H_
#define ONBOARD_PREDICTION_FEATURE_EXTRACTOR_FEATURE_EXTRACTION_UTIL_H_
#include <algorithm>  // for max, min
#include <vector>     // for vector

#include "onboard/global/buffered_logger.h"  // for BufferedLoggerWrapper
#include "onboard/lite/check.h"              // for QCHECK_GT
#include "onboard/maps/maps_common.h"        // for IntersectionInfo
#include "onboard/math/vec.h"                // for Vec2d
#include "onboard/planner/planner_semantic_map_manager.h"  // for PlannerSemanticMapManager
#include "onboard/planner/router/drive_passage.h"  // for DrivePassage
#include "onboard/prediction/proto/prediction_common.pb.h"  // for PredictedTrajectoryPointProto
#include "onboard/proto/trajectory.pb.h"  // for TrajectoryProto
#include "onboard/proto/vehicle.pb.h"     // for VehicleGeometryParamsProto

namespace qcraft {
namespace prediction {
std::vector<PredictedTrajectoryPointProto> AlignPredictedTrajectoryPoints(
    const std::vector<PredictedTrajectoryPointProto>& prev_pts,
    double prev_time, double cur_time, double new_horizon, double dt);

std::vector<PredictedTrajectoryPointProto>
ConvertTrajectoryProtoToPredictedTrajectoryPoints(
    const TrajectoryProto& trajectory_proto,
    const VehicleGeometryParamsProto& veh_geom_params);

template <typename T>
std::vector<float> OneHotObjectType(T type, int num_class) {
  // Put type into range
  QCHECK_GT(num_class, 0);
  int type_idx =
      std::max(std::min<int>(static_cast<int>(type), num_class - 1), 0);
  std::vector<float> one_hot(num_class, 0.0);
  one_hot[type_idx] = 1.0;
  return one_hot;
}

std::vector<Vec2d> GetExitsOfIntersection(
    const planner::PlannerSemanticMapManager& semantic_map_manager,
    const qcraft::mapping::IntersectionInfo& intersection,
    double reduction_radius);

struct LeftRightLaneBoundaries {
  std::vector<float> left_boundary;
  std::vector<float> right_boundary;
};

LeftRightLaneBoundaries GetLeftRightLaneBoundaries(
    const planner::DrivePassage& drive_passage);
constexpr double kMaxLaneBoundaryOffset = 3.75 / 2.0;  // m

}  // namespace prediction
}  // namespace qcraft

#endif  // ONBOARD_PREDICTION_FEATURE_EXTRACTOR_FEATURE_EXTRACTION_UTIL_H_
