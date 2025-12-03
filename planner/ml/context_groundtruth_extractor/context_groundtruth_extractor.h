#ifndef ONBOARD_PLANNER_ML_CONTEXT_GROUND_TRUTH_EXTRACTOR_CONTEXT_GROUND_TRUTH_EXTRACTOR_H_  // NOLINT
#define ONBOARD_PLANNER_ML_CONTEXT_GROUND_TRUTH_EXTRACTOR_CONTEXT_GROUND_TRUTH_EXTRACTOR_H_  // NOLINT

#include "onboard/math/vec.h"  // for Vec2d
#include "onboard/planner/ml/context_feature_extractor/context_feature.h"  // for ContextFeature
#include "onboard/planner/ml/context_groundtruth_extractor/proto/context_groundtruth.pb.h"  // for TrajectoryGroundTruth
#include "onboard/proto/prediction.pb.h"  // for ObjectPredictionProto
#include "onboard/proto/trajectory.pb.h"  // for TrajectoryProto
#include "onboard/proto/vehicle.pb.h"     // for VehicleGeometryParamsProto

namespace qcraft {
namespace planner {
namespace ml {

struct ContextGroundTruthExtractionInput {
  const ContextFeature* context_feature = nullptr;
  const TrajectoryProto* log_av_trajectory = nullptr;
  const ObjectsPredictionProto* log_prediction = nullptr;
  const VehicleGeometryParamsProto* veh_geom_params = nullptr;
};

TrajectoryGroundTruth ExtractAVTrajectoryGroundTruth(
    const TrajectoryProto& trajectory_proto, double cur_ts,
    const Vec2d& ref_pos, double rot_rad);

TrajectoryGroundTruth ExtractObjectTrajectoryGroundTruth(
    const ObjectPredictionProto& object_prediction_proto, double cur_ts,
    const Vec2d& ref_pos, double rot_rad);

ContextGroundTruth ExtractContextGroundTruth(
    const ContextGroundTruthExtractionInput& input);

}  // namespace ml
}  // namespace planner
}  // namespace qcraft

// NOLINTNEXTLINE
#endif  // ONBOARD_PLANNER_ML_CONTEXT_GROUND_TRUTH_EXTRACTOR_CONTEXT_GROUND_TRUTH_EXTRACTOR_H_
