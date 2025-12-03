#ifndef ONBOARD_PLANNER_SPEED_DECIDER_INTERACTION_UTIL_H_
#define ONBOARD_PLANNER_SPEED_DECIDER_INTERACTION_UTIL_H_

#include <vector>

#include "onboard/planner/common/discretized_path.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/object/spacetime_object_trajectory.h"
#include "onboard/planner/speed/overlap_info.h"
#include "onboard/planner/speed/proto/speed_finder.pb.h"
#include "onboard/planner/speed/speed_vector.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace planner {

struct TtcInfo {
  double ttc_upper_limit = 0.0;
  double ttc_lower_limit = 0.0;
};

TtcInfo GetAvOverlapTtcInfo(const std::vector<OverlapInfo>& overlap_infos,
                            const StOverlapMetaProto& overlap_meta,
                            const DiscretizedPath& path,
                            const SpeedVector& preliminary_speed);

TtcInfo GetObjectOverlapTtcInfo(const OverlapInfo& fo_info,
                                const SpacetimeObjectTrajectory& spacetime_obj);

bool IsAvInObjectFov(const PathPoint& current_path_point,
                     const PlannerObject& planner_object,
                     const VehicleGeometryParamsProto& vehicle_geo_params);

bool IsAvCompletelyInObjectFov(
    const PathPoint& current_path_point, const PlannerObject& planner_object,
    const VehicleGeometryParamsProto& vehicle_geo_params);

bool IsObjectInAvFov(const PathPoint& current_path_point,
                     const PlannerObject& planner_object);

double CalcObjectYieldingTime(const std::vector<OverlapInfo>& overlap_infos,
                              const StOverlapMetaProto& overlap_meta,
                              const DiscretizedPath& path,
                              const SpeedVector& preliminary_speed);

bool IsParallelMerging(const StOverlapMetaProto& overlap_meta);

bool HasYieldingIntentionToFrontAv(
    const PathPoint& current_path_point, const PlannerObject& planner_object,
    const VehicleGeometryParamsProto& vehicle_geo_params,
    const StOverlapMetaProto& overlap_meta,
    const SpeedVector& preliminary_speed, double first_overlap_time);

bool IsAggressiveLeading(const SpeedVector& preliminary_speed,
                         double first_overlap_time);

}  // namespace planner
}  // namespace qcraft
#endif  // ONBOARD_PLANNER_SPEED_DECIDER_INTERACTION_UTIL_H_
