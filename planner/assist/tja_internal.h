#ifndef ONBOARD_PLANNER_ASSIST_TJA_INTERAL_H_
#define ONBOARD_PLANNER_ASSIST_TJA_INTERAL_H_
#include <memory>
#include <vector>

#include "absl/status/statusor.h"
#include "google/protobuf/repeated_ptr_field.h"

#include "onboard/maps/proto/online_semantic_map.pb.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/vec.h"
#include "onboard/planner/assist/tja_state.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft::planner {

absl::StatusOr<std::shared_ptr<const mapping::OnlineSemanticMapProto>>
ActivateOnlineSemanticMap(const PoseProto& pose,
                          const SpacetimeTrajectoryManager& st_traj_mgr,
                          const VehicleGeometryParamsProto& veh_geo_params,
                          const PlannerSemanticMapManager& psmm,
                          const mapping::OnlineSemanticMapProto& origin_map,
                          TjaState* tja_state);

void FillPlannerCenterLine(
    const google::protobuf::RepeatedPtrField<Vec2dProto>& input_center_line,
    std::vector<Vec2d>* real_center_line);

bool ShouldUseTjaOnlineSemanticMap(const PoseProto& pose,
                                   const PlannerSemanticMapManager& psmm,
                                   TjaState* tja_state);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_ASSIST_TJA_INTERAL_H_
