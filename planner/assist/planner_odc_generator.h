
#ifndef ONBOARD_PLANNER_ASSIST_PLANNER_ODC_GENERATOR_H_
#define ONBOARD_PLANNER_ASSIST_PLANNER_ODC_GENERATOR_H_

#include <optional>

#include "absl/status/statusor.h"

#include "onboard/maps/lane_path.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/router/route_manager_output.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft::planner {

struct OdcGeneratorOutput {
  std::optional<double> current_lane_width = std::nullopt;
  std::optional<double> current_lane_length = std::nullopt;
  std::optional<double> current_lane_average_curvature_radius = std::nullopt;
  std::optional<bool> is_av_overlap_boundary = std::nullopt;
  std::optional<bool> is_valid_both_side_boundary = std::nullopt;
  std::optional<bool> is_av_in_emergency_lane = std::nullopt;
  std::optional<bool> lane_path_lost = std::nullopt;
  bool is_acc_engage_only = false;
};

struct OdcGeneratorInput {
  const PlannerSemanticMapManager* psmm = nullptr;
  const PoseProto* pose = nullptr;
  const VehicleGeometryParamsProto* vehicle_geom = nullptr;
  const mapping::LanePath* target_lane_path = nullptr;
  bool is_lane_changing = false;
  bool is_acc_engage_only = false;
};

double CalculateLccMinRequiredLaneLength(const PoseProto& pose);

absl::StatusOr<OdcGeneratorOutput> GeneratePlannerOdcByPose(
    const OdcGeneratorInput& input);

absl::StatusOr<OdcGeneratorOutput> GeneratePlannerOdcByLanePath(
    const OdcGeneratorInput& input);

struct RouteRelatedOdc {
  std::optional<double> distance_to_toll = std::nullopt;
  std::optional<double> distance_to_traffic_light = std::nullopt;
};

absl::StatusOr<RouteRelatedOdc> GeneratePlannerOdcByRoute(
    const PlannerSemanticMapManager* psmm,
    const RouteManagerOutput* route_mgr_output);
}  // namespace qcraft::planner
#endif
