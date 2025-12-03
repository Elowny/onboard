#ifndef ONBOARD_PLANNER_ROUTER_NOA_NOA_MAIN_LOOP_H_
#define ONBOARD_PLANNER_ROUTER_NOA_NOA_MAIN_LOOP_H_

#include <stdint.h>

#include <memory>
#include <optional>
#include <string>

#include "absl/status/status.h"
#include "absl/status/statusor.h"

#include "common/proto/drive_mission.pb.h"

#include "onboard/maps/v2/semantic_map_listener.h"
#include "onboard/maps/v2/semantic_map_spatial_index.h"
#include "onboard/math/coordinate_converter.h"
#include "onboard/math/vec.h"
#include "onboard/planner/router/proto/route_manager_output.pb.h"
#include "onboard/planner/router/proto/route_params.pb.h"
#include "onboard/planner/router/route_manager_state.h"
#include "onboard/planner/router/route_module_input.h"
#include "onboard/planner/router/thirdparty/external_route_manager.h"
#include "onboard/proto/adasis.pb.h"
#include "onboard/proto/localization.pb.h"

namespace qcraft::planner::route::noa {

// route flags/constexpr
struct NoaStartupConfig {
  bool route_snapshot_sim_mode = false;
  bool route_trunc_mpp_by_nav = false;
  bool route_use_sdroute = false;
  double route_sd_max_project_distance = 100.0;
  std::string DebugString() const;
};
struct NoaRouteNavInput {
  const mapping::v2::SemanticMapSpatialIndex* smm_index;
  const RouteParamProto* route_param_proto;
  std::shared_ptr<const MppSectionsProto> mpp;
  std::shared_ptr<const LocalizationTransformProto> localization_transform;
  Vec2d av_global2d;
  std::optional<double> heading;
  int64_t update_id;
  CoordinateConverter cc;
  double av_speed;
  RestrictProto restrict_proto;  // Avoid lanes proto
};

struct NoaStatus {
  absl::Status route_nav_status;
  absl::Status sd_route_status;
};

struct IncrementalNoaOutput {
  ExternalRouteManager* external_route_manager;     // In&Out
  RouteManagerState* rms;                           // In&Out
  RouteManagerOutputProto* route_mgr_output_proto;  // Out
};

absl::StatusOr<RouteManagerOutputProto> ComputeNoaRouteNav(
    const NoaRouteNavInput& noa_route_nav_input, bool route_trunc_mpp_by_nav,
    /*In&Out*/ RouteManagerState* rms);

absl::Status IncremantalUpdateSdRoute(
    ExternalRouteManager* external_route_manager, const Vec2d& av_global2d,
    double route_sd_max_project_distance);

/**
 * @brief
 * (1) ComputeRouteNav( based on lane graph )
 * (2) Send to third party routing request (Hacker mode) or use sd route.
 * (3) Update route
 * Both 2 and 3 are not a real Requirements, only hacker for our HMI demo.
 */
NoaStatus NoaIncrementalUpdateLoop(
    const RoutingInput& routing_input,
    const std::shared_ptr<mapping::v2::SemanticMapSpatialIndexListenerAsnyc>&
        smm_listener,
    const RouteParamProto& route_param_proto,
    NoaStartupConfig noa_constant_input, const Vec2d& av_global2d,
    const std::optional<double>& heading, RestrictProto restrict_proto,
    IncrementalNoaOutput* incremental_noa_output);

absl::Status AddSdRouteToExternalRouteManager(
    const RoutingInput& routing_input, bool route_snapshot_sim_mode,
    bool route_use_sdroute, ExternalRouteManager* external_route_manager);
}  // namespace qcraft::planner::route::noa

#endif  // ONBOARD_PLANNER_ROUTER_NOA_NOA_MAIN_LOOP_H_
