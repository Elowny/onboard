#include "onboard/planner/router/noa/noa_main_loop.h"

#include <optional>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "google/protobuf/repeated_ptr_field.h"

#include "common/proto/drive_mission.pb.h"

#include "onboard/lite/logging.h"
#include "onboard/maps/v2/semantic_map_manager.h"
#include "onboard/maps/v2/semantic_map_spatial_index.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/geometry/util.h"
#include "onboard/planner/router/noa/hd_hmi_adapter.h"
#include "onboard/planner/router/noa/route_mpp.h"
#include "onboard/planner/router/route_util.h"
#include "onboard/planner/router/router_defs.h"
#include "onboard/planner/router/sim/snapshot_sim_state.h"
#include "onboard/proto/hmi_content.pb.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/proto/route.pb.h"
#include "onboard/proto/route_navigation.pb.h"
#include "onboard/utils/status_macros.h"

namespace qcraft::planner::route::noa {
namespace {

absl::Status CheckNoaRouteNavCondition(
    const NoaRouteNavInput& noa_route_nav_input, const RouteManagerState* rms) {
  if (noa_route_nav_input.mpp == nullptr) {
    return absl::FailedPreconditionError(
        "CheckNoaRouteNavCondition, The mpp cannot be null.");
  }
  if (noa_route_nav_input.smm_index == nullptr) {
    QLOG_EVERY_N_SEC(WARNING, 1) << "Not received smm v2 yet.";
    return absl::FailedPreconditionError(
        "CheckNoaRouteNavCondition, The smm_index is null.");
  }
  const auto& lt = noa_route_nav_input.localization_transform;
  if (lt == nullptr) {
    return absl::FailedPreconditionError(
        "CheckNoaRouteNavCondition , The localization_transform is null.");
  }
  if (!IsValidLocalization(lt)) {
    return absl::FailedPreconditionError(absl::StrCat(
        "CheckNoaRouteNavCondition, Inavlid localization_transform status.",
        lt->loc_status()));
  }
  if (rms == nullptr) {
    return absl::FailedPreconditionError(
        "CheckNoaRouteNavCondition, The output RouteManagerState is null.");
  }
  return absl::OkStatus();
}

absl::Status CheckNoaIncrementalUpdateLoopCondition(
    const IncrementalNoaOutput* incremental_noa_output) {
  if (incremental_noa_output == nullptr) {
    return absl::FailedPreconditionError("The incremental_noa_output is null.");
  }
  if (incremental_noa_output->external_route_manager == nullptr) {
    return absl::FailedPreconditionError("The external_route_manager is null.");
  }
  if (incremental_noa_output->rms == nullptr) {
    return absl::FailedPreconditionError("The rms is null.");
  }
  if (incremental_noa_output->route_mgr_output_proto == nullptr) {
    return absl::FailedPreconditionError("The route_mgr_output_proto is null.");
  }
  return absl::OkStatus();
}

void RectifyNavActionByRouteOutput(const mapping::v2::SemanticMapManager& smm,
                                   RouteManagerState* rms) {
  const auto& sections = rms->route_from_current.route_section_sequence();
  auto* nav_instruction =
      rms->routing_result_proto.mutable_current_navi_instruction();
  UpdateNavInstruction(smm, sections, odc::kNavActionHorizonAtMost,
                       nav_instruction);
  *rms->route_content_proto.mutable_nav_action() =
      rms->routing_result_proto.current_navi_instruction().navi_action();
  rms->route_content_proto.set_distance_to_action(nav_instruction->distance());
}

absl::Status InjectSdRouteBySnapshotSim(
    const RoutingInput& routing_input,
    ExternalRouteManager* external_route_manager) {
  auto route_manager_state_or = route::RecoverBasicSnapshotState({
      .routing_result_proto = routing_input.routing_result.get(),
      .rms_debug = &(routing_input.routing_state->route_mgr_state()),
  });
  if (route_manager_state_or.ok()) {
    external_route_manager->InjectState(
        std::move(route_manager_state_or).value());

    auto* coords = external_route_manager->mutable_coords();
    coords->clear();
    coords->reserve(
        routing_input.routing_result->path_points_from_current_size());
    for (const auto& pt_proto :
         routing_input.routing_result->path_points_from_current()) {
      Vec2d pt;
      pt.FromProto(pt_proto);
      coords->push_back(pt);
    }
    return absl::OkStatus();
  } else {
    QLOG(ERROR) << "RecoverPartialSnapshotState error: "
                << route_manager_state_or.status();
    return route_manager_state_or.status();
  }
}

void UpdateHdHmiContent(const mapping::v2::SemanticMapSpatialIndex& smm_index,
                        const RouteSectionSequenceProto& route_section_sequence,
                        const std::optional<Vec2d>& sd_destination,
                        RouteContentProto* route_content_proto) {
  route_content_proto->clear_nca_odc();
  NcaOdcProto* nca_odc = route_content_proto->mutable_nca_odc();
  nca_odc->set_distance_to_map_boundary(CalcRouteSectionsLength(
      *smm_index.semantic_manager(), route_section_sequence));
  int dist_to_sd_end = route_content_proto->distance_to_next_station();
  int dist_to_map_boundary =
      route_content_proto->nca_odc().distance_to_map_boundary();

  if (sd_destination.has_value()) {
    auto dist_to_hd_end =
        CaclDistanceToEnd(smm_index, route_section_sequence, *sd_destination,
                          dist_to_sd_end, dist_to_map_boundary);
    if (dist_to_hd_end.has_value()) {
      nca_odc->set_distance_to_hd_destination(*dist_to_hd_end);
    }
  }
  UpdateHmiOddEvent(*smm_index.semantic_manager(), route_section_sequence,
                    odc::kEventHorizonAtMost, nca_odc);
}

void SetHmiNavMode(bool on_sd_route, bool on_hd_route,
                   RouteContentProto* route_content) {
  if (on_sd_route && on_hd_route) {
    route_content->set_nav_mode(NavMode::ON_SD_HD_ROUTE);
  } else if (on_hd_route) {
    route_content->set_nav_mode(NavMode::ON_HD_ROUTE);
  } else if (on_sd_route) {
    route_content->set_nav_mode(NavMode::ON_SD_ROUTE);
  } else {
    route_content->set_nav_mode(NavMode::OFF_ROUTE);
  }
}

}  // namespace

std::string NoaStartupConfig::DebugString() const {
  return absl::StrFormat(
      "route_snapshot_sim_mode: %d, "
      "route_use_sdroute: %d,"
      " route_sd_max_project_distance: %f",
      route_snapshot_sim_mode, route_use_sdroute,
      route_sd_max_project_distance);
}

absl::StatusOr<RouteManagerOutputProto> ComputeNoaRouteNav(
    const NoaRouteNavInput& noa_route_nav_input, bool route_trunc_mpp_by_nav,
    /*In&Out*/ RouteManagerState* rms) {
  RETURN_IF_ERROR(CheckNoaRouteNavCondition(noa_route_nav_input, rms));
  const auto* smm_index = noa_route_nav_input.smm_index;
  const auto* mpp = noa_route_nav_input.mpp.get();
  const auto& heading = noa_route_nav_input.heading;
  const auto* route_param_proto = noa_route_nav_input.route_param_proto;
  const auto av_global2d = noa_route_nav_input.av_global2d;
  const auto update_id = noa_route_nav_input.update_id;
  auto rm_output_proto_or = route::noa::CalcNavInfoFromMpp(
      *smm_index, *mpp, av_global2d, heading, update_id,
      noa_route_nav_input.av_speed,
      route_param_proto->speed_limit_look_ahead_time(), route_trunc_mpp_by_nav,
      noa_route_nav_input.restrict_proto);

  if (rm_output_proto_or.ok()) {
    *rms->route_from_current.mutable_route_section_sequence() =
        rm_output_proto_or->route_sections_from_current();

    const auto& points = rms->routing_result_proto.path_points_from_current();
    std::optional<Vec2d> sd_destination =
        rms->has_route() && !points.empty()
            ? std::optional(Vec2dFromProto(points[points.size() - 1]))
            : std::nullopt;

    UpdateHdHmiContent(*smm_index,
                       rm_output_proto_or->route_sections_from_current(),
                       sd_destination, &rms->route_content_proto);

  } else {
    QLOG_EVERY_N_SEC(ERROR, 3)
        << "CalcNavInfoFromMpp failed, car pos: "
        << absl::StrFormat("%.8f %.8f", av_global2d.x(), av_global2d.y())
        << ", heading: "
        << (heading.has_value() ? std::to_string(*heading) : "NoHeading")
        << ", update_id:" << update_id
        << ", route_trunc_mpp_by_nav: " << route_trunc_mpp_by_nav << ". "
        << rm_output_proto_or.status();
  }
  return rm_output_proto_or;
}

absl::Status AddSdRouteToExternalRouteManager(
    const RoutingInput& routing_input, bool route_snapshot_sim_mode,
    bool route_use_sdroute, ExternalRouteManager* external_route_manager) {
  if (route_snapshot_sim_mode) {
    if (routing_input.routing_result == nullptr) {
      return absl::FailedPreconditionError(
          "The routing_result should not be nullptr at the snapshot sim mode.");
    }
    if (!external_route_manager->has_route()) {
      QLOG(INFO) << "Recover snapshot sim mode."
                 << routing_input.routing_result->DebugString();
      RETURN_IF_ERROR(
          InjectSdRouteBySnapshotSim(routing_input, external_route_manager));
    }
  }
  if (route_use_sdroute) {
    if (routing_input.sd_route_proto == nullptr) {
      return absl::FailedPreconditionError(
          "The sd_route_proto is nullptr, inconsitent with the "
          "route_use_sdroute mode.");
    } else {
      if (auto s = external_route_manager->InitSdRoute(
              routing_input.sd_route_proto,
              routing_input.sd_route_match_result_proto);
          s.ok()) {
        QLOG(INFO) << "Update sdroute success: "
                   << routing_input.sd_route_proto->route_id();
      } else {
        QLOG(WARNING) << "Cannot convert sdroute to routing result: " << s;
      }
    }
  }
  return absl::OkStatus();
}

NoaStatus NoaIncrementalUpdateLoop(
    const RoutingInput& routing_input,
    const std::shared_ptr<mapping::v2::SemanticMapSpatialIndexListenerAsnyc>&
        smm_listener,
    const RouteParamProto& route_param_proto,
    NoaStartupConfig noa_constant_input, const Vec2d& av_global2d,
    const std::optional<double>& heading, RestrictProto restrict_proto,
    IncrementalNoaOutput* incremental_noa_output) {
  const auto s = CheckNoaIncrementalUpdateLoopCondition(incremental_noa_output);
  if (!s.ok()) {
    QLOG_EVERY_N(WARNING, 3)
        << "Check NoaIncrementalUpdateLoop input failed. " << s;
    return {
        .route_nav_status = s,
        .sd_route_status = s,
    };
  }
  auto* external_route_manager = incremental_noa_output->external_route_manager;
  auto* rms = incremental_noa_output->rms;
  auto* route_mgr_output_proto = incremental_noa_output->route_mgr_output_proto;
  rms->route_content_proto.clear_nca_odc();
  rms->route_content_proto.clear_nav_action();
  // Avoid repeated generating the sd route
  if (routing_input.sd_route_proto != nullptr &&
      routing_input.sd_route_proto->status() == SDRouteProto::DELETE) {
    if (!external_route_manager->has_route()) {
      QLOG_EVERY_N_SEC(ERROR, 3) << "No sd route found before.";
    }
    external_route_manager->DeleteSdRoute(routing_input.sd_route_proto);
  } else if (!external_route_manager->has_route() ||
             (routing_input.sd_route_proto != nullptr &&
              routing_input.sd_route_proto->status() == SDRouteProto::CREATE)) {
    if (const auto gen_sd_route_status = AddSdRouteToExternalRouteManager(
            routing_input, noa_constant_input.route_snapshot_sim_mode,
            noa_constant_input.route_use_sdroute, external_route_manager);
        !gen_sd_route_status.ok()) {
      QLOG_EVERY_N_SEC(WARNING, 3)
          << "Generate the sd route failed at noa mode. "
          << noa_constant_input.DebugString()
          << ", status: " << gen_sd_route_status;
    }
  }

  CoordinateConverter cc = CoordinateConverter::FromMapOriginCoord(av_global2d);
  int64_t origin_update_id = rms->routing_result_proto.success()
                                 ? rms->routing_result_proto.update_id()
                                 : 0;
  absl::Status route_nav_status = absl::NotFoundError("Cannot found ");

  route_mgr_output_proto->set_route_status(RouteManagerOutputProto::INVALID);
  if (smm_listener != nullptr && routing_input.mpp != nullptr) {
    const auto smm_index = smm_listener->Get();
    auto route_manager_output_or = ComputeNoaRouteNav(
        NoaRouteNavInput{
            .smm_index = smm_index.get(),
            .route_param_proto = &route_param_proto,
            .mpp = routing_input.mpp,
            .localization_transform = routing_input.localization_transform,
            .av_global2d = av_global2d,
            .heading = heading,
            .update_id = origin_update_id,
            .cc = cc,
            .av_speed = routing_input.pose->vel_body().x(),
            .restrict_proto = std::move(restrict_proto),
        },
        noa_constant_input.route_trunc_mpp_by_nav, rms);
    if (route_manager_output_or.ok()) {
      route_nav_status = route_manager_output_or.status();
      *route_mgr_output_proto = std::move(route_manager_output_or).value();
    } else {
      route_nav_status = std::move(route_manager_output_or).status();
    }
    if (smm_index != nullptr && smm_index->semantic_manager() != nullptr) {
      RectifyNavActionByRouteOutput(*smm_index->semantic_manager(), rms);
    }
  }
  absl::Status sd_route_status = absl::OkStatus();
  if (external_route_manager->has_route()) {
    sd_route_status = IncremantalUpdateSdRoute(
        external_route_manager, av_global2d,
        noa_constant_input.route_sd_max_project_distance);
  } else {
    sd_route_status = absl::NotFoundError("Ignore emtpy sd route.");
  }

  // Set hmi nav_mode
  SetHmiNavMode(sd_route_status.ok(), route_nav_status.ok(),
                &rms->route_content_proto);

  return {
      .route_nav_status = route_nav_status,
      .sd_route_status = std::move(sd_route_status),
  };
}

absl::Status IncremantalUpdateSdRoute(
    ExternalRouteManager* external_route_manager, const Vec2d& av_global2d,
    double route_sd_max_project_distance) {
  bool ok_update = external_route_manager->UpdateRoute(
      av_global2d, route_sd_max_project_distance);
  if (!ok_update) {
    return absl::InternalError(absl::StrCat(
        "Update route failed, pose: ",
        absl::StrFormat("%.8f %.8f", av_global2d.x(), av_global2d.y())));
  }
  return absl::OkStatus();
}
}  // namespace qcraft::planner::route::noa
