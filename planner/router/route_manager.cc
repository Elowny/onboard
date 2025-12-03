#include "onboard/planner/router/route_manager.h"

// IWYU pragma: no_include <boost/move/utility_core.hpp>
// IWYU pragma: no_include <ext/alloc_traits.h>
// IWYU pragma: no_include <google/protobuf/repeated_ptr_field.h>

#include <algorithm>
#include <memory>
#include <optional>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

#include "absl/cleanup/cleanup.h"
#include "absl/container/flat_hash_set.h"
#include "absl/hash/hash.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "absl/time/time.h"
#include "glog/logging.h"

#include "common/proto/drive_mission.pb.h"
#include "common/proto/lane_point.pb.h"
#include "common/proto/map_geometry.pb.h"

#include "onboard/autonomy_state/autonomy_state_util.h"
#include "onboard/container/strong_int.h"
#include "onboard/eval/qevent.h"
#include "onboard/eval/qevent_base.h"
#include "onboard/global/clock.h"
#include "onboard/global/trace.h"
#include "onboard/lite/logging.h"
#include "onboard/lite/qissue_trans.h"
#include "onboard/maps/lane_path_util.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/map_or_die_macros.h"
#include "onboard/maps/proto/lane_path.pb.h"
#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/maps/spatial_search_util.h"
#include "onboard/maps/v2/semantic_map_object.h"
#include "onboard/math/coordinate_converter.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/geometry/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/composite_lane_path.h"
#include "onboard/planner/router/alternate/alternative_route.h"
#include "onboard/planner/router/alternate/alternative_route_util.h"
#include "onboard/planner/router/geometry/gfc.h"
#include "onboard/planner/router/map_match.h"
#include "onboard/planner/router/multi_stops_request.h"
#include "onboard/planner/router/navi/navi_output.h"
#include "onboard/planner/router/navi/route_navi_info.h"
#include "onboard/planner/router/proto/route_manager_output.pb.h"
#include "onboard/planner/router/request/routing_request_context.h"
#include "onboard/planner/router/road_conditions_process.h"
#include "onboard/planner/router/route_ego_tracker.h"
#include "onboard/planner/router/route_generator.h"
#include "onboard/planner/router/route_manager_util.h"
#include "onboard/planner/router/route_sections.h"
#include "onboard/planner/router/route_sections_util.h"
#include "onboard/planner/router/route_speed_limit_util.h"
#include "onboard/planner/router/route_util.h"
#include "onboard/planner/router/router_defs.h"
#include "onboard/planner/router/router_flags.h"
#include "onboard/proto/q_issue.pb.h"
#include "onboard/proto/route_navigation.pb.h"
#include "onboard/utils/source_location.h"
#include "onboard/utils/status_macros.h"
#include "onboard/utils/time_util.h"

namespace qcraft {
namespace planner {
namespace {

//  The min time interval if reroute when map-match failed
constexpr double kResetThresholdMicros = 250 * 1000.0;
// The min travel distance if reset  when map-match failed
constexpr double kDriftDistanceResetThreshold = 50.0;
// The min time interval if reroute when map-match failed
constexpr double kRerouteThresholdMicros = 1500 * 1000.0;
// The max manual driving time interval before reroute even
// map-match success
constexpr double kManualDrivingThresholdMicros = 5000 * 1000.0;
constexpr double kInvalidDistance = 10000000.0;  // m.

// Alternative route params
constexpr double kAlternateRouteTravelError = 5.0;        // m.
constexpr double kAlternateRouteLookAheadDist = 500.0;    // m.
constexpr double kAlternateRouteExtraCostPerMeter = 2.5;  // m.
constexpr double kSwitchRouteDistance = 20;               // m.

template <typename Container>
double GetPercentile(const Container& c, double percentile) {
  std::vector v(c.begin(), c.end());
  std::sort(v.begin(), v.end());
  return v[static_cast<int>(v.size() * percentile)];
}

absl::StatusOr<RoutingResultProto> GenerateOffRoadRouteResult(
    const RoutingRequestProto& routing_request, int64_t next_request_id) {
  if (HasOnlyOneOffRoadRequest(routing_request)) {
    RoutingResultProto result_proto;
    result_proto.set_success(true);
    *result_proto.mutable_routing_request() = routing_request;
    result_proto.set_update_id(next_request_id);
    result_proto.set_routing_type(RoutingType::PARKING_OFFROAD);
    return result_proto;
  }
  return absl::UnavailableError("Only support only one offroad stop yet.");
}

std::optional<TurnSignal> GetTurnSignalOnStart(double dist_from_start) {
  constexpr double kDepartSignalDistance = 10.0;  // meters.
  return (dist_from_start < kDepartSignalDistance)
             ? std::make_optional(TURN_SIGNAL_LEFT)
             : std::nullopt;
}

std::optional<TurnSignal> GetTurnSignalOnEnd(double dist_to_end) {
  constexpr double kStopSignalDistanceAhead = 30.0;  // meters
  return (dist_to_end < kStopSignalDistanceAhead)
             ? std::make_optional(TURN_SIGNAL_RIGHT)
             : std::nullopt;
}

// When the AV current pos to lane failed, return true if out of the distance or
// time threshold, otherwise false.
bool ShouldResetPathInfo(int64_t miss_matched_time,
                         double reset_threshold_micros, double drift_distance,
                         double drift_distance_reset_threshold) {
  return miss_matched_time > reset_threshold_micros ||
         drift_distance > drift_distance_reset_threshold;
}

absl::StatusOr<double> CalcDistanceToNextDestination(
    const mapping::v2::SemanticMapManager& semantic_map_manager,
    const RouteProto& route_from_current,
    const MultipleStopsRequest& multi_stops, int next_destination_index) {
  if (route_from_current.has_lane_path() &&
      route_from_current.routing_request().destinations().empty()) {
    return absl::InvalidArgumentError("No current destination");
  }
  if (next_destination_index < 0) {
    return absl::InvalidArgumentError("next_destination_index must be invalid");
  }
  const auto index_pair = multi_stops.GetIndexPair(next_destination_index);
  const mapping::LanePoint& lane_point =
      multi_stops
          .lane_point_destinations()[index_pair.first][index_pair.second];
  SMM_ASSIGN_LANE_OR_RETURN(lane_info, semantic_map_manager,
                            lane_point.lane_id(),
                            absl::InternalError(absl::StrCat(
                                "Cannot find lane id ", lane_point.lane_id())));
  SMM_SECTION_PROTO_OR_RETURN(
      section_proto, semantic_map_manager, lane_info.Proto().section_id(),
      absl::InternalError(absl::StrCat("Cannot find section id ",
                                       lane_info.Proto().section_id())));
  auto point_to_composite_lane_path_or = FindLaneFromLastCompositeIndex(
      route_from_current.lane_path(), {0, 0}, section_proto->lanes(),
      mapping::ElementId(lane_info.Proto().id()));
  if (!point_to_composite_lane_path_or.ok()) {
    return point_to_composite_lane_path_or.status();
  }
  return TravelDistanceBetween(
      semantic_map_manager, route_from_current.lane_path(), {0, 0},
      route_from_current.lane_path().lane_paths()[0].start_fraction(),
      point_to_composite_lane_path_or->composite_index, lane_point.fraction());
}

void VLog(int n, const std::string& content) { VLOG(n) << content; }

void InfoEveryNSec(int n, const std::string& content) {
  QLOG_EVERY_N_SEC(INFO, n) << content;
}

void WarningEveryNSec(int n, const std::string& content) {
  QLOG_EVERY_N_SEC(ERROR, n) << content;
}

[[maybe_unused]] void ErrorEveryNSec(int n, const std::string& content) {
  QLOG_EVERY_N_SEC(ERROR, n) << content;
}

}  // namespace

namespace internal {
bool IsStopArrived(double distance) {
  return distance <= FLAGS_route_arrive_distance;
}
}  // namespace internal

RouteContentProto RouteManager::GenerateRouteContent() const {
  RouteContentProto content_proto;

  for (const auto& stop_name : data_.multi_stops.stop_name_list()) {
    *content_proto.add_stations_list() = stop_name;
  }

  content_proto.set_next_station(
      data_.next_destination_index >= 0
          ? data_.multi_stops.GetStopSerialNumber(data_.next_destination_index)
          : -1);

  content_proto.set_distance_to_next_station(
      data_.next_destination_index >= 0
          ? data_.rms_debug.distance_to_next_stop()
          : kInvalidDistance);
  *content_proto.mutable_routing_request_id() = data_.multi_stops.request_id();
  content_proto.set_next_viapoint_id(data_.rms_debug.next_destination_id());
  content_proto.set_distance_to_next_viapoint(
      data_.rms_debug.distance_to_next_viapoint());

  // Add HMI special requirements, ONLY add sub action type ENTER_RAMP
  const auto& cur_inst = data_.routing_result_proto.current_navi_instruction();
  if (cur_inst.has_navi_action()) {
    *content_proto.mutable_nav_action() = cur_inst.navi_action();
    content_proto.set_distance_to_action(
        data_.routing_result_proto.current_navi_instruction().distance());
  }
  return content_proto;
}

void RouteManager::InitRequest(
    const std::shared_ptr<const RecordedRouteProto>& recorded_route,
    const std::shared_ptr<const RoutingResultProto>& log_routing_result,
    int64_t injected_teleop_micro_secs) {
  SCOPED_QTRACE("RouteManager::InitRequest");
  // Recover routing request from log
  if (data_.pending_multi_stops.empty() &&
      (FLAGS_route_recover_destinations_from_log ||
       FLAGS_route_recover_multi_stops_from_log)) {
    if (!log_routing_result && !recorded_route) {
      QLOG(INFO) << "Recovering route request from recorded routing result: "
                    "waiting for "
                    "recorded route message teleop secs: "
                 << injected_teleop_micro_secs;
      return;
    }
    const RoutingRequestProto* log_request = nullptr;
    if (log_routing_result && log_routing_result->has_routing_request()) {
      log_request = &log_routing_result->routing_request();
    } else if (recorded_route && recorded_route->has_routing_request()) {
      log_request = &recorded_route->routing_request();
    }
    if (log_request != nullptr) {
      auto status = RecoverRoutingRequest(*log_request, *semantic_map_manager_,
                                          map_index_);
      if (!status.ok()) {
        QLOG(WARNING) << status.status().message();
        QISSUEX(QIssueSeverity::QIS_FATAL, QIssueType::QIT_BUSINESS,
                QIssueSubType::QIST_PLANNER_ROUTING_ROUTE_RECOVER_FAILED,
                "Router initialization from log route failed.");
        QLOG(INFO) << "Router initialization from log route failed.";
      } else {
        data_.pending_multi_stops = std::move(status).value();
        QLOG(INFO) << "Router initialization done, recorded route recovered.";
      }
    }
  }
  // Create a default routing request if route is empty
  if (data_.pending_multi_stops.empty()) {
    const auto default_request = ConvertAllRouteFlagsToRoutingRequestProto();
    if (default_request.ok()) {
      auto multiple_stops_request_or = BuildMultipleStopsRequest(
          *semantic_map_manager_, map_index_, default_request.value());
      if (multiple_stops_request_or.ok()) {
        QLOG(INFO) << "Router initialization done, new default request queued.";
        data_.pending_multi_stops =
            std::move(multiple_stops_request_or).value();
      }
    } else {
      QLOG_EVERY_N_SEC(INFO, 1)
          << "Fail to create default routing request. waiting request..";
    }
  }
}

// NOLINTBEGIN(readability-function-size)
// NOLINTNEXTLINE
absl::StatusOr<RouteManagerOutput> RouteManager::Update(
    bool is_bus, const RouteManagerInput& input) {
  SCOPED_QTRACE("RouteManager::Update");
  ClearIterationState();
  vx_history_.push_back(input.pose->vel_body().x());
  data_.vx_p90 = GetPercentile(vx_history_, 0.9);
  auto& rms_debug = data_.rms_debug;
  const auto coordinate_converter =
      CoordinateConverter::FromLocalizationTransform(
          *input.localization_transform);
  const Vec3d pos = Vec3dFromProto(input.pose->pos_smooth());
  const Vec3d global = coordinate_converter.SmoothToGlobal(pos);
  const auto heading =
      coordinate_converter.SmoothYawToGlobal(input.pose->yaw());
  VLog(3, absl::StrCat("The last p90 s speed: ", data_.vx_p90));
  RouteManagerOutput rm_output;
  bool is_new_route = false;
  bool new_request_arrived = !data_.pending_multi_stops.empty();
  // New route request arrived or initiated by route FLAGS
  if (data_.active_routing_request == nullptr ||
      !data_.pending_multi_stops.empty()) {
    UpdateDistance();  // Work around, compute distance twice
    const auto routing_result_or = InitiateRoutingRequest(input, is_bus);
    if (routing_result_or.ok()) {
      data_.routing_result_proto.Clear();
      VLog(3,
           absl::StrCat("routing result: ", routing_result_or->DebugString()));
      data_.routing_result_proto = *routing_result_or;
      const bool received = ReceivedSuccessfulRoutingResult(*routing_result_or);
      if (!received) {
        QLOG(ERROR) << "ReceivedSuccessfulRoutingResult failed. ";
        this->last_error_ = route::RouteCreateLanePathFailedError(absl::StrCat(
            "Create route lane path failed.", routing_result_or->update_id()));
        return absl::InternalError("ReceivedSuccessfulRoutingResult");
      } else {
        QLOG(INFO)
            << "ReceivedSuccessfulRoutingResult ok. new route routing result:"
            << routing_result_or->success();
        data_.alter_route_state = std::nullopt;
        is_new_route = true;
        rms_debug.set_is_internal_reroute(false);
        rms_debug.set_last_update_time(absl::ToUnixMicros(Clock::Now()));
        if (routing_result_or->has_lane_path() &&
            routing_result_or->lane_path().lane_paths().size() > 0) {
          const auto& fist_lane_path =
              routing_result_or->lane_path().lane_paths(0);
          data_.last_lane_point = {
              mapping::ElementId(fist_lane_path.lane_ids(0)),
              fist_lane_path.start_fraction()};
        }
      }
    } else {
      this->last_error_ =
          route::RoutePathNotFoundError(routing_result_or.status().ToString());
      VLog(3, absl::StrCat("Failed initiate routing request: ",
                           routing_result_or.status().message()));
      if (new_request_arrived) {
        // Return for kickout if we have a new routing request and failed.
        WarningEveryNSec(
            2, absl::StrCat("New routing request failed, maybe take into "
                            "account last request and reroute later."));
        return routing_result_or.status();
      }
    }
  }

  if (data_.has_route()) {
    // Once reached here, we must already have a valid route.
    // next_destination_index must also be valid.
    VLog(3, absl::StrCat("Get route_from_current, dist: ",
                         rms_debug.distance_to_next_stop(),
                         ", secs:", rms_debug.eta_secs_to_next_stop()));

    // Switch alternative route if necessary.
    if (input.switch_alternate_route && FLAGS_route_enable_alternative_path) {
      InfoEveryNSec(
          3, "Received switch signal, try to switch to alternate route.");
      if (data_.rms_debug.distance_to_next_stop() <= kSwitchRouteDistance) {
        InfoEveryNSec(3,
                      "Ego car is approaching station, no need "
                      "to switch to alternative route.");
      } else if (data_.alter_route_state.has_value()) {
        data_.route = std::move(data_.alter_route_state->route);
        data_.route_from_current =
            std::move(data_.alter_route_state->route_from_current);
        data_.routing_result_proto =
            std::move(data_.alter_route_state->routing_result_proto);
        data_.alter_route_state = std::nullopt;
        ResetRoutePathInfo();
        is_new_route = true;
        rms_debug.set_is_internal_reroute(true);
        data_.route.set_update_id(++data_.request_id);
        data_.route_from_current.set_update_id(data_.request_id);
        data_.routing_result_proto.set_update_id(data_.request_id);
        InfoEveryNSec(3, "Switch to alternate route success in Router.");
      }
    }

    auto tracker_result = GetTrackerInfoOnRoute(
        *semantic_map_manager_, map_index_, coordinate_converter.GetLevel(),
        data_, {global.x(), global.y()}, heading, input.pose->vel_body().x());

    if (tracker_result.success()) {
      const auto clp_proto = AfterCompositeIndexAndLanePoint(
          data_.route.lane_path(), tracker_result.composite_index(),
          tracker_result.lane_point());
      VLog(3, absl::StrCat(
                  "Car to lane path result: composite_index:",
                  tracker_result.composite_index().DebugString(),
                  "\n composite_lane_path: ", clp_proto.ShortDebugString()));
      const auto station_status = UpdateStationsInfo(clp_proto);
      if (!station_status.ok()) {
        InfoEveryNSec(
            10, "Parse destinations failed, can not update station info.");
      }
      const auto status = UpdateRouteFromCurrent(is_bus, clp_proto);
      if (status.ok() &&
          data_.route_from_current.lane_path().lane_paths().size() > 0) {
        auto navi_output_or = route::CreateNaviOutputFromRoute(
            *semantic_map_manager_, data_.route_from_current.lane_path(),
            data_.routing_result_proto.update_id());
        if (navi_output_or.ok()) {
          if (is_new_route) {
            *data_.routing_result_proto.mutable_navi_preview() =
                std::move(navi_output_or->navi_preview);
            QLOG(INFO) << "Total navi segment size: "
                       << data_.routing_result_proto.navi_preview()
                              .navi_segments_size();
          }
          *data_.routing_result_proto.mutable_current_navi_instruction() =
              std::move(navi_output_or->current_navi_instruction);
        }
      } else {
        InfoEveryNSec(5, absl::StrCat("Failed to update route from current: ",
                                      status.message()));
      }
      // TODO(xiang): Move to rms_debug proto
      data_.composite_index.FromProto(tracker_result.composite_index());
      data_.last_lane_point.FromProto(tracker_result.lane_point());
      rms_debug.set_route_s(rms_debug.route_s() +
                            tracker_result.travel_distance());
      rms_debug.set_last_update_time(absl::ToUnixMicros(Clock::Now()));
    } else {
      WarningEveryNSec(
          2,
          absl::StrFormat("Route ego tracker failed, reason: Failed to track "
                          "AV. Car pose is: (%.8f, %.8f) \n. Last lane "
                          "id is: %d",
                          global.x(), global.y(),
                          rms_debug.route_tracker().lane_point().lane_id()));
    }

    absl::Cleanup tracker_destroy_fn = [&] {
      *rms_debug.mutable_route_tracker() = std::move(tracker_result);
    };
    UpdateDistance();  // Update distance even tracker failed.

    int64_t missed_micros =
        absl::ToUnixMicros(Clock::Now()) - rms_debug.last_update_time();
    if (input.planner_reroute || !tracker_result.success() ||
        missed_micros > kManualDrivingThresholdMicros) {
      if (!tracker_result.success()) {
        WarningEveryNSec(
            3, absl::StrCat(
                   "Pose to lane failed, off-route now, should need Reroute "
                   "later, "
                   "pose:",
                   PoseDebugString(coordinate_converter, *input.pose),
                   ", reach destination index: ",
                   rms_debug.reached_destination_index(),
                   ", pending_next_destination_index: ",
                   rms_debug.pending_next_destination_index(),
                   ", auto drive state: ",
                   IS_AUTO_DRIVE(input.autonomy_state->autonomy_state())));
        QEVENT_EVERY_N_SECONDS(
            "xiang", "route_map_match_fail", 10, [&](QEvent* qevent) {
              qevent->AddField("x", global.x());
              qevent->AddField("y", global.y());
              qevent->AddField("z", global.z());
              qevent->AddField("yaw", input.pose->yaw());
              qevent->AddField("vx", input.pose->vel_body().x());
            });
      }
      VLog(3, absl::StrCat(
                  "reach:", rms_debug.reached_destination_index(),
                  ", pending:", rms_debug.pending_next_destination_index(),
                  ", auto drive state:",
                  IS_AUTO_DRIVE(input.autonomy_state->autonomy_state())));
      const bool route_tracker_reroute =
          missed_micros > kRerouteThresholdMicros &&
          !internal::IsStopArrived(rms_debug.distance_to_next_stop());
      const bool planner_reroute =
          input.planner_reroute &&
          !internal::IsStopArrived(rms_debug.distance_to_next_stop());
      if (planner_reroute || route_tracker_reroute) {
        if (route_tracker_reroute) {
          InfoEveryNSec(
              1,
              absl::StrCat(
                  "reroute, missed_micros:", missed_micros,
                  ", already reached: ", rms_debug.reached_destination_index(),
                  ", pending: ", rms_debug.pending_next_destination_index(),
                  ", missed_micros:", missed_micros,
                  ", tracker_result:", tracker_result.success()));
        }
        if (planner_reroute) {
          InfoEveryNSec(
              1, absl::StrCat(
                     "Planner request reroute, respond now. planner_reroute: ",
                     input.planner_reroute));
        }
        auto status =
            Reroute(*semantic_map_manager_, *map_index_, coordinate_converter,
                    *route_param_proto_, *input.pose,
                    data_.route_from_current.routing_request(),
                    data_.multi_stops, RouteSectionSequenceProto(),
                    ++data_.request_id, is_bus, &route_core_, &data_);
        if (!status.ok()) {
          return status;
        }
        is_new_route = true;
      } else if (IS_AUTO_DRIVE(input.autonomy_state->autonomy_state()) &&
                 ShouldResetPathInfo(missed_micros, kResetThresholdMicros,
                                     MicroSecondsToSeconds(missed_micros) *
                                         input.pose->vel_body().x(),
                                     kDriftDistanceResetThreshold)) {
        InfoEveryNSec(
            3, absl::StrCat(
                   "No update for a long time. reset route path. time diff: ",
                   missed_micros, ", max speed: ", data_.vx_p90,
                   ", kResetThresholdMicros: ", kResetThresholdMicros,
                   ", kDriftDistanceResetThreshold: ",
                   kDriftDistanceResetThreshold));
        // Because the tracker is success, SHOULD not set to the route start.
        data_.rms_debug.set_last_update_time(-1);
      } else {
        InfoEveryNSec(
            3, absl::StrCat(
                   "No-op although the AV is not on route. ",
                   "drive_state: ", input.autonomy_state->autonomy_state(),
                   ", time diff:", missed_micros, ", kResetThresholdMicros:",
                   kResetThresholdMicros, ", kDriftDistanceResetThreshold:",
                   kDriftDistanceResetThreshold));
      }
    }

    rm_output.route_sections_from_current = RouteSections::BuildFromProto(
        data_.route_from_current.route_section_sequence());

    const double look_ahead_dist =
        route_param_proto_->speed_limit_look_ahead_time() *
        input.pose->vel_body().x();
    const auto recommend_speed_limit = FindFrontOverwrittenSpeedLimit(
        *semantic_map_manager_,
        data_.route_from_current.route_section_sequence(), look_ahead_dist);
    if (recommend_speed_limit.has_value()) {
      rm_output.recommend_max_speed_limit = *recommend_speed_limit;
    }

    rm_output.avoid_lanes = {data_.route.avoid_lanes().begin(),
                             data_.route.avoid_lanes().end()};
    rm_output.routing_request_proto =
        data_.route_from_current.routing_request();

    // Calculate route navi info.
    auto route_navi_info_or = CalcCurRouteNaviInfo(
        *semantic_map_manager_, data_.route.route_section_sequence(),
        data_.route_from_current.route_section_sequence(),
        rm_output.avoid_lanes, data_.last_lane_point,
        route::noa::kNaviInfoBackwardExtendLength,
        route::noa::kNaviInfoPreviewDistance);

    if (route_navi_info_or.ok()) {
      rm_output.route_navi_info = std::move(route_navi_info_or).value();
    }

    if (is_new_route && !rms_debug.is_internal_reroute()) {
      const Vec2d global2d = coordinate_converter.SmoothToGlobal(
          {input.pose->pos_smooth().x(), input.pose->pos_smooth().y()});
      global2d.ToProto(rms_debug.mutable_start_next_route_pos());
    }

    // Alternative route.
    UpdateAlternativeRoute(is_bus);
    if (data_.alter_route_state.has_value()) {
      const absl::flat_hash_set<mapping::ElementId> alter_avoid_lanes = {
          data_.alter_route_state->route.avoid_lanes().begin(),
          data_.alter_route_state->route.avoid_lanes().end()};

      auto alter_route_navi_info_or = CalcCurRouteNaviInfo(
          *semantic_map_manager_,
          data_.alter_route_state->route.route_section_sequence(),
          data_.alter_route_state->route_from_current.route_section_sequence(),
          alter_avoid_lanes, data_.last_lane_point,
          route::noa::kNaviInfoBackwardExtendLength,
          route::noa::kNaviInfoPreviewDistance);

      rm_output.alter_route_msg = {
          .route_sections_from_current = RouteSections::BuildFromProto(
              data_.alter_route_state->route_from_current
                  .route_section_sequence()),
          .roundabout_distance = data_.alter_route_state->roundabout_distance};
      if (alter_route_navi_info_or.ok()) {
        rm_output.alter_route_msg->route_navi_info =
            std::move(alter_route_navi_info_or).value();
      } else {
        QLOG(ERROR) << "Calculate alternate route's route navi info failed.";
      }
    }
  }
  if (data_.routing_result_proto.success()) {
    auto stop_seq_no = data_.multi_stops.GetStopSerialNumber(
        rms_debug.pending_next_destination_index());
    rm_output.destination_stop =
        data_.multi_stops.multi_stops_request_proto().stops(stop_seq_no);
    if (data_.routing_result_proto.has_link_lane_point()) {
      *rm_output.destination_stop.mutable_depart_strategy()
           ->mutable_off_road()
           ->add_specified_onroad_points()
           ->mutable_lane_point() =
          data_.routing_result_proto.link_lane_point();
    }
    VLOG(4) << "Next stop is off road."
            << rm_output.destination_stop.ShortDebugString();
    data_.route_content_proto = GenerateRouteContent();
  }
  const Vec2d global_start_pos =
      Vec2dFromProto(rms_debug.start_next_route_pos());
  const Vec2d smooth_start_pos =
      coordinate_converter.GlobalToSmooth(global_start_pos);
  auto start_signal_opt = GetTurnSignalOnStart(smooth_start_pos.DistanceTo(
      {input.pose->pos_smooth().x(), input.pose->pos_smooth().y()}));
  if (start_signal_opt.has_value()) {
    rm_output.signal = *start_signal_opt;
  }
  auto end_signal_opt = GetTurnSignalOnEnd(rms_debug.distance_to_next_stop());
  if (end_signal_opt.has_value()) {
    rm_output.signal = *end_signal_opt;
  }
  rms_debug.set_is_new_route(is_new_route);
  rm_output.rerouted = is_new_route;
  rm_output.update_id = data_.routing_result_proto.has_update_id()
                            ? data_.routing_result_proto.update_id()
                            : kInvalidRouteUpdateId;
  rm_output.status = data_.routing_result_proto.success()
                         ? RouteManagerOutputProto::NORMAL
                         : RouteManagerOutputProto::INVALID;
  return rm_output;
}
// NOLINTEND(readability-function-size)

absl::Status RouteManager::AddManualReroutingRequest(
    const ReroutingRequestProto& request) {
  auto multiple_stops_request_or = BuildMultipleStopsRequest(
      *semantic_map_manager_, map_index_, request.routing_request());
  if (multiple_stops_request_or.ok()) {
    data_.pending_multi_stops = std::move(multiple_stops_request_or).value();
    return absl::OkStatus();
  } else {
    return multiple_stops_request_or.status();
  }
}

bool RouteManager::ReceivedSuccessfulRoutingResult(
    const RoutingResultProto& response) {
  if (!response.success()) {
    QLOG(ERROR) << "Last routing request failed";
    return false;
  }
  data_.active_routing_request.reset();
  ResetRoutePathInfo();

  const auto multi_stops = data_.pending_multi_stops.empty()
                               ? data_.multi_stops
                               : data_.pending_multi_stops;
  if (!HasOnlyOneOffRoadRequest(response.routing_request())) {
    *data_.route.mutable_lane_path() = response.lane_path();
    *data_.route.mutable_route_section_sequence() =
        response.route_section_sequence();
    *data_.route.mutable_routing_request() = response.routing_request();
    data_.route.set_update_id(response.update_id());
    data_.route.mutable_avoid_lanes()->Reserve(
        multi_stops.avoid_lanes().size());
    for (const auto& avoid_lane : multi_stops.avoid_lanes()) {
      data_.route.add_avoid_lanes(avoid_lane.value());
    }

    data_.route_from_current = data_.route;
  }
  QLOG(INFO) << "Successful routing received";
  if (!data_.pending_multi_stops.empty()) {
    data_.multi_stops = std::move(data_.pending_multi_stops);
  }
  data_.next_destination_index =
      data_.rms_debug.pending_next_destination_index();
  data_.rms_debug.set_reached_dest_cnt(0);
  data_.composite_index = {0, 0};
  UpdateDistance();
  return true;
}

absl::StatusOr<RoutingResultProto> RouteManager::InitiateRoutingRequest(
    const RouteManagerInput& input, bool is_bus) {
  FUNC_QTRACE();
  route::RoutingRequestContext context = {
      .semantic_map_manager = semantic_map_manager_,
      .map_index = map_index_,
      .route_param_proto = route_param_proto_,
      .request_id = data_.request_id + 1,
      .pred_trajectory_point = input.pred_trajectory_point,
  };
  auto coordinate_converter = CoordinateConverter::FromLocalizationTransform(
      *input.localization_transform);
  const Vec2d av_global2d = coordinate_converter.SmoothToGlobal(
      Vec2d(input.pose->pos_smooth().x(), input.pose->pos_smooth().y()));
  const auto heading =
      coordinate_converter.SmoothYawToGlobal(input.pose->yaw());

  if (!data_.pending_multi_stops.empty()) {
    RoutingRequestProto routing_request;
    data_.pending_multi_stops.ToRoutingRequestFromMultipleStops(
        &routing_request);
    VLOG(2) << "routing request:" << routing_request.DebugString() << ", "
            << data_.pending_multi_stops.destination_size();
    if (HasOnlyOneOffRoadRequest(routing_request)) {
      data_.active_routing_request =
          std::make_unique<PlannerRoutingRequestProto>();
      *(data_.active_routing_request->mutable_init()
            ->mutable_routing_request()) = routing_request;
      data_.active_routing_request->set_request_id(++data_.request_id);
      data_.rms_debug.set_pending_next_destination_index(0);
      data_.rms_debug.set_reached_destination_index(-1);
      data_.rms_debug.set_reached_dest_cnt(0);
      return GenerateOffRoadRouteResult(routing_request, data_.request_id);
    }

    const auto next_destination_index_or =
        FindNextDestinationIndexViaLanePoints(
            *semantic_map_manager_, data_.pending_multi_stops,
            [this, av_global2d, heading,
             &coordinate_converter](const RouteSections& sections) {
              auto pt_lanes = route::map_match::GetNearLanesFromPose(
                  *semantic_map_manager_, map_index_,
                  coordinate_converter.GetLevel(), av_global2d, heading,
                  route_param_proto_->on_driving_param());
              if (pt_lanes.empty()) {
                return false;
              }
              constexpr double kLengthErrorThresh = 1.0;  // m.
              for (int i = 0; i < sections.size(); ++i) {
                const auto sec_seg = sections.route_section_segment(i);
                SMM_SECTION_PROTO_OR_CONTINUE(sec_info, *semantic_map_manager_,
                                              sec_seg.id);
                const double start_frac = std::max(
                    0.0, sec_seg.start_fraction -
                             kLengthErrorThresh / sec_info->average_length());
                const double end_frac = std::min(
                    1.0, sec_seg.end_fraction +
                             kLengthErrorThresh / sec_info->average_length());
                if (pt_lanes[0].lane_proto->section_id() ==
                        sec_seg.id.value() &&
                    pt_lanes[0].fraction >= start_frac &&
                    pt_lanes[0].fraction <= end_frac) {
                  return true;
                }
              }
              return false;
            },
            &route_core_);
    if (!next_destination_index_or.ok()) {
      return absl::NotFoundError("Find next destination from pose failed.");
    }

    // sending routing request to first stop
    data_.active_routing_request =
        std::make_unique<PlannerRoutingRequestProto>();
    *(data_.active_routing_request->mutable_init()->mutable_pose()) =
        *input.pose;

    ASSIGN_OR_RETURN(auto next_stop_request,
                     GenerateAndCheckRoutingRequestProtoToNextStop(
                         *semantic_map_manager_, map_index_,
                         *route_param_proto_, data_.pending_multi_stops,
                         av_global2d, heading, *next_destination_index_or));

    *(data_.active_routing_request->mutable_init()->mutable_routing_request()) =
        std::move(next_stop_request);
    data_.active_routing_request->set_request_id(++data_.request_id);
    auto result_or = GenerateRoutingResultProto(
        context, coordinate_converter,
        data_.pending_multi_stops.route_restrict_district(),
        data_.active_routing_request.get(), is_bus, &route_core_);
    if (result_or.ok()) {
      data_.rms_debug.set_pending_next_destination_index(
          *next_destination_index_or);
      data_.rms_debug.set_reached_destination_index(-1);
      data_.rms_debug.set_reached_dest_cnt(0);
    }
    return result_or;
  }
  if (data_.next_destination_index < 0 || data_.multi_stops.empty()) {
    return absl::NotFoundError("No routing request available yet.");
  }
  const int next_stop_index =
      data_.multi_stops.GetStopIndex(data_.next_destination_index);
  const auto& next_stop = data_.multi_stops.destination(next_stop_index);
  if (next_stop.has_off_road()) {
    VLOG(3) << "Next stop off road, Not Supported."
            << next_stop.ShortDebugString();
    return absl::AlreadyExistsError("Already exist off road stop.");
  }
  if (!has_route()) {
    return absl::NotFoundError("No routing request available yet.");
  }

  // Once reached here, we must already have a valid route.
  // next_destination_index must also be valid.

  const auto next_stop_lane_point_or =
      data_.multi_stops.ComputeDestinationLanePointByTotalIndex(
          *semantic_map_manager_, map_index_, next_stop_index);
  if (!next_stop_lane_point_or.ok()) {
    return next_stop_lane_point_or.status();
  }

  const auto near_bus_station = [this, &next_stop_lane_point_or](
                                    const Vec2d& current_pos, double radius) {
    SMM_LANE_PROTO_OR_RETURN(lane_proto, *semantic_map_manager_,
                             next_stop_lane_point_or->lane_id(), false);
    const auto global3d = ComputePointOnLane(
        lane_proto->polyline().points(), next_stop_lane_point_or->fraction());
    return route::HaversineDistance({global3d.x(), global3d.y()}, current_pos) <
           radius;
  };

  bool jump_next_stop =
      FLAGS_route_auto_next_stop &&
      near_bus_station(av_global2d, FLAGS_route_arrive_distance) &&
      data_.rms_debug.distance_to_next_stop() < FLAGS_route_arrive_distance;
  // Reroute to next stop if necessary
  data_.rms_debug.set_goto_next_stop(input.res_button_pressed ||
                                     jump_next_stop);
  if (input.res_button_pressed) {
    QLOG(INFO) << "RES BUTTON pressed.";
  }

  if (!data_.rms_debug.goto_next_stop()) {
    return absl::NotFoundError("No routing request, not go to next stop.");
  }
  if (data_.has_route()) {
    int new_next_dest_idx;
    if (data_.multi_stops.HasNextDestinationIndex(next_stop_index,
                                                  &new_next_dest_idx)) {
      data_.rms_debug.set_pending_next_destination_index(new_next_dest_idx);
      ASSIGN_OR_RETURN(
          const auto new_request_proto,
          GenerateAndCheckRoutingRequestProtoToNextStop(
              *semantic_map_manager_, map_index_, *route_param_proto_,
              data_.multi_stops, av_global2d, heading, new_next_dest_idx));

      QLOG(INFO) << "new route to next stop "
                 << new_request_proto.ShortDebugString();
      data_.active_routing_request =
          std::make_unique<PlannerRoutingRequestProto>();
      *(data_.active_routing_request->mutable_init()->mutable_pose()) =
          *input.pose;
      *(data_.active_routing_request->mutable_init()
            ->mutable_routing_request()) = new_request_proto;
      data_.active_routing_request->set_request_id(++data_.request_id);
      context.pred_trajectory_point = std::nullopt;
      return GenerateRoutingResultProto(
          context, coordinate_converter,
          data_.multi_stops.route_restrict_district(),
          data_.active_routing_request.get(), is_bus, &route_core_);
    } else {
      return absl::NotFoundError("No routing request, no next stop.");
    }
  } else {
    return absl::NotFoundError("No routing request, not reached bus station.");
  }
}

absl::Status RouteManager::UpdateRouteFromCurrent(
    bool is_bus, const CompositeLanePathProto& route_path_from_current) {
  SCOPED_QTRACE("RouteManager::UpdateRouteFromCurrent");
  if (!is_bus) {
    route::UpdateAvoidLanes(*semantic_map_manager_,
                            data_.route.route_section_sequence().section_id(),
                            data_.route.mutable_avoid_lanes());
  }

  *data_.route_from_current.mutable_lane_path() = route_path_from_current;

  const auto route_sections = RouteSectionsFromCompositeLanePath(
      *semantic_map_manager_,
      CompositeLanePath(semantic_map_manager_, route_path_from_current));
  route_sections.ToProto(
      data_.route_from_current.mutable_route_section_sequence());

  data_.route_from_current.set_update_id(data_.route.update_id());
  *data_.route_from_current.mutable_avoid_lanes() = data_.route.avoid_lanes();

  return absl::OkStatus();
}

absl::Status RouteManager::UpdateStationsInfo(
    const CompositeLanePathProto& route_path_from_current) {
  SCOPED_QTRACE("RouteManager::UpdateStationsInfo");
  const auto& routing_request =
      *data_.route_from_current.mutable_routing_request();
  if (routing_request.destinations_size() > 0 &&
      route_path_from_current.lane_paths().size() > 0) {
    const auto& lane_path_proto = route_path_from_current.lane_paths()[0];
    if (lane_path_proto.lane_ids().empty()) {
      return absl::InvalidArgumentError(
          "route_path_from_current is invalid, the lane ids is empty.");
    }
    const bool next_destination_is_stop =
        (routing_request.destinations_size() == 1);
    const bool stop_arrived =
        next_destination_is_stop &&
        internal::IsStopArrived(data_.rms_debug.distance_to_next_stop());
    const bool via_point_arrived =
        !next_destination_is_stop &&
        data_.rms_debug.has_distance_to_next_viapoint() &&
        data_.rms_debug.distance_to_next_viapoint() <
            FLAGS_route_arrive_distance;
    if (stop_arrived || via_point_arrived) {
      if (data_.rms_debug.pending_next_destination_index() !=
          data_.rms_debug.reached_destination_index()) {
        data_.rms_debug.set_reached_dest_cnt(
            data_.rms_debug.reached_dest_cnt() + 1);
      }
      data_.rms_debug.set_reached_destination_index(
          data_.rms_debug.pending_next_destination_index());
      if (!next_destination_is_stop) {
        // Not approaching a stop
        data_.rms_debug.set_pending_next_destination_index(
            data_.rms_debug.pending_next_destination_index() + 1);
      }

      QLOG_EVERY_N_SEC(INFO, 10)
          << "Reached No." << data_.rms_debug.reached_dest_cnt()
          << ", is_stop:" << std::boolalpha << next_destination_is_stop
          << ", distance: "
          << (next_destination_is_stop
                  ? data_.rms_debug.distance_to_next_stop()
                  : data_.rms_debug.distance_to_next_viapoint())
          << ", reached destination index:"
          << data_.rms_debug.reached_destination_index();
      if (routing_request.destinations_size() > 1) {
        data_.route_from_current.mutable_routing_request()
            ->mutable_destinations()
            ->erase(data_.route_from_current.mutable_routing_request()
                        ->destinations()
                        .begin());
      }
    }
  }
  return absl::OkStatus();
}

void RouteManager::ResetRoutePathInfo() {
  data_.rms_debug.set_last_update_time(-1);
  data_.composite_index = {0, 0};
  data_.last_lane_point = {mapping::kInvalidElementId, 0.0};
}

bool RouteManager::has_offroad_route() const {
  if (data_.multi_stops.destination_size() > 0 &&
      data_.rms_debug.pending_next_destination_index() <
          data_.multi_stops.destination_size()) {
    const int next_stop_index = data_.multi_stops.GetStopIndex(
        data_.rms_debug.pending_next_destination_index());
    const auto& next_stop = data_.multi_stops.destination(next_stop_index);
    return next_stop.has_off_road();
  }
  return false;
}

void RouteManager::RoutingResetRequest() { RouteResetRequest(&data_); }

void RouteManager::UpdateDistance() {
  const auto& route_from_current = data_.route_from_current;
  if (route_from_current.has_lane_path()) {
    data_.rms_debug.set_distance_to_next_stop(GetCompositeLanePathLength(
        *semantic_map_manager_, route_from_current.lane_path()));
  } else {
    data_.rms_debug.set_distance_to_next_stop(kInvalidDistance);
  }
  if (route_from_current.routing_request().destinations().size() > 1) {
    auto distance_to_next_destination_or = CalcDistanceToNextDestination(
        *semantic_map_manager_, route_from_current, data_.multi_stops,
        data_.rms_debug.pending_next_destination_index());
    if (distance_to_next_destination_or.ok()) {
      data_.rms_debug.set_distance_to_next_viapoint(
          *distance_to_next_destination_or);
    } else {
      QLOG_EVERY_N_SEC(INFO, 3)
          << "Clear via point distance failed: "
          << distance_to_next_destination_or.status().ToString();
      data_.rms_debug.set_distance_to_next_viapoint(kInvalidDistance);
    }
    data_.rms_debug.set_next_destination_id(
        route_from_current.routing_request().destinations()[0].id());
  } else {
    VLOG(3) << "Clear via point settings.";
    data_.rms_debug.clear_next_destination_id();
    data_.rms_debug.clear_distance_to_next_viapoint();
  }
}

void RouteManager::ClearIterationState() {
  data_.rms_debug.set_goto_next_stop(false);
}

void RouteManager::UpdateAlternativeRoute(bool is_bus) {
  SCOPED_QTRACE("RouteManager::UpdateAlternativeRoute");
  if (!FLAGS_route_enable_alternative_path) {
    return;
  }
  if (data_.alter_route_state.has_value()) {
    auto updated_current_route_or = TrackAlternateRoute(
        *semantic_map_manager_, data_.alter_route_state->route_from_current,
        data_.last_lane_point,
        data_.rms_debug.route_tracker().travel_distance() +
            kAlternateRouteTravelError);
    if (updated_current_route_or.ok()) {
      data_.alter_route_state->route_from_current =
          std::move(updated_current_route_or).value();
      route::UpdateAvoidLanes(
          *semantic_map_manager_,
          data_.alter_route_state->route_from_current.route_section_sequence()
              .section_id(),
          data_.alter_route_state->route_from_current.mutable_avoid_lanes());
      return;
    }
    QLOG_EVERY_N_SEC(INFO, 3)
        << "Track alternative route failed, should find a new one.";
  }
  const route::AlternateRouteInput input = {
      .origin = data_.last_lane_point,
      .is_bus = is_bus,
      .look_ahead_dist = kAlternateRouteLookAheadDist,
      .cost_per_meter = kAlternateRouteExtraCostPerMeter,
      .primary_sections = &data_.route_from_current.route_section_sequence(),
      .route_context = {.semantic_map_manager = semantic_map_manager_,
                        .map_index = map_index_,
                        .route_param_proto = route_param_proto_,
                        .request_id = data_.request_id},
      .routing_request = &data_.route_from_current.routing_request(),
      .multi_stops = &data_.multi_stops};

  auto alter_route_or = route::FindAlternateRoute(input, &route_core_);
  if (alter_route_or.ok()) {
    if (!route::IsAlternateRouteDiffPrimary(
            data_.route_from_current.route_section_sequence(),
            alter_route_or->route_from_current.route_section_sequence())) {
      QLOG_EVERY_N_SEC(INFO, 10)
          << "Can not find a different alternative route, "
             "please drive along primary route.";
      data_.alter_route_state = std::nullopt;
      return;
    }
    QLOG_EVERY_N_SEC(INFO, 10) << "Find a new alternative route.";
    const double roundabout_dist =
        AccumRouteSectionsLengthBetween(
            *semantic_map_manager_,
            alter_route_or->route_from_current.route_section_sequence(), 0,
            alter_route_or->route_from_current.route_section_sequence()
                .section_id_size()) -
        AccumRouteSectionsLengthBetween(
            *semantic_map_manager_,
            data_.route_from_current.route_section_sequence(), 0,
            data_.route_from_current.route_section_sequence()
                .section_id_size());
    data_.alter_route_state = {
        .route = std::move(alter_route_or->route),
        .route_from_current = std::move(alter_route_or->route_from_current),
        .routing_result_proto = std::move(alter_route_or->routing_result_proto),
        .roundabout_distance = roundabout_dist};
  } else {
    QLOG_EVERY_N_SEC(INFO, 3)
        << "Can not find a new alternative route, retry later.";
    data_.alter_route_state = std::nullopt;
  }
}

void RouteManager::InjectState(RouteManagerState state) {
  data_ = std::move(state);
}

absl::Status RouteManager::Reroute(
    const mapping::v2::SemanticMapManager& semantic_map_manager,
    const route::MapIndex& map_index,
    const CoordinateConverter& coordinate_converter,
    const RouteParamProto& route_param_proto, const PoseProto& pose,
    const RoutingRequestProto& origin_request,
    const MultipleStopsRequest& multi_stops,
    const RouteSectionSequenceProto& avoid_route_sections, int64_t request_id,
    bool is_bus, route::RouteCore* route_core, RouteManagerState* data) {
  auto lane_point_or =
      FindLanePointFromPose(semantic_map_manager, map_index,
                            coordinate_converter, route_param_proto, pose);
  if (!lane_point_or.ok()) {
    last_error_ = route::RerouteError(lane_point_or.status().message());
    return lane_point_or.status();
  }

  route::AlternateRouteInput reroute_input = {
      .origin = *lane_point_or,
      .is_bus = is_bus,
      .look_ahead_dist = kAlternateRouteLookAheadDist,
      .cost_per_meter = kAlternateRouteExtraCostPerMeter,
      .primary_sections = &avoid_route_sections,
      .route_context = {.semantic_map_manager = &semantic_map_manager,
                        .map_index = &map_index,
                        .route_param_proto = &route_param_proto,
                        .request_id = request_id},
      .routing_request = &origin_request,
      .multi_stops = &multi_stops};
  VLOG(3) << "reroute request:" << reroute_input.route_context.request_id;
  auto reroute_output_or = route::FindAlternateRoute(reroute_input, route_core);
  if (reroute_output_or.ok()) {
    VLOG(3) << "Update reroute result.";
    data->route = reroute_output_or->route;
    data->route_from_current = reroute_output_or->route_from_current;
    data->routing_result_proto =
        std::move(reroute_output_or->routing_result_proto);
    ResetRoutePathInfo();
    data->rms_debug.set_is_internal_reroute(true);
    QLOG_EVERY_N_SEC(INFO, 3)
        << "Reroute success." << reroute_input.route_context.request_id;
  } else {
    last_error_ = route::RerouteError(reroute_output_or.status().message());
    VLOG(3) << "reroute error:" << last_error_.ToString();
    return reroute_output_or.status();
  }
  return absl::OkStatus();
}

}  // namespace planner
}  // namespace qcraft
