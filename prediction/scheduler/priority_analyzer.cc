#include "onboard/prediction/scheduler/priority_analyzer.h"

#include <algorithm>
#include <cmath>  // for fabs
#include <string>
#include <vector>  // for vector

#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "glog/logging.h"

#include "onboard/container/strong_int.h"  // for NullStrongIntValidator
#include "onboard/global/trace.h"
#include "onboard/maps/maps_common.h"  // for LaneBoundaryIncidenceInfo, LaneInfo, LaneBoundaryInfo
#include "onboard/maps/proto/semantic_map.pb.h"  // for LaneBoundaryProto, LaneProto
#include "onboard/maps/semantic_map_defs.h"      // for ElementId
#include "onboard/math/frenet_common.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/planner_semantic_map_manager.h"  // for PlannerSemanticMapManager
#include "onboard/planner/router/drive_passage.h"
#include "onboard/prediction/container/av_context.h"
#include "onboard/prediction/container/object_history_span.h"
#include "onboard/prediction/container/prediction_object.h"
#include "onboard/prediction/prediction_util.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/utils/elements_history.h"

namespace qcraft {
namespace prediction {
namespace {
// Align with noa detection range requirement.
constexpr double kScanFrontDistance = 120.0;                // m.
constexpr double kScanBackDistance = 30.0;                  // m.
constexpr double kScanAreaWidth = 12.0;                     // m.
constexpr double kNearLaneDistanceThreshold = 4.0;          // m.
constexpr double kNearIntersectionDistanceThreshold = 2.0;  // m.
constexpr double kBackDistIgnorePed = -4.0;                 // m.
constexpr double kNearDrivePassageDistanceThreshold = 7.0;  // m.
constexpr double kOpposingObjectAvHeadingDiff = M_PI / 2.0;

bool IsIgnoredObject(const ObjectHistory& obj,
                     const ObjectPredictionScenario& scenario,
                     const Box2d& scan_area, const Vec2d& av_pos,
                     const Vec2d& av_heading_vec, std::string* annotation) {
  const auto hist = obj.GetHistory();
  const bool is_in_scan_box = scan_area.IsPointIn(hist.back().val.pos());
  const bool is_on_road = (scenario.road_status() == ORS_ON_ROAD);
  const Vec2d ego_to_obstacle_vec = hist.back().val.pos() - av_pos;
  const double s = ego_to_obstacle_vec.Dot(av_heading_vec);
  bool maybe_vru_in_front_near_lanes = false;
  if (scenario.has_abs_dist_to_nearest_lane()) {
    maybe_vru_in_front_near_lanes =
        (s > kBackDistIgnorePed) && MaybeVRU(hist.type()) &&
        (scenario.abs_dist_to_nearest_lane() < kNearLaneDistanceThreshold);
  }
  bool is_near_intersection = (scenario.intersection_status() ==
                               ObjectIntersectionStatus::OIS_IN_INTERSECTION);
  if (scenario.has_abs_dist_to_nearest_intersection()) {
    is_near_intersection = (scenario.abs_dist_to_nearest_intersection() <
                            kNearIntersectionDistanceThreshold) ||
                           is_near_intersection;
  }
  *annotation = absl::StrCat(
      "obj ", hist.id(), " is in scan box: ", is_in_scan_box,
      "; is on road: ", is_on_road,
      "; is ped like in front near lanes: ", maybe_vru_in_front_near_lanes,
      "; is near intersection: ", is_near_intersection);
  VLOG(2) << *annotation;
  const bool need_consider =
      (is_in_scan_box || is_on_road || maybe_vru_in_front_near_lanes ||
       is_near_intersection);
  return !need_consider;
}

bool IsIrrelevantObject(const ObjectHistory& obj,
                        const ObjectPredictionScenario& scenario,
                        const planner::DrivePassage& dp,
                        std::string* annotation) {
  const auto hist = obj.GetHistory();
  const auto& obj_pos = hist.back().val.pos();
  const auto& obj_v = hist.back().val.v();

  const auto frenet_or = dp.QueryUnboundedFrenetCoordinateAt(obj_pos);
  if (!frenet_or.ok()) {
    return false;
  }
  const bool is_in_intersection =
      (scenario.intersection_status() ==
       ObjectIntersectionStatus::OIS_IN_INTERSECTION);
  if (is_in_intersection) {
    return false;
  }
  const auto& obj_frenet = frenet_or.value();
  const auto bound_response = dp.QueryEnclosingLaneBoundariesAtS(obj_frenet.s);
  constexpr double kDefaultHalfLaneWidth = 2.0;
  const double right_bound = bound_response.right.has_value()
                                 ? bound_response.right->lat_offset
                                 : -kDefaultHalfLaneWidth;
  const double left_bound = bound_response.left.has_value()
                                ? bound_response.left->lat_offset
                                : kDefaultHalfLaneWidth;
  double dist_to_bound = 0.0;
  if (obj_frenet.l < right_bound) {
    dist_to_bound = right_bound - obj_frenet.l;
  } else if (obj_frenet.l > left_bound) {
    dist_to_bound = obj_frenet.l - left_bound;
  }

  dist_to_bound =
      std::min(dist_to_bound,
               std::max(std::fabs(obj_frenet.l) - kDefaultHalfLaneWidth, 0.0));
  *annotation =
      absl::StrCat("obj ", hist.id(), " right bound: ", right_bound,
                   "left_bound: ", left_bound, "obj l : ", obj_frenet.l,
                   "; dist to bound: ", dist_to_bound, " max reachable l ",
                   obj_v * kComfortableHorizon);
  VLOG(2) << *annotation;
  if (dist_to_bound > kNearDrivePassageDistanceThreshold &&
      obj_v * kComfortableHorizon < dist_to_bound) {
    return true;
  }
  return false;
}

bool IsOpposingOffroadObject(const PredictionContext& prediction_context,
                             const ObjectPredictionScenario& scenario,
                             const ObjectHistory& obj, double av_heading,
                             std::string* annotation) {
  const auto hist = obj.GetHistory();
  const auto& obj_heading = hist.back().val.heading();
  const auto& obj_pos = hist.back().val.pos();
  const planner::PlannerSemanticMapManager& psmm =
      prediction_context.semantic_map_manager();
  bool is_off_curb = false;
  if (scenario.road_status() == ORS_ON_ROAD) {
    const auto* lane_info =
        psmm.FindLaneInfoOrNull(mapping::ElementId(scenario.nearest_lane_id()));
    if (lane_info == nullptr) {
      return false;
    }
    if (lane_info->Type() != mapping::LaneProto::EMERGENCY) {
      return false;
    }
    if (lane_info->lane_boundaries_on_left.empty()) {
      return false;
    }
    const auto* lane_boundary_info = psmm.FindLaneBoundaryByIdOrNull(
        lane_info->lane_boundaries_on_left[0].lane_boundary_id);
    if (lane_boundary_info == nullptr) {
      return false;
    }
    if (lane_boundary_info->type != mapping::LaneBoundaryProto::CURB) {
      return false;
    }
    Vec2d lane_boundary_proj_point;
    Vec2d lane_proj_point;
    double lane_boundary_dist = 0.0;
    double lane_dist = 0.0;
    if (psmm.GetLaneBoundaryProjection(
            obj_pos, lane_info->lane_boundaries_on_left[0].lane_boundary_id,
            /*fraction=*/nullptr, &lane_boundary_proj_point,
            &lane_boundary_dist, /*segment=*/nullptr) &&
        psmm.GetLaneProjection(obj_pos,
                               mapping::ElementId(scenario.nearest_lane_id()),
                               /*fraction=*/nullptr, &lane_proj_point,
                               &lane_dist, /*segment=*/nullptr)) {
      if (lane_boundary_dist > lane_dist) {
        return false;
      }
      const Vec2d lane_boundary_vec(lane_boundary_proj_point - obj_pos);
      const Vec2d lane_vec(lane_proj_point - obj_pos);
      const double angle_diff =
          AngleDifference(lane_boundary_vec.Angle(), lane_vec.Angle());
      if (std::abs(angle_diff) > M_PI_2) {
        return false;
      }
      is_off_curb = true;
    }
  }
  const bool is_off_road =
      (scenario.road_status() == ORS_OFF_ROAD) || is_off_curb;
  const auto* autonomy_state = prediction_context.autonomy_state();
  const double heading_diff =
      std::abs(NormalizeAngle(av_heading - obj_heading));
  if (autonomy_state == nullptr) {
    return false;
  }
  if (is_off_road && MustReceiveHDMapForPrediction(*autonomy_state) &&
      heading_diff > kOpposingObjectAvHeadingDiff) {
    *annotation = absl::StrCat(
        "obj ", hist.id(), "; Is off road: ", is_off_road,
        ", Must Receive HDMap For Prediction", ", heading diff: ", heading_diff,
        ", av heading: ", av_heading, ", obj heading: ", obj_heading);
    return true;
  }
  return false;
}
}  // namespace

std::map<ObjectIDType, ObjectPredictionPriorityInfo> AnalyzePriorities(
    const PredictionContext& prediction_context,
    absl::Span<const ObjectHistory* const> objs_to_predict,
    const std::map<ObjectIDType, ObjectPredictionScenario>& obj_scenarios) {
  FUNC_QTRACE();
  std::map<ObjectIDType, ObjectPredictionPriorityInfo> res;
  const auto& av =
      prediction_context.av_context().GetAvObjectHistory().back().val;
  const Vec2d& av_pos = av.pos();
  const Vec2d av_heading_vec = Vec2d::UnitFromAngle(av.heading());
  const double scan_length = kScanFrontDistance + kScanBackDistance;
  const double av_dist_to_center = scan_length * 0.5 - kScanBackDistance;
  const Box2d scan_area(av_pos + av_dist_to_center * av_heading_vec,
                        av.heading(), scan_length, kScanAreaWidth);
  const auto* dp = prediction_context.av_drive_passage();
  for (int i = 0; i < objs_to_predict.size(); ++i) {
    const auto* obj = objs_to_predict[i];
    const auto& scenario = obj_scenarios.at(obj->id());
    const auto& id = obj->id();
    std::string priority_annotation;
    const bool is_ignored =
        IsIgnoredObject(*obj, scenario, scan_area, av_pos, av_heading_vec,
                        &priority_annotation);
    if (is_ignored) {
      res[id].priority = OPP_P3;  // Objects that can be ignored.
      res[id].priority_annotation = priority_annotation;
      continue;
    }
    const bool is_irrelevant =
        (dp != nullptr &&
         IsIrrelevantObject(*obj, scenario, *dp, &priority_annotation));
    if (is_irrelevant) {
      res[id].priority = OPP_P3;  // Objects that can be ignored.
      res[id].priority_annotation = priority_annotation;
      continue;
    }
    const bool is_opposing_off_road_object = IsOpposingOffroadObject(
        prediction_context, scenario, *obj, av.heading(), &priority_annotation);
    if (is_opposing_off_road_object) {
      res[id].priority = OPP_P3;  // Objects that can be ignored.
      res[id].priority_annotation = priority_annotation;
      continue;
    }
    if (obj->object_proto().is_radar_only_object()) {
      res[id].priority = OPP_P2;  // Low priority objects.
      res[id].priority_annotation = "Radar only";
      continue;
    }
    res[id].priority = OPP_P1;  // Normal priority objects.
    res[id].priority_annotation = "Normal";
  }
  return res;
}
}  // namespace prediction
}  // namespace qcraft
