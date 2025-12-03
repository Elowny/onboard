#include "onboard/prediction/scheduler/scenario_analyzer.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <ostream>
#include <string>
#include <vector>

#include "absl/status/statusor.h"
#include "glog/logging.h"

#include "onboard/global/buffered_logger.h"
#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/maps_common.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/geometry/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/util/planner_semantic_map_util.h"
#include "onboard/planner/util/spatial_search_util.h"
#include "onboard/prediction/container/object_history_span.h"
#include "onboard/prediction/container/prediction_object.h"
#include "onboard/utils/elements_history.h"

namespace qcraft {
namespace prediction {
namespace {
constexpr double kDistanceEpsilon = 1e-2;  // m.
}  // namespace
std::map<ObjectIDType, ObjectPredictionScenario> AnalyzeScenarios(
    const PredictionContext& prediction_context,
    absl::Span<const ObjectHistory* const> objs_to_predict) {
  FUNC_QTRACE();
  const auto& semantic_map_mgr = prediction_context.semantic_map_manager();
  std::map<ObjectIDType, ObjectPredictionScenario> scenarios;
  for (int i = 0; i < objs_to_predict.size(); ++i) {
    const auto hist = objs_to_predict[i]->GetHistory();
    const std::string& id = objs_to_predict[i]->id();
    scenarios[id] = AnalyzeScenarioWithSemanticMapAndObjectProto(
        semantic_map_mgr, hist.back().val.object_proto());
    VLOG(2) << "Scenario: " << id << " " << scenarios[id].DebugString();
  }
  return scenarios;
}

ObjectPredictionScenario AnalyzeScenarioWithSemanticMapAndObjectProto(
    const planner::PlannerSemanticMapManager& semantic_map_mgr,
    const ObjectProto& obj_proto) {
  ObjectPredictionScenario scenario;
  const Vec2d obj_pos = Vec2dFromProto(obj_proto.pos());
  const Box2d bbox(obj_proto.bounding_box());
  scenario.set_abs_dist_to_nearest_lane(std::numeric_limits<double>::max());
  scenario.set_abs_dist_to_nearest_intersection(
      std::numeric_limits<double>::max());

  if (const auto nearest_lane_point_or =
          FindClosestLanePointToSmoothPointAtLevel(semantic_map_mgr.GetLevel(),
                                                   semantic_map_mgr, obj_pos);
      !nearest_lane_point_or.ok()) {
    scenario.set_road_status(ObjectRoadStatus::ORS_OFF_ROAD);
  } else {
    scenario.set_nearest_lane_id(nearest_lane_point_or->lane_id().value());
    // Check if object bbox is on road.
    bool is_off_road = true;
    double abs_dist_to_lane = std::numeric_limits<double>::max();
    for (const auto& corner_pt : bbox.GetCornersCounterClockwise()) {
      const double invasion_dist = planner::GetDistOfPointInvasionLaneSupport(
          corner_pt, semantic_map_mgr, nearest_lane_point_or->lane_id());
      if (invasion_dist < 0) {  // Not on lane.
        abs_dist_to_lane = std::min(abs_dist_to_lane, std::fabs(invasion_dist));
      } else {  // Already in lane.
        abs_dist_to_lane = 0.0;
        is_off_road = false;
        break;
      }
    }
    scenario.set_abs_dist_to_nearest_lane(abs_dist_to_lane);
    if (is_off_road) {
      scenario.set_road_status(ObjectRoadStatus::ORS_OFF_ROAD);
    } else {
      QCHECK_LE(abs_dist_to_lane, 1e-3);
      scenario.set_road_status(ObjectRoadStatus::ORS_ON_ROAD);
    }
  }
  const auto* intersection_ptr =
      semantic_map_mgr.GetNearestIntersectionInfoAtLevel(
          semantic_map_mgr.GetLevel(), obj_pos);
  if (intersection_ptr != nullptr) {
    scenario.set_nearest_intersection_id(intersection_ptr->id.value());
    bool is_out_intersection = true;
    double abs_dist2_to_intersection = std::numeric_limits<double>::max();
    // Check if object bbox is in intersection.
    for (const auto& corner_pt : bbox.GetCornersCounterClockwise()) {
      const double cur_dist2 =
          intersection_ptr->polygon_smooth.DistanceSquareTo(corner_pt);
      abs_dist2_to_intersection =
          std::min(abs_dist2_to_intersection, cur_dist2);
      if (cur_dist2 < Sqr(kDistanceEpsilon)) {
        is_out_intersection = false;
        break;
      }
    }
    scenario.set_abs_dist_to_nearest_intersection(
        std::sqrt(abs_dist2_to_intersection));
    if (is_out_intersection) {
      scenario.set_intersection_status(
          ObjectIntersectionStatus::OIS_OUT_INTERSECTION);
    } else {
      scenario.set_intersection_status(
          ObjectIntersectionStatus::OIS_IN_INTERSECTION);
    }
  } else {
    scenario.set_intersection_status(
        ObjectIntersectionStatus::OIS_OUT_INTERSECTION);
  }
  return scenario;
}
}  // namespace prediction
}  // namespace qcraft
