#include "onboard/ml_planner/ml_cruise_task.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <ostream>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "absl/status/statusor.h"
#include "absl/strings/str_join.h"
#include "absl/types/span.h"
#include "glog/logging.h"

#include "onboard/global/trace.h"
// IWYU pragma: no_include <google/protobuf/repeated_ptr_field.h>

#include "onboard/container/strong_int.h"
#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/maps/lane_path.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/maps_common.h"
#include "onboard/maps/proto/lane_path.pb.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/ml_planner/ml_planner_input.h"
#include "onboard/ml_planner/multi_conditions_ml_cruise_planner.h"
#include "onboard/ml_planner/multi_conditions_ml_cruise_planner_input.h"
#include "onboard/planner/ml/context_feature_extractor/context_feature.h"
#include "onboard/planner/ml/context_feature_extractor/context_feature_extractor.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/util/lane_point_util.h"
#include "onboard/planner/util/spatial_search_util.h"
#include "onboard/prediction/prediction_util.h"
#include "onboard/prediction/util/drive_passage_util.h"
#include "onboard/prediction/util/lane_path_finder.h"
#include "onboard/proto/positioning.pb.h"

namespace qcraft::mlplanner {

namespace {

// For find lane paths.
constexpr double kDpFrontScanDistance = 40;
constexpr double kDpBackScanDistance = 20;
constexpr double kDpSideScanHalfWidth = 10;
constexpr int kDpMaxNum = 16;

constexpr double kDpCacheFrontLength = 180;
constexpr double kDpCacheBackLength = 80;

std::vector<mapping::LanePath> FilterAndTruncateLanePaths(
    absl::Span<const mapping::LanePath> lps,
    const planner::PlannerSemanticMapManager& semantic_map_manager,
    const Vec2d& ego_pos, double ego_heading, int max_num) {
  struct LanePathInfo {
    double square_dis;
    const mapping::LanePath* lp;
    mapping::LanePoint lane_point;
    Vec2d closest_point_on_lane;
  };
  SCOPED_QTRACE("MlPlannerModule::FilterAndTruncateLanePaths");
  std::vector<LanePathInfo> infos;
  infos.reserve(lps.size());
  constexpr double kDpSearchRadiusThreshold = 200.0;
  for (const auto& lp : lps) {
    Vec2d closest_pt;
    auto closest_lane_point_or = planner::
        FindClosestLanePointToSmoothPointWithHeadingBoundAlongLanePathAtLevel(
            semantic_map_manager.GetLevel(), semantic_map_manager, ego_pos, lp,
            ego_heading,
            /*heading_penalty_weight=*/0.0,
            /*closest_point_on_lane=*/&closest_pt,
            /*cutoff_distance=*/kDpSearchRadiusThreshold);
    // The projection must be valid.
    if (!closest_lane_point_or.ok()) {
      continue;
    }
    const double square_dis = ego_pos.DistanceSquareTo(closest_pt);
    infos.push_back(
        LanePathInfo{.square_dis = square_dis,
                     .lp = &lp,
                     .lane_point = std::move(closest_lane_point_or).value(),
                     .closest_point_on_lane = std::move(closest_pt)});
  }
  std::sort(infos.begin(), infos.end(),
            [](const LanePathInfo& info1, const LanePathInfo& info2) {
              return info1.square_dis < info2.square_dis;
            });
  if (infos.size() > max_num) {
    infos.resize(max_num);
  }
  std::vector<mapping::LanePath> res;
  res.reserve(infos.size());
  for (const auto& info : infos) {
    const double heading_diff =
        NormalizeAngle(ego_heading - ComputeLanePointTangent(
                                         semantic_map_manager, info.lane_point)
                                         .FastAngle());
    if (std::fabs(heading_diff) < M_PI_2) {
      res.push_back(prediction::PruneLanePathByLength(
          semantic_map_manager, *info.lp, info.lane_point, kDpCacheFrontLength,
          kDpCacheBackLength));
    } else {
      res.push_back(prediction::PruneLanePathByLength(
          semantic_map_manager, *info.lp, info.lane_point, kDpCacheBackLength,
          kDpCacheFrontLength));
    }
  }
  return res;
}

void FilterSameStartLanePaths(std::vector<mapping::LanePath>* lane_paths) {
  mapping::ElementId last_start_lane_id;
  auto it = lane_paths->begin();
  if (it != lane_paths->end()) {
    last_start_lane_id = it->lane_id(0);
    ++it;
  }
  while (it != lane_paths->end()) {
    if (it->lane_id(0) == last_start_lane_id) {
      it = lane_paths->erase(it);
    } else {
      last_start_lane_id = it->lane_id(0);
      ++it;
    }
  }
}

std::vector<mapping::LanePath> FindAllFeasibleLanePaths(
    const planner::PlannerSemanticMapManager& planner_semantic_map_manager,
    const Vec2d& ego_pos, double ego_heading, double front_dist,
    double back_dist, double side_dist, int max_dp_num) {
  SCOPED_QTRACE("MlPlannerModule::FindAllFeasibleLanePaths");
  const auto region_box = prediction::GetRegionBox(
      ego_pos, ego_heading, front_dist, back_dist, side_dist);
  const auto level_id = planner_semantic_map_manager.GetLevel();
  const auto lanes = planner_semantic_map_manager.GetLanesInfoAtLevel(
      level_id, region_box.center(), region_box.diagonal() * 0.5);
  constexpr int kMaxLaneNum = 128;
  const auto filtered_lanes =
      prediction::GetElementsInBox2d(lanes, region_box, kMaxLaneNum);
  std::set<mapping::ElementId> lane_ids;
  for (const auto* lane : filtered_lanes) {
    QCHECK(lane->id != mapping::kInvalidElementId);
    lane_ids.insert(lane->id);
  }
  const auto filtered_ids = prediction::FilterLanesByConsecutiveRelationship(
      planner_semantic_map_manager, lane_ids, /*filter_virtual=*/false);
  VLOG(2) << "Lane numbers: "
          << "Before filtering " << lane_ids.size() << " ,after filtering "
          << filtered_ids.size();
  VLOG(2) << "Original Lane Ids: " << absl::StrJoin(lane_ids, ",");
  VLOG(2) << "Filtered Lane Ids: " << absl::StrJoin(filtered_ids, ",");

  std::vector<mapping::LanePath> lane_paths;
  lane_paths.reserve(2 * filtered_ids.size());
  for (const auto& lane_id : filtered_ids) {
    auto lps = prediction::SearchLanePath(ego_pos, planner_semantic_map_manager,
                                          lane_id, kDpCacheFrontLength,
                                          /*is_reverse_driving=*/false);
    for (auto& lp : lps) {
      lane_paths.push_back(std::move(lp));
    }
  }
  VLOG(2) << "Feasible lane path total num: " << lane_paths.size();
  lane_paths =
      FilterAndTruncateLanePaths(lane_paths, planner_semantic_map_manager,
                                 ego_pos, ego_heading, max_dp_num);
  VLOG(2) << "Filtered feasible lane path total num: " << lane_paths.size();
  FilterSameStartLanePaths(&lane_paths);
  VLOG(2) << "After filtered by same start lane path num: "
          << lane_paths.size();
  return lane_paths;
}

}  // namespace

MLPlannerStatus RunMLCruiseTask(const MLCruiseTaskInput& input,
                                MlPlannerOutputProto* output,
                                MlPlannerDebugProto* ml_planner_debug) {
  SCOPED_QTRACE("MlPlannerModule::RunMLCruiseTask");
  // ------------------------------------------------------------------
  // ---------------- Construct planner-ml context feature ------------
  // ------------------------------------------------------------------

  std::shared_ptr<const planner::ml::ContextFeature> context_feature = nullptr;

  const planner::ml::ContextFeatureExtractionInput
      context_feature_extraction_input{
          .av_context = input.ml_planner_input.av_context.get(),
          .traffic_light_states =
              input.ml_planner_input.traffic_light_states.get(),
          .object_manager = input.object_manager.get(),
          .real_objects = input.ml_planner_input.real_objects,
          .virtual_objects = input.ml_planner_input.virtual_objects,
          .psmm = input.ml_planner_input.planner_semantic_map_manager.get(),
      };
  context_feature = std::make_shared<const planner::ml::ContextFeature>(
      planner::ml::ExtractContextFeature(context_feature_extraction_input));

  const auto& av_pose = input.ml_planner_input.pose;
  auto all_feasible_lane_paths = FindAllFeasibleLanePaths(
      *input.ml_planner_input.planner_semantic_map_manager,
      Vec2d(av_pose->pos_smooth().x(), av_pose->pos_smooth().y()),
      av_pose->yaw(), kDpFrontScanDistance, kDpBackScanDistance,
      kDpSideScanHalfWidth, kDpMaxNum);

  for (const auto& lane_path : all_feasible_lane_paths) {
    mapping::LanePathProto lane_path_proto;
    lane_path.ToProto(&lane_path_proto);
    ml_planner_debug->mutable_captain_net_debug()
        ->mutable_all_feasible_lane_path()
        ->Add(std::move(lane_path_proto));
  }

  auto ml_planner_status = RunMultiConditionsMLCruisePlanner(
      MultiConditionsMLCruisePlannerInput{
          .ml_planner_input = input.ml_planner_input,
          .context_feature = context_feature,
          .lane_paths = &all_feasible_lane_paths,
      },
      output, ml_planner_debug);

  return ml_planner_status;
}

}  // namespace qcraft::mlplanner
