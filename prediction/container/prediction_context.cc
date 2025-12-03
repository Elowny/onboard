#include "onboard/prediction/container/prediction_context.h"

#include <algorithm>  // for clamp, max
#include <cmath>      // for M_PI
#include <memory>   // for unique_ptr, make_unique, shared_ptr, __shared_ptr...
#include <utility>  // for move

#include "absl/status/status.h"
#include "absl/status/statusor.h"                // for StatusOr
#include "google/protobuf/repeated_ptr_field.h"  // for RepeatedPtrField, RepeatedPtrIterator

#include "onboard/global/trace.h"  // for ScopedTrace, SCOPED_QTRACE, FUNC_QTRACE
#include "onboard/lite/check.h"    // for QCHECK_NOTNULL
#include "onboard/math/geometry/proto/affine_transformation.pb.h"  // for Vec3dProto
#include "onboard/math/vec.h"  // for Vec2d, MatrixBase::cwiseAbs2, MatrixBase::norm
#include "onboard/planner/router/navi/route_navi_info.h"  // for RouteNaviInfo
#include "onboard/planner/router/route_manager_output.h"  // for RouteManagerOutput
#include "onboard/planner/router/route_sections.h"        // for RouteSections
#include "onboard/planner/router/route_sections_util.h"  // for SpliceRouteSections
#include "onboard/planner/router/route_util.h"  // for IsValidRouteOutputProto
#include "onboard/prediction/container/map_cache_builder.h"
#include "onboard/prediction/container/prediction_input.h"  // for PredictionInput
#include "onboard/prediction/container/prediction_object.h"  // for PredictionObject
#include "onboard/prediction/prediction_util.h"
#include "onboard/prediction/util/drive_passage_util.h"  // for BuildAvDrivePassageWithNearestLane, BuildAvDriveP...
#include "onboard/proto/positioning.pb.h"    // for PoseProto
#include "onboard/utils/elements_history.h"  // for Node
namespace qcraft {
namespace prediction {
namespace {
// Build lane boundary cache.
constexpr double kLBCFrontScanDistance = 50;
constexpr double kLBCBackScanDistance = 10;
constexpr double kLBCSideScanHalfWidth = 10;
constexpr double kLBCSamplingStep = 4.0;
// Build drive passage cache.
constexpr double kDpFrontScanDistance = 100;
constexpr double kDpBackScanDistance = 40;
constexpr double kDpSideScanHalfWidth = 20;
constexpr double kDpSamplingStep = 5.0;
constexpr int kDpMaxNum = 16;

constexpr double kMaxHeadingDiff = M_PI / 6.0;
constexpr double kAvDrivePassageFrontLength = 200.0;  // m.
constexpr double kAvDrivePassageBackLength = 60.0;    // m.

absl::StatusOr<planner::DrivePassage> BuildAvDrivePassage(
    const planner::PlannerSemanticMapManager& semantic_map_manager,
    const planner::LaneBoundaryCache& cache, const AvContext& av_context,
    const AutonomyStateProto* autonomy_state,
    const planner::RouteManagerOutputProto* route_out_proto) {
  SCOPED_QTRACE("BuildDrivePassageWithRouting");
  if (autonomy_state == nullptr || route_out_proto == nullptr) {
    return absl::FailedPreconditionError("No autonomy state or routing.");
  }
  const auto& av = av_context.GetAvObjectHistory().back().val;
  const Vec2d& av_pos = av.pos();
  // Build the AV drive passage with routing information
  if (!planner::IsValidRouteOutputProto(*route_out_proto)) {
    return absl::FailedPreconditionError("Routing invalid.");
  }
  planner::RouteManagerOutput route_mgr_output;
  route_mgr_output.FromProto(*route_out_proto);

  const auto total_route_sections = [&route_mgr_output]() {
    if (route_mgr_output.route_navi_info.back_extend_sections.empty()) {
      return route_mgr_output.route_sections_from_current;
    }
    auto total_sections = planner::SpliceRouteSections(
        route_mgr_output.route_navi_info.back_extend_sections,
        route_mgr_output.route_sections_from_current);
    return total_sections.ok() ? *total_sections
                               : route_mgr_output.route_sections_from_current;
  };
  const planner::RouteSections sections = total_route_sections();
  if (sections.empty() || av_context.GetAvObjectHistory().empty()) {
    return absl::FailedPreconditionError("No section or no av history.");
  }
  return BuildAvDrivePassageWithRouting(
      semantic_map_manager, cache, sections, av_pos, kAvDrivePassageBackLength,
      kAvDrivePassageFrontLength, kAvDrivePassageBackLength);
}

}  // namespace
PredictionContext::PredictionContext(const PredictionInput& input)
    : prediction_init_time_(input.prediction_init_time),
      objects_history_(input.objects_history.get()),
      obj_long_term_hist_mgr_(input.long_term_objects_history.get()),
      av_context_(input.av_context.get()),
      veh_geom_params_(input.veh_geom_params),
      semantic_map_manager_(input.semantic_map_manager),
      conflict_resolver_params_(input.conflict_resolver_params),
      autonomy_state_(input.autonomy_state.get()),
      route_manager_output_(input.route_manager_output.get()) {
  // The following variables cannot be nullptr.
  QCHECK_NOTNULL(objects_history_);
  QCHECK_NOTNULL(obj_long_term_hist_mgr_);
  QCHECK_NOTNULL(av_context_);
  QCHECK_NOTNULL(veh_geom_params_);
  QCHECK_NOTNULL(semantic_map_manager_);
  QCHECK_NOTNULL(conflict_resolver_params_);

  // route_manager_output can be nullptr.
  // autonomy_state can be nullptr.
  // av_drive_passage can be nullptr.

  traffic_light_manager_.UpdateTlStateMap(*semantic_map_manager_,
                                          *input.traffic_light_states);
  const auto& av_pose = *input.pose;
  // Build map lane cache.
  lane_boundary_cache_ = BuildLaneBoundaryCache(
      *semantic_map_manager_,
      Vec2d(av_pose.pos_smooth().x(), av_pose.pos_smooth().y()), av_pose.yaw(),
      kLBCFrontScanDistance, kLBCBackScanDistance, kLBCSideScanHalfWidth,
      kLBCSamplingStep);
  // If state is NOA, we do not ignore virtual lane.
  bool filter_virtual = false;
  if (autonomy_state_ != nullptr) {
    filter_virtual = !MustReceiveHDMapForPrediction(*autonomy_state_);
  }
  auto [passages, cache] = BuildDrivePassageCache(
      *semantic_map_manager_, lane_boundary_cache_,
      Vec2d(av_pose.pos_smooth().x(), av_pose.pos_smooth().y()), av_pose.yaw(),
      kDpFrontScanDistance, kDpBackScanDistance, kDpSideScanHalfWidth,
      kDpSamplingStep, kDpMaxNum, filter_virtual);
  drive_passages_ = std::move(passages);
  drive_passage_cache_ = std::move(cache);

  av_drive_passage_ = nullptr;
  auto drive_passage_with_routing =
      BuildAvDrivePassage(*semantic_map_manager_, lane_boundary_cache_,
                          *av_context_, autonomy_state_, route_manager_output_);
  if (drive_passage_with_routing.ok()) {
    av_drive_passage_ = std::make_unique<planner::DrivePassage>(
        std::move(drive_passage_with_routing).value());
    return;
  }
  {
    SCOPED_QTRACE("BuildDrivePassageWithNearestLane");
    auto drive_passage_with_nearest_lane = BuildAvDrivePassageWithNearestLane(
        *semantic_map_manager_, lane_boundary_cache_,
        Vec2d(av_pose.pos_smooth().x(), av_pose.pos_smooth().y()),
        av_pose.yaw(), kMaxHeadingDiff, kAvDrivePassageBackLength,
        kAvDrivePassageFrontLength, kAvDrivePassageBackLength);
    if (drive_passage_with_nearest_lane.ok()) {
      av_drive_passage_ = std::make_unique<planner::DrivePassage>(
          std::move(drive_passage_with_nearest_lane).value());
    }
  }
}

std::vector<const ObjectHistory*> PredictionContext::GetObjectsToPredict(
    const ObjectsProto& objects_proto) const {
  FUNC_QTRACE();
  std::vector<const ObjectHistory*> object_to_predict;
  object_to_predict.reserve(objects_proto.objects().size());
  for (const auto& object : objects_proto.objects()) {
    const auto& object_id = object.id();
    const auto* hist = objects_history_->FindOrNull(object_id);
    if (hist == nullptr) {
      continue;
    }
    object_to_predict.push_back(hist);
  }
  return object_to_predict;
}
}  // namespace prediction
}  // namespace qcraft
