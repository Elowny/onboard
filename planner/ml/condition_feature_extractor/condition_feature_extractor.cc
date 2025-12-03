#include "onboard/planner/ml/condition_feature_extractor/condition_feature_extractor.h"

#include <algorithm>  // for max
#include <utility>

#include "absl/status/status.h"  // for Status, FailedPreconditionError
#include "absl/types/span.h"     // for Span

#include "onboard/maps/maps_common.h"  // for LaneInfo
#include "onboard/math/fast_math.h"
#include "onboard/math/geometry/util.h"  // for Vec2dFromApolloTrajectoryPointProto
#include "onboard/math/util.h"           // for NormalizeAngle
#include "onboard/math/vec.h"  // for Vec2d, Vec2, MatrixBase::operator-, MatrixBase::operator+
#include "onboard/planner/util/lane_path_util.h"  // for GetLanesInfo
#include "onboard/utils/status_macros.h"

namespace qcraft {
namespace planner {
namespace ml {

absl::StatusOr<TrajectoryFeature> ExtractTrajectoryFeature(
    const ContextFeature& context_feature,
    const std::vector<ApolloTrajectoryPointProto>& traj,
    const double start_ts) {
  if (traj.size() < 2) {
    return absl::FailedPreconditionError(
        " A trajectory is too short! Less than two points.");
  }
  const auto ref_pos = context_feature.ref_position;
  const float rot_rad = context_feature.rot_rad;

  TrajectoryFeature feat;

  const auto abs_cur_pos = Vec2dFromApolloTrajectoryPointProto(traj.front());
  const Vec2d cur_pos = (abs_cur_pos - ref_pos).Rotate(rot_rad);
  const float time_step = traj[1].relative_time() - traj[0].relative_time();
  double theta_cos_sin[2];
  fast_math::CosAndSin<12>(traj[0].path_point().theta(), theta_cos_sin);
  const float init_v_x = traj[0].v() * theta_cos_sin[0];
  const float init_v_y = traj[0].v() * theta_cos_sin[1];
  Vec2d rot_vel = (Vec2d(init_v_x, init_v_y)).Rotate(rot_rad);
  const double rot_heading =
      NormalizeAngle(traj[0].path_point().theta() + rot_rad);
  feat.pos.push_back(cur_pos.x());
  feat.pos.push_back(cur_pos.y());
  feat.pos_diff.push_back(rot_vel.x() * time_step);
  feat.pos_diff.push_back(rot_vel.y() * time_step);
  feat.speed.push_back(traj[0].v());
  double rot_cos_sin[2];
  fast_math::CosAndSin<12>(rot_heading, rot_cos_sin);
  feat.yaw.push_back(rot_cos_sin[1]);
  feat.yaw.push_back(rot_cos_sin[0]);
  feat.ts.push_back(start_ts);

  Vec2d prev_pos = cur_pos;
  double prev_t = traj[0].relative_time();
  for (int i = 1, size = traj.size(); i < size; ++i) {
    const Vec2d cur_pos =
        (Vec2dFromApolloTrajectoryPointProto(traj[i]) - ref_pos)
            .Rotate(rot_rad);
    double cur_t = traj[i].relative_time();
    const double rot_heading =
        NormalizeAngle(traj[i].path_point().theta() + rot_rad);
    feat.pos.push_back(cur_pos.x());
    feat.pos.push_back(cur_pos.y());
    feat.pos_diff.push_back((cur_pos - prev_pos).x());
    feat.pos_diff.push_back((cur_pos - prev_pos).y());
    feat.speed.push_back(traj[i].v());
    double rot_cos_sin[2];
    fast_math::CosAndSin<12>(rot_heading, rot_cos_sin);
    feat.yaw.push_back(rot_cos_sin[1]);
    feat.yaw.push_back(rot_cos_sin[0]);
    feat.ts.push_back(start_ts + cur_t - prev_t);
    prev_pos = cur_pos;
    prev_t = cur_t;
  }
  return feat;
}

absl::StatusOr<LineSegmentsFeature> ExtractLanePathFeature(
    const ContextFeature& context_feature,
    const PlannerSemanticMapManager& psmm, const mapping::LanePath& lane_path) {
  const auto lanes_info = GetLanesInfoBreakIfNotFound(psmm, lane_path);
  const int sample_points_num = kMapLanePointsNum + 1;
  const double fraction_step = 1.0 / (sample_points_num - 1);

  const auto ref_pos = context_feature.ref_position;
  const float rot_rad = context_feature.rot_rad;

  LineSegmentsFeature ls_feat;
  for (int i = 0; i < lanes_info.size(); ++i) {
    std::vector<Vec2d> lane_points;
    lane_points.reserve(sample_points_num);
    for (int j = 0; j < sample_points_num; ++j) {
      lane_points.push_back(
          lanes_info[i]->LerpPointFromFraction(j * fraction_step));
    }
    Vec2d prev_transformed_lane_point =
        (lane_points[0] - ref_pos).Rotate(rot_rad);
    for (int k = 1; k < sample_points_num; ++k) {
      const Vec2d transformed_lane_point =
          (lane_points[k] - ref_pos).Rotate(rot_rad);
      const Vec2d lane_center =
          (transformed_lane_point + prev_transformed_lane_point) * 0.5f;
      ls_feat.node.push_back(lane_center.x());
      ls_feat.node.push_back(lane_center.y());
      prev_transformed_lane_point = transformed_lane_point;
    }
  }
  return ls_feat;
}

absl::StatusOr<MultiLanePathFeature> ExtractMultiLanePathFeature(
    const ContextFeature& context_feature,
    const PlannerSemanticMapManager& psmm,
    absl::Span<const mapping::LanePath* const> lane_paths, int max_lane_size) {
  std::vector<std::vector<float>> lanes;
  std::vector<int> lanes_mask(max_lane_size, 0);
  for (int i = 0,
           size = std::min(static_cast<int>(lane_paths.size()), max_lane_size);
       i < size; ++i) {
    ASSIGN_OR_RETURN(
        auto lane_model_feature,
        ml::ExtractLanePathFeature(context_feature, psmm, *lane_paths[i]),
        absl::FailedPreconditionError("ExtractLanePathFeature Failed."));

    lanes.push_back(std::vector<float>(lane_model_feature.node.begin(),
                                       lane_model_feature.node.end()));
    lanes_mask[i] = 1;
  }
  MultiLanePathFeature multi_lane{.lanes = std::move(lanes),
                                  .lanes_mask = std::move(lanes_mask)};
  return multi_lane;
}

absl::StatusOr<PathBoundaryFeature> ExtractPathBoundaryFeature(
    const ContextFeature& context_feature,
    const PathSlBoundary& path_sl_boundary) {
  const auto ref_pos = context_feature.ref_position;
  const float rot_rad = context_feature.rot_rad;
  PathBoundaryFeature pb_feature;
  for (double s = path_sl_boundary.s_vector().front(),
              end = path_sl_boundary.s_vector().back();
       s < end; s += kPathBoundarySampleLen) {
    const auto [rb, lb] = path_sl_boundary.QueryBoundaryXY(s);
    const auto cl = path_sl_boundary.QueryReferenceCenterXY(s);
    const auto trans_rb = (rb - ref_pos).Rotate(rot_rad);
    const auto trans_lb = (lb - ref_pos).Rotate(rot_rad);
    const auto trans_cl = (cl - ref_pos).Rotate(rot_rad);
    pb_feature.left_path_boundary.node.push_back(trans_lb.x());
    pb_feature.left_path_boundary.node.push_back(trans_lb.y());
    pb_feature.right_path_boundary.node.push_back(trans_rb.x());
    pb_feature.right_path_boundary.node.push_back(trans_rb.y());
    pb_feature.smooth_ref_center_line.node.push_back(trans_cl.x());
    pb_feature.smooth_ref_center_line.node.push_back(trans_cl.y());
  }
  return pb_feature;
}

absl::StatusOr<LineSegmentsFeature> ExtractRefPathFeature(
    const ContextFeature& context_feature,
    const PathSlBoundary& path_sl_boundary) {
  const auto ref_pos = context_feature.ref_position;
  const float rot_rad = context_feature.rot_rad;
  LineSegmentsFeature ls_feat;
  for (double s = path_sl_boundary.s_vector().front(),
              end = path_sl_boundary.s_vector().back();
       s < end; s += kPathBoundarySampleLen) {
    const auto cl = path_sl_boundary.QueryReferenceCenterXY(s);
    const auto trans_cl = (cl - ref_pos).Rotate(rot_rad);
    ls_feat.node.push_back(trans_cl.x());
    ls_feat.node.push_back(trans_cl.y());
  }
  return ls_feat;
}

}  // namespace ml
}  // namespace planner
}  // namespace qcraft
