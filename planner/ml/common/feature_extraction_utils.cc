#include "onboard/planner/ml/common/feature_extraction_utils.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type

#include <algorithm>

#include "absl/types/span.h"

#include "onboard/maps/maps_common.h"
#include "onboard/math/fast_math.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/geometry/util.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/util/lane_path_util.h"
#include "onboard/prediction/proto/act_net.pb.h"

namespace qcraft::planner {
absl::Status ExtractTrajectoryFeature(
    const PredictionDebugProto& prediction_debug,
    const std::vector<ApolloTrajectoryPointProto>& traj,
    ProphnetDumpedFeatureProto::ObjectsDumpedFeature* traj_model_feature) {
  if (traj.size() < 2) {
    return absl::FailedPreconditionError(
        " A trajectory is too short! Less than two points.");
  }

  if (!prediction_debug.has_features() ||
      prediction_debug.features().act_net_data_size() != 1) {
    return absl::FailedPreconditionError(
        "No valid est act_net_data output for "
        "ExtractTrajectoryFeature");
  }

  const auto& av_actnet_feature = prediction_debug.features().act_net_data(0);
  const auto ref_pos =
      Vec2d(av_actnet_feature.ref_pos().x(), av_actnet_feature.ref_pos().y());
  const float rot_rad = av_actnet_feature.rot_angle();

  // Transform pos to ego-based coordinate.
  const auto abs_cur_pos = Vec2dFromApolloTrajectoryPointProto(traj.front());
  const Vec2d cur_pos = (abs_cur_pos - ref_pos).Rotate(rot_rad);
  traj_model_feature->add_cur_poses(cur_pos.x());
  traj_model_feature->add_cur_poses(cur_pos.y());

  const float time_step = traj[1].relative_time() - traj[0].relative_time();
  double theta_cos_sin[2];
  fast_math::CosAndSin<12>(traj[0].path_point().theta(), theta_cos_sin);
  const float init_v_x = traj[0].v() * theta_cos_sin[0];
  const float init_v_y = traj[0].v() * theta_cos_sin[1];
  Vec2d rot_vel = (Vec2d(init_v_x, init_v_y)).Rotate(rot_rad);
  traj_model_feature->add_trajs(rot_vel.x() * time_step);
  traj_model_feature->add_trajs(rot_vel.y() * time_step);
  traj_model_feature->add_speeds(traj[0].v());
  const double rot_heading =
      NormalizeAngle(traj[0].path_point().theta() + rot_rad);
  double rot_cos_sin[2];
  fast_math::CosAndSin<12>(rot_heading, rot_cos_sin);
  traj_model_feature->add_headings(rot_cos_sin[1]);
  traj_model_feature->add_headings(rot_cos_sin[0]);

  Vec2d prev_pos = cur_pos;
  for (int i = 1, size = traj.size(); i < size; ++i) {
    const Vec2d cur_pos =
        (Vec2dFromApolloTrajectoryPointProto(traj[i]) - ref_pos)
            .Rotate(rot_rad);
    traj_model_feature->add_trajs((cur_pos - prev_pos).x());
    traj_model_feature->add_trajs((cur_pos - prev_pos).y());
    prev_pos = cur_pos;
    traj_model_feature->add_speeds(traj[i].v());
    const double rot_heading =
        NormalizeAngle(traj[i].path_point().theta() + rot_rad);
    double rot_cos_sin[2];
    fast_math::CosAndSin<12>(rot_heading, rot_cos_sin);
    traj_model_feature->add_headings(rot_cos_sin[1]);
    traj_model_feature->add_headings(rot_cos_sin[0]);
  }

  return absl::OkStatus();
}

absl::Status ExtractLanePathFeature(
    const PredictionDebugProto& prediction_debug,
    const PlannerSemanticMapManager& psmm, const mapping::LanePath& lane_path,
    ProphnetDumpedFeatureProto::LanesDumpedFeature* lane_feature) {
  const auto lanes_info = GetLanesInfoBreakIfNotFound(psmm, lane_path);
  const int sample_points_num = 21;
  const double fraction_step = 1.0 / (sample_points_num - 1);

  if (!prediction_debug.has_features() ||
      prediction_debug.features().act_net_data_size() != 1) {
    return absl::FailedPreconditionError(
        "No valid est act_net_data output for "
        "ExtractLanePathFeature");
  }

  const auto& av_actnet_feature = prediction_debug.features().act_net_data(0);
  const auto ref_pos =
      Vec2d(av_actnet_feature.ref_pos().x(), av_actnet_feature.ref_pos().y());
  const float rot_rad = av_actnet_feature.rot_angle();

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
      lane_feature->add_lane_centers(lane_center.x());
      lane_feature->add_lane_centers(lane_center.y());
      prev_transformed_lane_point = transformed_lane_point;
    }
  }

  return absl::OkStatus();
}

absl::Status ExtractPathBoundaryFeature(
    const PredictionDebugProto& prediction_debug,
    const PathSlBoundary& path_sl_boundary,
    ProphnetDumpedFeatureProto::LanesDumpedFeature* left_path_boundary,
    ProphnetDumpedFeatureProto::LanesDumpedFeature* right_path_boundary,
    ProphnetDumpedFeatureProto::LanesDumpedFeature* smooth_ref_center_line) {
  if (!prediction_debug.has_features() ||
      prediction_debug.features().act_net_data_size() != 1) {
    return absl::FailedPreconditionError(
        "No valid est act_net_data output for "
        "ExtractPathBoundaryFeature");
  }

  const auto& av_actnet_feature = prediction_debug.features().act_net_data(0);
  const auto ref_pos =
      Vec2d(av_actnet_feature.ref_pos().x(), av_actnet_feature.ref_pos().y());
  const float rot_rad = av_actnet_feature.rot_angle();

  for (double s = path_sl_boundary.s_vector().front(),
              end = path_sl_boundary.s_vector().back();
       s < end; s += kPathBoundarySampleLen) {
    const auto [rb, lb] = path_sl_boundary.QueryBoundaryXY(s);
    const auto cl = path_sl_boundary.QueryReferenceCenterXY(s);
    const auto trans_rb = (rb - ref_pos).Rotate(rot_rad);
    const auto trans_lb = (lb - ref_pos).Rotate(rot_rad);
    const auto trans_cl = (cl - ref_pos).Rotate(rot_rad);
    left_path_boundary->add_lane_centers(trans_lb.x());
    left_path_boundary->add_lane_centers(trans_lb.y());
    right_path_boundary->add_lane_centers(trans_rb.x());
    right_path_boundary->add_lane_centers(trans_rb.y());
    smooth_ref_center_line->add_lane_centers(trans_cl.x());
    smooth_ref_center_line->add_lane_centers(trans_cl.y());
  }

  return absl::OkStatus();
}

absl::Status ExtractRefPathFeature(
    const PredictionDebugProto& prediction_debug,
    const PathSlBoundary& path_sl_boundary,
    ProphnetDumpedFeatureProto::LanesDumpedFeature* smooth_ref_center_line) {
  if (!prediction_debug.has_features() ||
      prediction_debug.features().act_net_data_size() != 1) {
    return absl::FailedPreconditionError(
        "No valid est act_net_data output for "
        "ExtractRefPathFeature");
  }

  const auto& av_actnet_feature = prediction_debug.features().act_net_data(0);
  const auto ref_pos =
      Vec2d(av_actnet_feature.ref_pos().x(), av_actnet_feature.ref_pos().y());
  const float rot_rad = av_actnet_feature.rot_angle();

  for (double s = path_sl_boundary.s_vector().front(),
              end = path_sl_boundary.s_vector().back();
       s < end; s += kPathBoundarySampleLen) {
    const auto cl = path_sl_boundary.QueryReferenceCenterXY(s);
    const auto trans_cl = (cl - ref_pos).Rotate(rot_rad);
    smooth_ref_center_line->add_lane_centers(trans_cl.x());
    smooth_ref_center_line->add_lane_centers(trans_cl.y());
  }

  return absl::OkStatus();
}

}  // namespace qcraft::planner
