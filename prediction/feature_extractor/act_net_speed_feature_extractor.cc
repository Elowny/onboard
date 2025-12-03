#include "onboard/prediction/feature_extractor/act_net_speed_feature_extractor.h"

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

#include "absl/types/span.h"

#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/math/fast_math.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/common/discretized_path.h"
#include "onboard/planner/object/spacetime_object_state.h"
#include "onboard/prediction/feature_extractor/act_net_feature_extractor.h"
#include "onboard/prediction/predicted_trajectory.h"
#include "onboard/proto/trajectory_point.pb.h"

namespace qcraft {
namespace prediction {
namespace {
void FillPathPointFeature(const std::vector<PathPoint>& path_points,
                          const Vec2d& ref_pos, double rot_rad, double ref_s,
                          std::vector<float>* xy, std::vector<float>* s,
                          std::vector<float>* s_offset,
                          std::vector<float>* heading) {
  xy->reserve(path_points.size() * kActNetSpeedConfig.coord_num);
  s->reserve(path_points.size());
  s_offset->reserve(path_points.size());
  heading->reserve(path_points.size() * kActNetSpeedConfig.coord_num);
  for (const auto& path_point : path_points) {
    const Vec2d pt_pos(path_point.x(), path_point.y());
    const Vec2d pt_pos_convert =
        (pt_pos - ref_pos).Rotate(rot_rad);  // Convert to agent coord.
    const double yaw = path_point.theta() + rot_rad;
    // In order: x, y, s, s_offset, yaw_cos, yaw_sin.
    xy->push_back(pt_pos_convert.x());
    xy->push_back(pt_pos_convert.y());
    s->push_back(path_point.s());
    const auto offset = ref_s - path_point.s();
    s_offset->push_back(offset);
    heading->push_back(fast_math::Cos(yaw));
    heading->push_back(fast_math::Sin(yaw));
  }
}

planner::DiscretizedPath SpacetimeObjectTrajectoryToDiscretizedPath(
    const planner::SpacetimeObjectTrajectory& st_traj) {
  planner::DiscretizedPath path;
  path.reserve(st_traj.states().size());
  for (const auto& state : st_traj.states()) {
    const auto& pt = *(state.traj_point);
    PathPoint path_pt;
    path_pt.set_x(pt.pos().x());
    path_pt.set_y(pt.pos().y());
    path_pt.set_s(pt.s());
    path_pt.set_theta(pt.theta());
    path_pt.set_kappa(pt.kappa());
    path.push_back(std::move(path_pt));
  }
  return path;
}

std::vector<PathPoint> SampleNPathPointsBeforeS(
    const planner::DiscretizedPath& path, double final_s, int point_num) {
  const double max_s = std::min<double>(final_s, path.length());
  const double ds = max_s / (point_num - 1);
  std::vector<PathPoint> points;
  points.reserve(point_num);
  for (int i = 0; i < point_num; ++i) {
    const double evaluate_s = std::min(i * ds, max_s);
    auto path_point = path.Evaluate(evaluate_s);
    points.push_back(std::move(path_point));
  }
  return points;
}

PathFeature ExtractPathFeature(const planner::DiscretizedPath& path,
                               const std::vector<PathPoint>& sampled_points,
                               double s_intersect, double rot_rad,
                               const Vec2d& ref_pos) {
  PathFeature path_feature;
  QCHECK_EQ(sampled_points.size(), kActNetSpeedConfig.path_point_num);
  // Sample 20 points along the path (maybe after intersection points)
  FillPathPointFeature(sampled_points, ref_pos, rot_rad, s_intersect,
                       &path_feature.path_xy, &path_feature.path_s,
                       &path_feature.path_s_offset, &path_feature.path_heading);
  // Fill in point valid (point mask).
  path_feature.point_valid.reserve(kActNetSpeedConfig.path_point_num);
  for (const auto& s_offset : path_feature.path_s_offset) {
    if (s_offset >= 0.0) {
      path_feature.point_valid.push_back(1.0);
    } else {
      path_feature.point_valid.push_back(0.0);
    }
  }
  // Sample 20 points before point of intersection (s_intersect).
  const std::vector<PathPoint> sampled_points_pre_intersection =
      SampleNPathPointsBeforeS(
          path, s_intersect,
          kActNetSpeedConfig.path_point_num_pre_intersection);
  FillPathPointFeature(
      sampled_points_pre_intersection, ref_pos, rot_rad, s_intersect,
      &path_feature.path_xy_preint, &path_feature.path_s_preint,
      &path_feature.path_s_offset_preint, &path_feature.path_heading_preint);
  return path_feature;
}

ActNetSpeedFeature AssembleActNetSpeedFeature(
    const std::string& agent_id, double rot_rad, const Vec2d& ref_position,
    const ActNetObjectAbsoluteFeature& agent_feature,
    const std::vector<ActNetObjectFeature>& context_obj_features,
    const std::vector<float>& context_obj_masks,
    const planner::DiscretizedPath& av_discretized_path,
    const planner::DiscretizedPath& agent_discretized_path,
    const std::vector<PathPoint>& av_sampled_pts, int traj_index, double av_s,
    double object_s) {
  ActNetSpeedFeature speed_feature;
  speed_feature.traj_index = traj_index;
  speed_feature.agent_id = agent_id;
  speed_feature.rot_rad = rot_rad;
  speed_feature.ref_position = ref_position;
  // Copy common agent and av features.
  speed_feature.agent_feature = agent_feature;
  speed_feature.context_obj_features = context_obj_features;
  speed_feature.context_obj_masks = context_obj_masks;
  // Extract av path features.
  auto av_path = ExtractPathFeature(av_discretized_path, av_sampled_pts, av_s,
                                    rot_rad, ref_position);
  speed_feature.av_path = std::move(av_path);
  // Extract agent path features.
  const auto agent_sampled_pts = SampleNPathPointsBeforeS(
      agent_discretized_path, agent_discretized_path.length(),
      kActNetSpeedConfig.path_point_num);
  auto agent_path =
      ExtractPathFeature(agent_discretized_path, agent_sampled_pts, object_s,
                         rot_rad, ref_position);
  speed_feature.agent_path = std::move(agent_path);
  return speed_feature;
}

ActNetSpeedFeature AssembleActNetSpeedFeatureFromActNetFeature(
    const ActNetFeature& act_net_feature,
    const planner::DiscretizedPath& av_path,
    const planner::DiscretizedPath& agent_path,
    const std::vector<PathPoint>& av_sampled_pts, int traj_index, double av_s,
    double object_s) {
  return AssembleActNetSpeedFeature(
      act_net_feature.agent_id, act_net_feature.rot_rad,
      act_net_feature.ref_position, act_net_feature.agent_feature,
      act_net_feature.context_obj_features, act_net_feature.context_obj_masks,
      av_path, agent_path, av_sampled_pts, traj_index, av_s, object_s);
}
}  // namespace

// Planner module.
ActNetSpeedFeature ExtractActNetSpeedFeaturesForPlannerTarget(
    const ActNetFeature& act_net_feature,
    const planner::DiscretizedPath& av_path,
    const ActNetSpeedPlannerTarget& target) {
  // Sample av discretized path.
  const auto av_sampled_pts = SampleNPathPointsBeforeS(
      av_path, av_path.length(), kActNetSpeedConfig.path_point_num);
  const auto& st_traj = *target.st_traj;
  const planner::DiscretizedPath agent_discretized_path =
      SpacetimeObjectTrajectoryToDiscretizedPath(st_traj);
  return AssembleActNetSpeedFeatureFromActNetFeature(
      act_net_feature, av_path, agent_discretized_path, av_sampled_pts,
      st_traj.traj_index(), target.av_s, target.object_s);
}

ActNetFeature ExtractActNetFeatureWithAVOnly(
    const ObjectIDType& agent_id, const ObjectMotionHistory& agent_history,
    const ObjectMotionHistory& av_history) {
  const double heading = agent_history.states.back().heading;
  const auto ref_position = agent_history.states.back().pos;
  const double rot_rad = -heading;

  const double normal_rot_rad = NormalizeAngle(rot_rad);
  const auto agent_coord_transform =
      AgentCoordTransform{.orig = ref_position,
                          .rot_rad = rot_rad,
                          .rot_sin = fast_math::SinNormalized(normal_rot_rad),
                          .rot_cos = fast_math::CosNormalized(normal_rot_rad)};

  // Agent feature.
  auto agent_feature = GetActNetObjectAbsoluteFeature(
      agent_history, agent_coord_transform, kActNetSpeedConfig.coord_num,
      kFeatureV2HistoryStepLen, kFeatureV2HistoryStepNum);

  // AV feature.
  auto abs_feat = GetActNetObjectAbsoluteFeature(
      av_history, agent_coord_transform, kActNetSpeedConfig.coord_num,
      kFeatureV2HistoryStepLen, kFeatureV2HistoryStepNum);
  auto rel_feat = GetActNetObjectRelFeature(agent_feature, abs_feat,
                                            kActNetSpeedConfig.coord_num,
                                            kFeatureV2HistoryStepNum);
  std::vector<ActNetObjectFeature> context_obj_features;
  context_obj_features.reserve(1);
  context_obj_features.emplace_back(ActNetObjectFeature{
      .id = av_history.id,
      .abs_feat = std::move(abs_feat),
      .rel_feat = std::move(rel_feat),
  });
  std::vector<float> context_obj_masks(kActNetSpeedConfig.max_other_objects_num,
                                       0.0);
  for (int i = 0; i < context_obj_features.size(); ++i) {
    context_obj_masks[i] = 1.0;
  }
  return ActNetFeature{
      .agent_id = agent_id,
      .ref_position = ref_position,
      .rot_rad = static_cast<float>(rot_rad),
      .agent_feature = std::move(agent_feature),
      .context_obj_features = std::move(context_obj_features),
      .context_obj_masks = std::move(context_obj_masks),
  };
}
}  // namespace prediction
}  // namespace qcraft
