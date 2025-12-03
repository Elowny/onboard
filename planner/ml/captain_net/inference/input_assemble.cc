#include "onboard/planner/ml/captain_net/inference/input_assemble.h"

#include <stdint.h>

#include <algorithm>
#include <vector>

#include "onboard/prediction/feature_extractor/act_net_feature.h"

namespace qcraft::planner::ml {

captain_net::CaptainNetFeature AssembleInputFeature(
    const ContextFeature& context_feature,
    const MultiLanePathFeature& condition_feature) {
  const auto& act_net_data = context_feature.act_net_feature;
  const auto& agent_feature = act_net_data.agent_feature;
  const auto& objects_feature = act_net_data.context_obj_features;
  const auto& lane_centers_feature = act_net_data.lc_features;
  const auto& lane_boundaries_feature = act_net_data.solid_lb_features;
  const auto& crosswalks_feature = act_net_data.cw_features;

  captain_net::CaptainNetFeature input_features =
      captain_net::CaptainNetFeature{
          .agent_feature =
              captain_net::AgentFeature{
                  .agent_type = static_cast<int>(agent_feature.type),
                  .agent_lw = std::vector<float>(agent_feature.lw.begin(),
                                                 agent_feature.lw.end()),
                  .agent_pos = std::vector<float>(agent_feature.pos.begin(),
                                                  agent_feature.pos.end()),
                  .agent_pos_diff =
                      std::vector<float>(agent_feature.pos_diff.begin(),
                                         agent_feature.pos_diff.end()),
                  .agent_speed = std::vector<float>(agent_feature.speed.begin(),
                                                    agent_feature.speed.end()),
                  .agent_heading = std::vector<float>(agent_feature.yaw.begin(),
                                                      agent_feature.yaw.end()),
              },
          .actors_feature =
              captain_net::ActorsFeature{
                  .obj_type = std::vector<int64_t>(),
                  .obj_lw = std::vector<float>(),
                  .obj_pos = std::vector<float>(),
                  .obj_pos_diff = std::vector<float>(),
                  .obj_speed = std::vector<float>(),
                  .obj_heading = std::vector<float>(),
                  .rel_dist = std::vector<float>(),
                  .rel_pos = std::vector<float>(),
                  .rel_yaw = std::vector<float>(),
                  .rel_speed = std::vector<float>(),
                  .valid_mask =
                      std::vector<int>(act_net_data.context_obj_masks.begin(),
                                       act_net_data.context_obj_masks.end()),
              },
          .lane_center_feature =
              captain_net::LanesFeature{
                  .seg = std::vector<float>(),
                  .unit_seg_vec = std::vector<float>(),
                  .seg_len = std::vector<float>(),
                  .unit_tangent = std::vector<float>(),
                  .dist = std::vector<float>(),
                  .unit_dist_vec = std::vector<float>(),
                  .nearest_to_end_dist = std::vector<float>(),
                  .speed_limits = std::vector<float>(),
                  .yaw_diff = std::vector<float>(),
                  .light = std::vector<float>(),
                  .type = std::vector<int64_t>(),
                  .valid_mask =
                      std::vector<int>(act_net_data.lc_masks.begin(),
                                       act_net_data.lc_masks.end()),
              },
          .lane_boundary_feature =
              captain_net::LanesFeature{
                  .seg = std::vector<float>(),
                  .seg_len = std::vector<float>(),
                  .unit_tangent = std::vector<float>(),
                  .dist = std::vector<float>(),
                  .unit_dist_vec = std::vector<float>(),
                  .nearest_to_end_dist = std::vector<float>(),
                  .speed_limits = std::vector<float>(),
                  .light = std::vector<float>(),
                  .type = std::vector<int64_t>(),
                  .valid_mask =
                      std::vector<int>(act_net_data.solid_lb_masks.begin(),
                                       act_net_data.solid_lb_masks.end()),
              },
          .crosswalk_feature =
              captain_net::LanesFeature{
                  .seg = std::vector<float>(),
                  .seg_len = std::vector<float>(),
                  .unit_tangent = std::vector<float>(),
                  .dist = std::vector<float>(),
                  .unit_dist_vec = std::vector<float>(),
                  .nearest_to_end_dist = std::vector<float>(),
                  .speed_limits = std::vector<float>(),
                  .light = std::vector<float>(),
                  .type = std::vector<int64_t>(),
                  .valid_mask =
                      std::vector<int>(act_net_data.cw_masks.begin(),
                                       act_net_data.cw_masks.end()),
              },
          .target_lane_feature =
              captain_net::TargetLanesFeature{
                  .target_lanes = condition_feature.lanes,
                  .valid_mask =
                      std::vector<int>(condition_feature.lanes_mask.begin(),
                                       condition_feature.lanes_mask.end()),
              },
      };
  auto& actors_feature = input_features.actors_feature;
  for (const auto& obj_feature : objects_feature) {
    actors_feature.obj_type.emplace_back(obj_feature.abs_feat.type);
    actors_feature.obj_lw.insert(actors_feature.obj_lw.end(),
                                 obj_feature.abs_feat.lw.begin(),
                                 obj_feature.abs_feat.lw.end());
    actors_feature.obj_pos.insert(actors_feature.obj_pos.end(),
                                  obj_feature.abs_feat.pos.begin(),
                                  obj_feature.abs_feat.pos.end());
    actors_feature.obj_pos_diff.insert(actors_feature.obj_pos_diff.end(),
                                       obj_feature.abs_feat.pos_diff.begin(),
                                       obj_feature.abs_feat.pos_diff.end());
    actors_feature.obj_speed.insert(actors_feature.obj_speed.end(),
                                    obj_feature.abs_feat.speed.begin(),
                                    obj_feature.abs_feat.speed.end());
    actors_feature.obj_heading.insert(actors_feature.obj_heading.end(),
                                      obj_feature.abs_feat.yaw.begin(),
                                      obj_feature.abs_feat.yaw.end());
    actors_feature.rel_dist.insert(actors_feature.rel_dist.end(),
                                   obj_feature.rel_feat.rel_dist.begin(),
                                   obj_feature.rel_feat.rel_dist.end());
    actors_feature.rel_pos.insert(actors_feature.rel_pos.end(),
                                  obj_feature.rel_feat.rel_pos.begin(),
                                  obj_feature.rel_feat.rel_pos.end());
    actors_feature.rel_yaw.insert(actors_feature.rel_yaw.end(),
                                  obj_feature.rel_feat.rel_yaw.begin(),
                                  obj_feature.rel_feat.rel_yaw.end());
    actors_feature.rel_speed.insert(actors_feature.rel_speed.end(),
                                    obj_feature.rel_feat.rel_speed.begin(),
                                    obj_feature.rel_feat.rel_speed.end());
  }
  auto& input_lane_center_feature = input_features.lane_center_feature;
  for (const auto& lane_center_feature : lane_centers_feature) {
    input_lane_center_feature.seg.insert(input_lane_center_feature.seg.end(),
                                         lane_center_feature.seg.begin(),
                                         lane_center_feature.seg.end());
    input_lane_center_feature.unit_seg_vec.insert(
        input_lane_center_feature.unit_seg_vec.end(),
        lane_center_feature.unit_seg_vec.begin(),
        lane_center_feature.unit_seg_vec.end());
    input_lane_center_feature.seg_len.insert(
        input_lane_center_feature.seg_len.end(),
        lane_center_feature.seg_len.begin(), lane_center_feature.seg_len.end());
    input_lane_center_feature.unit_tangent.insert(
        input_lane_center_feature.unit_tangent.end(),
        lane_center_feature.unit_tangent.begin(),
        lane_center_feature.unit_tangent.end());
    input_lane_center_feature.dist.insert(input_lane_center_feature.dist.end(),
                                          lane_center_feature.dist.begin(),
                                          lane_center_feature.dist.end());
    input_lane_center_feature.unit_dist_vec.insert(
        input_lane_center_feature.unit_dist_vec.end(),
        lane_center_feature.unit_dist_vec.begin(),
        lane_center_feature.unit_dist_vec.end());
    input_lane_center_feature.nearest_to_end_dist.insert(
        input_lane_center_feature.nearest_to_end_dist.end(),
        lane_center_feature.nearest_to_end_dist.begin(),
        lane_center_feature.nearest_to_end_dist.end());
    input_lane_center_feature.speed_limits.insert(
        input_lane_center_feature.speed_limits.end(),
        lane_center_feature.speed_limit_kph.begin(),
        lane_center_feature.speed_limit_kph.end());
    input_lane_center_feature.yaw_diff.insert(
        input_lane_center_feature.yaw_diff.end(),
        lane_center_feature.yaw_diff.begin(),
        lane_center_feature.yaw_diff.end());
    input_lane_center_feature.light.insert(
        input_lane_center_feature.light.end(),
        lane_center_feature.light.begin(), lane_center_feature.light.end());
    input_lane_center_feature.type.insert(input_lane_center_feature.type.end(),
                                          lane_center_feature.type.begin(),
                                          lane_center_feature.type.end());
  }
  auto& input_lane_boundary_feature = input_features.lane_boundary_feature;
  for (const auto& lane_boundary_feature : lane_boundaries_feature) {
    input_lane_boundary_feature.seg.insert(
        input_lane_boundary_feature.seg.end(),
        lane_boundary_feature.seg.begin(), lane_boundary_feature.seg.end());
    input_lane_boundary_feature.unit_seg_vec.insert(
        input_lane_boundary_feature.unit_seg_vec.end(),
        lane_boundary_feature.unit_seg_vec.begin(),
        lane_boundary_feature.unit_seg_vec.end());
    input_lane_boundary_feature.seg_len.insert(
        input_lane_boundary_feature.seg_len.end(),
        lane_boundary_feature.seg_len.begin(),
        lane_boundary_feature.seg_len.end());
    input_lane_boundary_feature.unit_tangent.insert(
        input_lane_boundary_feature.unit_tangent.end(),
        lane_boundary_feature.unit_tangent.begin(),
        lane_boundary_feature.unit_tangent.end());
    input_lane_boundary_feature.dist.insert(
        input_lane_boundary_feature.dist.end(),
        lane_boundary_feature.dist.begin(), lane_boundary_feature.dist.end());
    input_lane_boundary_feature.unit_dist_vec.insert(
        input_lane_boundary_feature.unit_dist_vec.end(),
        lane_boundary_feature.unit_dist_vec.begin(),
        lane_boundary_feature.unit_dist_vec.end());
    input_lane_boundary_feature.nearest_to_end_dist.insert(
        input_lane_boundary_feature.nearest_to_end_dist.end(),
        lane_boundary_feature.nearest_to_end_dist.begin(),
        lane_boundary_feature.nearest_to_end_dist.end());
    input_lane_boundary_feature.speed_limits.insert(
        input_lane_boundary_feature.speed_limits.end(),
        lane_boundary_feature.speed_limit_kph.begin(),
        lane_boundary_feature.speed_limit_kph.end());
    input_lane_boundary_feature.yaw_diff.insert(
        input_lane_boundary_feature.yaw_diff.end(),
        lane_boundary_feature.yaw_diff.begin(),
        lane_boundary_feature.yaw_diff.end());
    input_lane_boundary_feature.light.insert(
        input_lane_boundary_feature.light.end(),
        lane_boundary_feature.light.begin(), lane_boundary_feature.light.end());
    input_lane_boundary_feature.type.insert(
        input_lane_boundary_feature.type.end(),
        lane_boundary_feature.type.begin(), lane_boundary_feature.type.end());
  }

  auto& input_crosswalk_feature = input_features.crosswalk_feature;
  for (const auto& crosswalk_feature : crosswalks_feature) {
    input_crosswalk_feature.seg.insert(input_crosswalk_feature.seg.end(),
                                       crosswalk_feature.seg.begin(),
                                       crosswalk_feature.seg.end());
    input_crosswalk_feature.unit_seg_vec.insert(
        input_crosswalk_feature.unit_seg_vec.end(),
        crosswalk_feature.unit_seg_vec.begin(),
        crosswalk_feature.unit_seg_vec.end());
    input_crosswalk_feature.seg_len.insert(
        input_crosswalk_feature.seg_len.end(),
        crosswalk_feature.seg_len.begin(), crosswalk_feature.seg_len.end());
    input_crosswalk_feature.unit_tangent.insert(
        input_crosswalk_feature.unit_tangent.end(),
        crosswalk_feature.unit_tangent.begin(),
        crosswalk_feature.unit_tangent.end());
    input_crosswalk_feature.dist.insert(input_crosswalk_feature.dist.end(),
                                        crosswalk_feature.dist.begin(),
                                        crosswalk_feature.dist.end());
    input_crosswalk_feature.unit_dist_vec.insert(
        input_crosswalk_feature.unit_dist_vec.end(),
        crosswalk_feature.unit_dist_vec.begin(),
        crosswalk_feature.unit_dist_vec.end());
    input_crosswalk_feature.nearest_to_end_dist.insert(
        input_crosswalk_feature.nearest_to_end_dist.end(),
        crosswalk_feature.nearest_to_end_dist.begin(),
        crosswalk_feature.nearest_to_end_dist.end());
    input_crosswalk_feature.speed_limits.insert(
        input_crosswalk_feature.speed_limits.end(),
        crosswalk_feature.speed_limit_kph.begin(),
        crosswalk_feature.speed_limit_kph.end());
    input_crosswalk_feature.yaw_diff.insert(
        input_crosswalk_feature.yaw_diff.end(),
        crosswalk_feature.yaw_diff.begin(), crosswalk_feature.yaw_diff.end());
    input_crosswalk_feature.light.insert(input_crosswalk_feature.light.end(),
                                         crosswalk_feature.light.begin(),
                                         crosswalk_feature.light.end());
    input_crosswalk_feature.type.insert(input_crosswalk_feature.type.end(),
                                        crosswalk_feature.type.begin(),
                                        crosswalk_feature.type.end());
  }
  return input_features;
}

}  // namespace qcraft::planner::ml
