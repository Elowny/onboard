#include "onboard/planner/ml/context_feature_extractor/context_feature_proto_converter.h"

#include <algorithm>  // for copy
#include <utility>    // for move
#include <vector>     // for vector

#include "onboard/math/geometry/util.h"  // for Vec2dToProto
#include "onboard/math/vec.h"            // for Vec2d
#include "onboard/prediction/feature_extractor/act_net_feature.h"  // for ActNetObjectAbsoluteFeature, ActNetObjectFeature
#include "onboard/prediction/proto/act_net.pb.h"  // for ActNetDumpedFeatureProto, ActNetDumpedFeatureProt...

namespace qcraft {
namespace planner {
namespace ml {

namespace {

ActNetDumpedFeatureProto ToActNetDumpedFeatureProto(
    const prediction::ActNetFeature& act_feature, double current_ts,
    const Vec2d& ref_pos, double rot_rad) {
  ActNetDumpedFeatureProto feature_proto;

  feature_proto.set_agent_id(act_feature.agent_id);
  feature_proto.set_timestamp(current_ts);
  Vec2dToProto(ref_pos, feature_proto.mutable_ref_pos());
  feature_proto.set_rot_angle(rot_rad);

  // Fill in agent feature.
  auto* mutable_agent_feature = feature_proto.mutable_agent_feature();
  const auto& agent_feature = act_feature.agent_feature;
  *mutable_agent_feature->mutable_pos() = {agent_feature.pos.begin(),
                                           agent_feature.pos.end()};
  *mutable_agent_feature->mutable_pos_diff() = {agent_feature.pos_diff.begin(),
                                                agent_feature.pos_diff.end()};
  *mutable_agent_feature->mutable_speed() = {agent_feature.speed.begin(),
                                             agent_feature.speed.end()};
  *mutable_agent_feature->mutable_yaw() = {agent_feature.yaw.begin(),
                                           agent_feature.yaw.end()};
  *mutable_agent_feature->mutable_shape() = {agent_feature.shape.begin(),
                                             agent_feature.shape.end()};
  *mutable_agent_feature->mutable_ts() = {agent_feature.ts.begin(),
                                          agent_feature.ts.end()};
  *mutable_agent_feature->mutable_mask() = {agent_feature.mask.begin(),
                                            agent_feature.mask.end()};
  mutable_agent_feature->set_type(agent_feature.type);
  *mutable_agent_feature->mutable_lw() = {agent_feature.lw.begin(),
                                          agent_feature.lw.end()};
  *mutable_agent_feature->mutable_stop_time_info() = {
      act_feature.stop_time_info.begin(), act_feature.stop_time_info.end()};
  // Fill in context object feature.
  const auto& context_obj_features = act_feature.context_obj_features;
  auto* mutable_objects_feature = feature_proto.mutable_objects_feature();
  for (const auto& obj_feature : context_obj_features) {
    mutable_objects_feature->mutable_pos()->Add(
        obj_feature.abs_feat.pos.begin(), obj_feature.abs_feat.pos.end());
    mutable_objects_feature->mutable_pos_diff()->Add(
        obj_feature.abs_feat.pos_diff.begin(),
        obj_feature.abs_feat.pos_diff.end());
    mutable_objects_feature->mutable_speed()->Add(
        obj_feature.abs_feat.speed.begin(), obj_feature.abs_feat.speed.end());
    mutable_objects_feature->mutable_yaw()->Add(
        obj_feature.abs_feat.yaw.begin(), obj_feature.abs_feat.yaw.end());
    mutable_objects_feature->mutable_shape()->Add(
        obj_feature.abs_feat.shape.begin(), obj_feature.abs_feat.shape.end());
    mutable_objects_feature->mutable_ts()->Add(obj_feature.abs_feat.ts.begin(),
                                               obj_feature.abs_feat.ts.end());
    mutable_objects_feature->mutable_mask()->Add(
        obj_feature.abs_feat.mask.begin(), obj_feature.abs_feat.mask.end());
    mutable_objects_feature->add_type(obj_feature.abs_feat.type);
    mutable_objects_feature->mutable_lw()->Add(obj_feature.abs_feat.lw.begin(),
                                               obj_feature.abs_feat.lw.end());

    mutable_objects_feature->mutable_rel_pos()->Add(
        obj_feature.rel_feat.rel_pos.begin(),
        obj_feature.rel_feat.rel_pos.end());
    mutable_objects_feature->mutable_rel_dist()->Add(
        obj_feature.rel_feat.rel_dist.begin(),
        obj_feature.rel_feat.rel_dist.end());
    mutable_objects_feature->mutable_rel_yaw()->Add(
        obj_feature.rel_feat.rel_yaw.begin(),
        obj_feature.rel_feat.rel_yaw.end());
    mutable_objects_feature->mutable_yaw_diff()->Add(
        obj_feature.rel_feat.yaw_diff.begin(),
        obj_feature.rel_feat.yaw_diff.end());
    mutable_objects_feature->mutable_rel_speed()->Add(
        obj_feature.rel_feat.rel_speed.begin(),
        obj_feature.rel_feat.rel_speed.end());
    mutable_objects_feature->mutable_rel_shape()->Add(
        obj_feature.rel_feat.rel_shape.begin(),
        obj_feature.rel_feat.rel_shape.end());
    mutable_objects_feature->mutable_rel_mask()->Add(
        obj_feature.rel_feat.rel_mask.begin(),
        obj_feature.rel_feat.rel_mask.end());
  }
  *feature_proto.mutable_objects_mask() = {
      act_feature.context_obj_masks.begin(),
      act_feature.context_obj_masks.end()};

  // Fill in map feature.
  std::vector<const std::vector<prediction::ActNetPolylineFeature>*>
      poly_features = {&act_feature.lc_features, &act_feature.solid_lb_features,
                       &act_feature.cw_features};
  auto* mutable_lc_feature = feature_proto.mutable_lane_centers_feature();
  auto* mutable_lb_feature = feature_proto.mutable_lane_boundaries_feature();
  auto* mutable_cw_feature = feature_proto.mutable_crosswalks_feature();
  std::vector<ActNetDumpedFeatureProto::MapDumpedFeature*> feature_protos = {
      mutable_lc_feature, mutable_lb_feature, mutable_cw_feature};
  for (int i = 0; i < poly_features.size(); ++i) {
    const auto& feats = *poly_features[i];
    auto& mutable_feat = *feature_protos[i];
    for (const auto& feat : feats) {
      mutable_feat.mutable_seg()->Add(feat.seg.begin(), feat.seg.end());
      mutable_feat.mutable_unit_seg_vec()->Add(feat.unit_seg_vec.begin(),
                                               feat.unit_seg_vec.end());
      mutable_feat.mutable_seg_len()->Add(feat.seg_len.begin(),
                                          feat.seg_len.end());
      mutable_feat.mutable_unit_tangent()->Add(feat.unit_tangent.begin(),
                                               feat.unit_tangent.end());
      mutable_feat.mutable_dist()->Add(feat.dist.begin(), feat.dist.end());
      mutable_feat.mutable_unit_dist_vec()->Add(feat.unit_dist_vec.begin(),
                                                feat.unit_dist_vec.end());
      mutable_feat.mutable_nearest_to_end_dist()->Add(
          feat.nearest_to_end_dist.begin(), feat.nearest_to_end_dist.end());
      mutable_feat.mutable_yaw_diff()->Add(feat.yaw_diff.begin(),
                                           feat.yaw_diff.end());
      mutable_feat.mutable_type()->Add(feat.type.begin(), feat.type.end());
      mutable_feat.mutable_light()->Add(feat.light.begin(), feat.light.end());
      mutable_feat.mutable_mask()->Add(feat.mask.begin(), feat.mask.end());
      mutable_feat.mutable_speed_limits()->Add(feat.speed_limit_kph.begin(),
                                               feat.speed_limit_kph.end());
    }
  }
  *feature_proto.mutable_lcs_mask() = {act_feature.lc_masks.begin(),
                                       act_feature.lc_masks.end()};
  *feature_proto.mutable_lbs_mask() = {act_feature.solid_lb_masks.begin(),
                                       act_feature.solid_lb_masks.end()};
  *feature_proto.mutable_cws_mask() = {act_feature.cw_masks.begin(),
                                       act_feature.cw_masks.end()};
  return feature_proto;
}

InferredFeatureProto ToInferredFeatureProto(const InferredFeature& input) {
  InferredFeatureProto pb;
  for (const auto& inferred_object : input.inferred_object_feature) {
    ObjectInferredFeatureProto obj_pb;
    obj_pb.set_object_id(inferred_object.object_id);
    obj_pb.set_is_static(inferred_object.is_static);
    obj_pb.set_has_longterm_observation(
        inferred_object.has_longterm_observation);
    obj_pb.set_average_speed(inferred_object.average_speed);
    obj_pb.set_observation_duration(inferred_object.observation_duration);
    *pb.add_inferred_object_feature() = std::move(obj_pb);
  }
  return pb;
}
}  // namespace

ContextFeatureProto ContextFeatureToProto(const ContextFeature& feat) {
  ContextFeatureProto pb;
  pb.set_current_ts(feat.current_ts);
  Vec2dToProto(feat.ref_position, pb.mutable_ref_position());
  pb.set_rot_rad(feat.rot_rad);
  *pb.mutable_act_net_context_feature() = ToActNetDumpedFeatureProto(
      feat.act_net_feature, feat.current_ts, feat.ref_position, feat.rot_rad);
  *pb.mutable_inferred_feature() =
      ToInferredFeatureProto(feat.inferred_feature);
  return pb;
}

}  // namespace ml
}  // namespace planner
}  // namespace qcraft
