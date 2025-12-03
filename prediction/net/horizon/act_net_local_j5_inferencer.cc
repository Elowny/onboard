#include "onboard/prediction/net/horizon/act_net_local_j5_inferencer.h"

#include <algorithm>
#include <array>
#include <initializer_list>
#include <iterator>
#include <limits>
#include <optional>
#include <unordered_map>
#include <utility>

#include "absl/container/flat_hash_set.h"
#include "absl/hash/hash.h"

#include "onboard/async/parallel_for.h"
#include "onboard/container/strong_int.h"
#include "onboard/global/buffered_logger.h"
#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/maps/maps_common.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/fast_math.h"
#include "onboard/math/geometry/aabox2d.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/geometry/segment2d.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/math/util.h"
#include "onboard/prediction/feature_extractor/act_net_feature.h"
#include "onboard/prediction/feature_extractor/qc_net_feature.h"
#include "onboard/prediction/net/horizon/act_net_local_j5.h"
#include "onboard/prediction/net/horizon/horizon_tensor_wrapper.h"
#include "onboard/prediction/prediction_util.h"
#include "onboard/proto/perception/fusion/object.pb.h"
#include "onboard/utils/map_util.h"

namespace qcraft {
namespace prediction {
namespace actnetlocalj5 {

namespace {

inline AgentCoordTransform BuildCoordTransform(const Vec2d& pos,
                                               double heading) {
  AgentCoordTransform ct;
  const auto rot_rad = -heading;
  const auto normalized_rot_rad = NormalizeAngle(rot_rad);
  ct.orig = pos;
  ct.rot_rad = rot_rad;
  ct.rot_cos = fast_math::CosNormalized(normalized_rot_rad);
  ct.rot_sin = fast_math::SinNormalized(normalized_rot_rad);
  return ct;
}

inline std::vector<double> GetScanBoxInfo(ObjectType type, double speed) {
  if (type == OT_PEDESTRIAN) {
    return {kPedestrianScanBoxFront, kPedestrianScanBoxBack,
            kPedestrianScanBoxFrontHalfWidth};
  } else {
    return {kScanBoxFrontPlf(speed), kScanBoxBackPlf(speed),
            kScanBoxFrontHalfWidthPlf(speed)};
  }
}

inline Box2d GetRegionBoxForObject(const Vec2d& pos, double heading,
                                   double speed, ObjectType type) {
  const auto scan_box = GetScanBoxInfo(type, speed);
  return GetRegionBox(pos, heading, scan_box[0], scan_box[1], scan_box[2]);
}

ActNetQcObjectFeature GetActNetQcObjectFeature(
    const ObjectMotionHistory& obj_history,
    const AgentCoordTransform& agent_coord_transform, int coord_num,
    double time_step, int max_steps) {
  const auto& orig = agent_coord_transform.orig;
  const double rot_rad = agent_coord_transform.rot_rad;
  const double rot_cos = agent_coord_transform.rot_cos;
  const double rot_sin = agent_coord_transform.rot_sin;
  ActNetQcObjectFeature feat;
  feat.type = static_cast<float>(ObjectTypeToActNetType(obj_history.type));
  const auto& last_state = obj_history.states.back();
  // pos, pos_diff, lw, speed might need scaling.
  const auto state_num = obj_history.states.size();
  const int start_idx = max_steps - state_num;
  std::vector<float> pos_vec, speed_vec, heading_vec;
  pos_vec.reserve(max_steps * coord_num);
  speed_vec.reserve(max_steps * coord_num);
  heading_vec.reserve(max_steps * coord_num);
  const double len = last_state.bbox.length();
  const double width = last_state.bbox.width();
  feat.lw = {static_cast<float>(len), static_cast<float>(width)};
  for (int i = 0; i < max_steps; ++i) {
    if (i < start_idx) {
      const auto empty_vec2d = {0.0, 0.0};
      std::copy(empty_vec2d.begin(), empty_vec2d.end(),
                std::back_inserter(pos_vec));
      std::copy(empty_vec2d.begin(), empty_vec2d.end(),
                std::back_inserter(speed_vec));
      std::copy(empty_vec2d.begin(), empty_vec2d.end(),
                std::back_inserter(heading_vec));
      continue;
    }
    const auto& state = obj_history.states[i - start_idx];
    const auto pos = (state.pos - orig).Rotate(rot_cos, rot_sin);
    const auto vel = state.vel.Rotate(rot_cos, rot_sin);
    const auto heading = state.heading + rot_rad;
    pos_vec.push_back(pos.x());
    pos_vec.push_back(pos.y());
    speed_vec.push_back(vel.x());
    speed_vec.push_back(vel.y());
    heading_vec.push_back(fast_math::Cos(heading));
    heading_vec.push_back(fast_math::Sin(heading));
  }
  std::vector<float> pos_diff_vec;
  pos_diff_vec.reserve(max_steps * coord_num);
  for (int i = 0; i < max_steps; ++i) {
    if (i < start_idx) {
      pos_diff_vec.push_back(0.0);
      pos_diff_vec.push_back(0.0);
      continue;
    }
    if (i == start_idx) {
      pos_diff_vec.push_back(speed_vec[i * coord_num] * time_step);
      pos_diff_vec.push_back(speed_vec[i * coord_num + 1] * time_step);
    } else {
      pos_diff_vec.push_back(pos_vec[i * coord_num] -
                             pos_vec[(i - 1) * coord_num]);
      pos_diff_vec.push_back(pos_vec[i * coord_num + 1] -
                             pos_vec[(i - 1) * coord_num + 1]);
    }
  }
  feat.pos = std::move(pos_vec);
  feat.pos_diff = std::move(pos_diff_vec);
  feat.speed = std::move(speed_vec);
  feat.heading = std::move(heading_vec);
  return feat;
}

// TODO(changqing): change to extract ActNetQcobjectFeature.
void BuildActNetObjectAbsoluteFeatureMap(
    absl::Span<const ObjectHistory* const> objects_history,
    const ObjectHistorySampler& obj_sampler,
    std::unordered_map<ObjectIDType, ActNetQcObjectFeature>* id_feature_map_ptr,
    Box2d* search_box, std::vector<AgentRef>& agent_origins) {  // NOLINT
  QCHECK_NOTNULL(id_feature_map_ptr);
  QCHECK_NOTNULL(search_box);
  FUNC_QTRACE();
  std::optional<AABox2d> merged_aabox_opt = std::nullopt;
  auto& id_feature_map = *id_feature_map_ptr;
  for (int agent_idx = 0; agent_idx < objects_history.size(); ++agent_idx) {
    const auto& agent_hist = *objects_history[agent_idx];
    const auto& agent_id = agent_hist.id();
    const auto& agent_motion_hist =
        obj_sampler.GetResampledMotionHistoryById(agent_id);
    // Fill agent origin info.
    auto& agent_origin_ref = agent_origins[agent_idx];
    const auto& latest_state = agent_motion_hist.states.back();
    agent_origin_ref.ref_position = latest_state.pos;
    agent_origin_ref.rot_rad = -latest_state.heading;
    agent_origin_ref.agent_id = agent_id;
    const auto agent_coord_trans =
        BuildCoordTransform(latest_state.pos, latest_state.heading);
    const auto region_box = GetRegionBoxForObject(
        latest_state.pos, latest_state.heading, latest_state.vel.Length(),
        agent_motion_hist.type);
    if (!merged_aabox_opt.has_value()) {
      merged_aabox_opt = region_box.aabox();
    } else {
      merged_aabox_opt->MergeFrom(region_box.aabox());
    }
    if (id_feature_map.find(agent_id) == id_feature_map.end()) {
      id_feature_map[agent_id] = GetActNetQcObjectFeature(
          agent_motion_hist, agent_coord_trans, kActNetLocalJ5Config.coord_num,
          kFeatureV2HistoryStepLen, kFeatureV2HistoryStepNum);
    }
    const auto objs = obj_sampler.GetResampledMotionHistoryWithAVInBox2d(
        agent_id, region_box, kActNetLocalJ5Config.max_other_objs_num);
    for (int obj_idx = 0; obj_idx < objs.size(); ++obj_idx) {
      const auto& obj_motion_hist = *objs[obj_idx];
      const auto& latest_state = obj_motion_hist.states.back();
      const auto obj_coord_trans =
          BuildCoordTransform(latest_state.pos, latest_state.heading);
      if (id_feature_map.find(obj_motion_hist.id) == id_feature_map.end()) {
        id_feature_map[obj_motion_hist.id] = GetActNetQcObjectFeature(
            obj_motion_hist, obj_coord_trans, kActNetLocalJ5Config.coord_num,
            kFeatureV2HistoryStepLen, kFeatureV2HistoryStepNum);
      }
    }
  }
  QCHECK(merged_aabox_opt.has_value());
  *search_box = Box2d(*merged_aabox_opt);  // For map feature extraction.
}

[[maybe_unused]] void ComputeAgentDependentPolylineFeature(
    const QcNetLocalCoord& agent_coord, const AgentCoordTransform& local_coord,
    absl::Span<const float> segs, int seg_idx, int coord_num,
    absl::Span<float> features) {
  // Make sure everything is in AV centered coordinate!
  const int query_idx = seg_idx * coord_num * 2;
  const Vec2d start(segs[query_idx], segs[query_idx + 1]);
  const Vec2d end(segs[query_idx + 2], segs[query_idx + 3]);
  const Segment2d seg_local(start, end);
  const Vec2d agent_pos_local =
      (agent_coord.pos - local_coord.orig)
          .Rotate(local_coord.rot_cos, local_coord.rot_sin);
  // dist.
  Vec2d nearest_pt;
  features[0] = seg_local.DistanceTo(agent_pos_local, &nearest_pt);
  // unit_dist_vec.
  const Vec2d nearest_pt_dir =
      features[0] < 1e-3 ? seg_local.unit_direction() : nearest_pt.Unit();
  features[1] = nearest_pt_dir.x();
  features[2] = nearest_pt_dir.y();
}

// TODO(changqing): extract agent-independent feature needed by the model
// only!
ActNetQcPolylineLocalFeature ExtractAgentIndependentMapFeatureFromPolyline(
    const SampledPolylineWithConstSegRef& polyline, int max_lane_seg_num,
    int coord_num) {
  FUNC_QTRACE();
  // Should extract seg(pos), seg_len, unit_tangent, speed_limits, type and
  // lights.
  const int k_light_num = 4;
  // TODO(changqing): build local coordinate entry point here! Start point of
  // first segment and first segment's unit tangent heading.
  const auto& seg_ptrs = polyline.seg_ptrs;

  ActNetQcPolylineLocalFeature feat;
  feat.coord_trans = BuildCoordTransform(seg_ptrs[0]->seg.start(),
                                         seg_ptrs[0]->unit_tangent.FastAngle());
  const auto& orig = feat.coord_trans.orig;
  const double rot_cos = feat.coord_trans.rot_cos;
  const double rot_sin = feat.coord_trans.rot_sin;

  const int segs_size = seg_ptrs.size();
  QCHECK_GT(segs_size, 0);
  feat.seg.resize(max_lane_seg_num * coord_num * 2, 0.0);
  feat.seg_len.resize(max_lane_seg_num, 0.0);
  feat.unit_tangent.resize(max_lane_seg_num * coord_num, 0.0);
  feat.unit_seg_vec.resize(max_lane_seg_num * coord_num, 0.0);
  feat.type.resize(max_lane_seg_num, 0.0);
  feat.light.resize(max_lane_seg_num * k_light_num, 0.0);
  feat.speed_limit_kph.resize(max_lane_seg_num, 0.0);
  for (int i = 0; i < segs_size; ++i) {
    const auto& cur = seg_ptrs[i]->seg.start();
    const auto& next = seg_ptrs[i]->seg.end();
    const auto cur_pos = (cur - orig).Rotate(rot_cos, rot_sin);
    const auto next_pos = (next - orig).Rotate(rot_cos, rot_sin);

    feat.seg[i * coord_num * 2] = cur_pos.x();
    feat.seg[i * coord_num * 2 + 1] = cur_pos.y();
    feat.seg[i * coord_num * 2 + 2] = next_pos.x();
    feat.seg[i * coord_num * 2 + 3] = next_pos.y();

    const auto seg_vec = next_pos - cur_pos;
    const auto seg_len = seg_vec.norm();
    feat.seg_len[i] = seg_len;

    const auto unit_tangent =
        seg_ptrs[i]->unit_tangent.Rotate(rot_cos, rot_sin);
    feat.unit_tangent[i * coord_num] = unit_tangent.x();
    feat.unit_tangent[i * coord_num + 1] = unit_tangent.y();

    const auto unit_seg_vec = seg_vec.normalized();
    feat.unit_seg_vec[i * coord_num] = unit_seg_vec.x();
    feat.unit_seg_vec[i * coord_num + 1] = unit_seg_vec.y();

    feat.type[i] = static_cast<float>(seg_ptrs[i]->type);
    for (int k = 0; k < k_light_num; ++k) {
      feat.light[i * k_light_num + k] = seg_ptrs[i]->lights[k];
    }
    feat.speed_limit_kph[i] = seg_ptrs[i]->speed_limit_kph;
  }
  return feat;
}

std::vector<ActNetQcPolylineLocalFeature>
BreakSampledPolylineAndExtractActNetPolylineFeature(
    const SampledPolyline& polyline, const Box2d& region_box,
    int max_lane_seg_num, int coord_num) {
  const auto& segs = polyline.segs;
  const int segs_size = segs.size();
  const int num_breaks = (segs_size % max_lane_seg_num == 0)
                             ? segs_size / max_lane_seg_num
                             : segs_size / max_lane_seg_num + 1;
  std::vector<ActNetQcPolylineLocalFeature> poly_feats;
  poly_feats.reserve(num_breaks);
  for (int i = 0; i < num_breaks; ++i) {
    SampledPolylineWithConstSegRef breaked_poly_segments;
    breaked_poly_segments.id = polyline.id;
    breaked_poly_segments.seg_ptrs.reserve(max_lane_seg_num);
    bool is_in_box = false;
    for (int j = i * max_lane_seg_num;
         j < std::min((i + 1) * max_lane_seg_num, segs_size); ++j) {
      if (!is_in_box && region_box.IsPointIn(segs[j].seg.start())) {
        is_in_box = true;
      }
      breaked_poly_segments.seg_ptrs.push_back(&segs[j]);
    }
    if (!is_in_box) continue;
    poly_feats.push_back(ExtractAgentIndependentMapFeatureFromPolyline(
        breaked_poly_segments, max_lane_seg_num, coord_num));
  }
  return poly_feats;
}

void BuildPolylineFeatureMapFromPolylines(
    const Box2d& search_box, absl::Span<const SampledPolyline* const> polys,
    int max_lane_seg_num, int coord_num,
    std::unordered_map<mapping::ElementId,
                       std::vector<ActNetQcPolylineLocalFeature>>*
        id_poly_features_ptr) {
  QCHECK_NOTNULL(id_poly_features_ptr);
  SCOPED_QTRACE_ARG1(
      "ActNetLocalJ5Inferencer::BuildPolylineFeatureMapFromPolylines",
      "poly num", polys.size());
  auto& id_poly_map = *id_poly_features_ptr;
  for (const auto* sampled_poly_ptr : polys) {
    if (sampled_poly_ptr == nullptr || sampled_poly_ptr->segs.empty()) continue;
    if (id_poly_map.find(sampled_poly_ptr->id) != id_poly_map.end()) continue;
    id_poly_map[sampled_poly_ptr->id] =
        BreakSampledPolylineAndExtractActNetPolylineFeature(
            *sampled_poly_ptr, search_box, max_lane_seg_num, coord_num);
  }
}

inline QcNetLocalCoord ObjectMotionStateToQcNetLocalCoord(
    const ObjectMotionState& state) {
  QcNetLocalCoord coord;
  coord.pos = state.pos;
  coord.heading = state.heading;
  const auto normalized_rot_rad = NormalizeAngle(-coord.heading);
  coord.rot_sin = fast_math::SinNormalized(normalized_rot_rad);
  coord.rot_cos = fast_math::CosNormalized(normalized_rot_rad);
  return coord;
}

inline void CalculateRelativePosition(const QcNetLocalCoord& coord,
                                      const Vec2d& pos, double heading,
                                      absl::Span<float> container) {
  // Make sure pos and heading are in smooth coordinate!
  const auto orig_to_pos = pos - coord.pos;
  container[0] = orig_to_pos.norm();  // Distance.
  container[1] =
      NormalizeAngle(orig_to_pos.FastAngle() - coord.heading);  // Direction.
  container[2] = NormalizeAngle(heading - coord.heading);       // Orientation.
}

using PolylineMap =
    std::unordered_map<mapping::ElementId,
                       std::vector<ActNetQcPolylineLocalFeature>>;

void ComputeAgentToPolylineRelativeFeature(
    const Vec2d& smooth_pos, const ActNetQcPolylineLocalFeature& feat,
    int max_poly_seg_num, int coord_num, double* min_dist,
    std::vector<float>* relative_feats) {
  // Transform pos(agent's smooth pos) to polyline local coord to calcualte
  // distance.
  const auto& ct = feat.coord_trans;
  const auto pos = (smooth_pos - ct.orig).Rotate(ct.rot_cos, ct.rot_sin);
  *min_dist = std::numeric_limits<double>::max();
  relative_feats->reserve(max_poly_seg_num * 3);  // Dist, unit_dist_vec.
  for (int i = 0; i < max_poly_seg_num; ++i) {
    const int query_idx =
        i * coord_num * 2;  // A segment's start point x_coord.
    if (query_idx + 3 >= feat.seg.size()) break;  // Out of range?
    const Vec2d start(feat.seg[query_idx], feat.seg[query_idx + 1]);
    const Vec2d end(feat.seg[query_idx + 2], feat.seg[query_idx + 3]);
    const Segment2d recovered_seg(start, end);
    Vec2d nearest_pt;
    // Distance.
    relative_feats->push_back(recovered_seg.DistanceTo(pos, &nearest_pt));
    *min_dist = std::min<double>(*min_dist, relative_feats->back());
    // Unit dist vec in local coordinate!
    const auto unit_dist_vec = relative_feats->back() < 1e-3
                                   ? recovered_seg.unit_direction()
                                   : (nearest_pt - pos).Unit();
    relative_feats->push_back(unit_dist_vec.x());
    relative_feats->push_back(unit_dist_vec.y());
  }
}

void SortAndCollectPolylineFeaturesWithCacheAndIds(
    const Vec2d& agent_pos_smooth,
    const absl::flat_hash_set<mapping::ElementId>& polylines,
    const PolylineMap& id_poly_map, int max_poly_seg_num, int max_poly_feat_num,
    int coord_num, std::vector<ActNetQcPolylineFeature>* feat_ptrs,
    int* count) {
  SCOPED_QTRACE_ARG1("SortAndCollectPolylineFeatureWithCache",
                     "poly_nums:", polylines.size());
  // Extracted polyline feature is in AV-centered coordinate. Compute everything
  // in that coordinate.
  std::vector<std::pair<ActNetQcPolylineFeature, double>> feat_dist;
  feat_dist.reserve(polylines.size() * 2);
  for (const auto& id : polylines) {
    const auto* feats_ptr = FindOrNull(id_poly_map, id);
    if (feats_ptr == nullptr) continue;
    const auto& sampled_feats = *feats_ptr;
    for (const auto& feat : sampled_feats) {
      ActNetQcPolylineFeature qc_poly_feat;
      qc_poly_feat.abs_feat = &feat;
      double min_dist = std::numeric_limits<double>::max();
      ComputeAgentToPolylineRelativeFeature(
          agent_pos_smooth, feat, max_poly_seg_num, coord_num, &min_dist,
          &qc_poly_feat.agent_rel_feat);
      feat_dist.push_back(std::make_pair(std::move(qc_poly_feat), min_dist));
    }
  }
  // Sort by distance from entry point of the polyline to position.
  std::stable_sort(
      feat_dist.begin(), feat_dist.end(),
      [](const std::pair<ActNetQcPolylineFeature, double>& feat_dist1,
         const std::pair<ActNetQcPolylineFeature, double>& feat_dist2) {
        return feat_dist1.second < feat_dist2.second;
      });

  if (feat_dist.size() <= max_poly_feat_num) {
    *count = feat_dist.size();
  } else {
    feat_dist.resize(max_poly_feat_num);
    *count = max_poly_feat_num;
  }
  for (auto& pair : feat_dist) {
    feat_ptrs->push_back(std::move(pair.first));
  }
}

[[maybe_unused]] void SortAndCollectPolylineFeaturesWithCache(
    const Vec2d& agent_pos_smooth,
    absl::Span<const SampledPolyline* const> polylines,
    const PolylineMap& id_poly_map, int max_poly_seg_num, int max_poly_feat_num,
    int coord_num, std::vector<ActNetQcPolylineFeature>* feat_ptrs,
    int* count) {
  SCOPED_QTRACE_ARG1("SortAndCollectPolylineFeatureWithCache",
                     "poly_nums:", polylines.size());
  // Extracted polyline feature is in AV-centered coordinate. Compute
  // everything in that coordiante.
  std::vector<std::pair<ActNetQcPolylineFeature, double>> feat_dist;
  feat_dist.reserve(polylines.size() * 2);
  for (const auto* poly_ptr : polylines) {
    const auto* feats_ptr = FindOrNull(id_poly_map, poly_ptr->id);
    if (feats_ptr == nullptr) continue;
    const auto& sampled_feats = *feats_ptr;
    for (const auto& feat : sampled_feats) {
      ActNetQcPolylineFeature qc_poly_feat;
      qc_poly_feat.abs_feat = &feat;
      double min_dist = std::numeric_limits<double>::max();
      ComputeAgentToPolylineRelativeFeature(
          agent_pos_smooth, feat, max_poly_seg_num, coord_num, &min_dist,
          &qc_poly_feat.agent_rel_feat);
      feat_dist.push_back(std::make_pair(std::move(qc_poly_feat), min_dist));
    }
  }
  // Sort by distance from entry point of the polyline to position.
  std::stable_sort(
      feat_dist.begin(), feat_dist.end(),
      [](const std::pair<ActNetQcPolylineFeature, double>& feat_dist1,
         const std::pair<ActNetQcPolylineFeature, double>& feat_dist2) {
        return feat_dist1.second < feat_dist2.second;
      });

  if (feat_dist.size() <= max_poly_feat_num) {
    *count = feat_dist.size();
  } else {
    feat_dist.resize(max_poly_feat_num);
    *count = max_poly_feat_num;
  }
  for (auto& pair : feat_dist) {
    feat_ptrs->push_back(std::move(pair.first));
  }
}

void ExtractPolylineFeatureToTensor(
    const QcNetLocalCoord& agent_coord,
    absl::Span<const ActNetQcPolylineFeature> poly_feats, int batch_idx,
    int coord_num, HorizonTensorWrapper* pos, HorizonTensorWrapper* seg_info,
    HorizonTensorWrapper* attr_info, HorizonTensorWrapper* rel_pos) {
  SCOPED_QTRACE_ARG1("ExtractPolylineFeatureToTensor",
                     "poly_nums:", poly_feats.size());
  for (int poly_idx = 0; poly_idx < poly_feats.size(); ++poly_idx) {
    const auto& agent_rel_feat = poly_feats[poly_idx].agent_rel_feat;
    const auto& poly_feat = *poly_feats[poly_idx].abs_feat;  // qc local feats.
    const auto& ct = poly_feat.coord_trans;
    const int segs_size = poly_feat.seg_len.size();
    for (int seg_idx = 0; seg_idx < segs_size; ++seg_idx) {
      // pos. End point of each seg. (scaled) in local coord!
      const int seg_query_idx = coord_num * seg_idx * 2;
      pos->index_put_qat(batch_idx, 0, seg_idx, poly_idx,
                         static_cast<float>(poly_feat.seg[seg_query_idx + 2] *
                                            kInvCoordScale));
      pos->index_put_qat(batch_idx, 1, seg_idx, poly_idx,
                         static_cast<float>(poly_feat.seg[seg_query_idx + 3] *
                                            kInvCoordScale));
      // seg info: seg_len(scaled), unit_tangent, unit_seg_vec, dist(scaled),
      // unit_dist_vec, speed_limits(scaled).

      // Scaled seg_len.
      seg_info->index_put_qat(
          batch_idx, 0, seg_idx, poly_idx,
          static_cast<float>(poly_feat.seg_len[seg_idx] * kInvMaxSegLen));
      const int coord_query_idx = coord_num * seg_idx;
      const int rel_feat_query_idx = 3 * seg_idx;
      // Unit tangent.
      seg_info->index_put_qat(batch_idx, 1, seg_idx, poly_idx,
                              poly_feat.unit_tangent[coord_query_idx]);
      seg_info->index_put_qat(batch_idx, 2, seg_idx, poly_idx,
                              poly_feat.unit_tangent[coord_query_idx + 1]);
      // Unit seg vec.
      seg_info->index_put_qat(batch_idx, 3, seg_idx, poly_idx,
                              poly_feat.unit_seg_vec[coord_query_idx]);
      seg_info->index_put_qat(batch_idx, 4, seg_idx, poly_idx,
                              poly_feat.unit_seg_vec[coord_query_idx + 1]);
      // Agent-dependent polyline feature including: distance,
      // unit_dist_vec, nearest_to_end_dist.
      // Scaled distance.
      seg_info->index_put_qat(
          batch_idx, 5, seg_idx, poly_idx,
          static_cast<float>(agent_rel_feat[rel_feat_query_idx] *
                             kInvCoordScale));
      // Unit dist vec.
      seg_info->index_put_qat(batch_idx, 6, seg_idx, poly_idx,
                              agent_rel_feat[rel_feat_query_idx + 1]);
      seg_info->index_put_qat(batch_idx, 7, seg_idx, poly_idx,
                              agent_rel_feat[rel_feat_query_idx + 2]);
      // Scaled and clamped speed limit.
      seg_info->index_put_qat(
          batch_idx, 8, seg_idx, poly_idx,
          std::clamp(poly_feat.speed_limit_kph[seg_idx] * kInvSpeedLimitScale,
                     0.0f, 1.0f));
    }
    // attr. lane_light, lane_type. per poly.
    for (int light_idx = 0; light_idx < kActNetLocalJ5Config.lane_light_dim;
         ++light_idx) {
      attr_info->index_put_qat(batch_idx, light_idx, 0, poly_idx,
                               poly_feat.light[light_idx]);
    }
    const int type = static_cast<int>(poly_feat.type.back());
    attr_info->index_put_qat(batch_idx,
                             kActNetLocalJ5Config.lane_light_dim + type, 0,
                             poly_idx, 1.0f);
    // Calculate relative position info. use agent coord and polyline local
    // coord transform's info.
    float poly_rel_pos[3];
    CalculateRelativePosition(agent_coord, ct.orig,
                              /*heading=*/-ct.rot_rad,
                              absl::MakeSpan(poly_rel_pos));
    rel_pos->index_put_qat(batch_idx, 0, 0, poly_idx,
                           poly_rel_pos[0] * kInvCoordScale);
    rel_pos->index_put_qat(batch_idx, 1, 0, poly_idx, poly_rel_pos[1]);
    rel_pos->index_put_qat(batch_idx, 2, 0, poly_idx, poly_rel_pos[2]);
  }
}

}  // namespace

ActNetLocalJ5Inferencer::ActNetLocalJ5Inferencer(const NetParam& net_param)
    : net_param_(net_param) {
  act_net_local_j5_ = std::make_unique<PredictionJ5QNN>(net_param_);
}

AgentCentricObjectsOut ActNetLocalJ5Inferencer::PredictForObjects(
    absl::Span<const ObjectHistory* const> objects_history,
    const ObjectHistorySampler& obj_sampler, MapSampler* map_sampler_ptr,
    ThreadPool* thread_pool) const {
  SCOPED_QTRACE("ActNetLocalJ5Inferencer::PredictForObjects");

  AgentCentricObjectsOut objs_out;

  std::vector<AgentRef> agent_origins;
  int current_batch_size = 0;

  // 1. Feature to batch inputs.
  {
    SCOPED_QTRACE("ActNetLocalJ5Inferencer::GetBatchInputs");
    current_batch_size = GetBatchInputs(objects_history, obj_sampler,
                                        map_sampler_ptr, agent_origins);
    if (current_batch_size == 0) {
      return objs_out;
    }
  }

  // 2. Forward.
  {
    SCOPED_QTRACE("ActNetLocalJ5Inferencer::ProcessInEngine");
    act_net_local_j5_->Run(current_batch_size);
  }

  // 3. Set objects
  {
    SCOPED_QTRACE("ActNetLocalJ5Inferencer::AssembleProbTrajs");
    objs_out = ExtractOutputs(current_batch_size, agent_origins, thread_pool);
  }
  return objs_out;
}

AgentCentricObjectsOut ActNetLocalJ5Inferencer::ExtractOutputs(
    int current_batch_size, const std::vector<AgentRef>& agent_origins,
    ThreadPool* thread_pool) const {
  const auto& batch_out_map = act_net_local_j5_->GetOutputMap();
  const int traj_size = kFutureNum * kOutCoords;
  AgentCentricObjectsOut objs_out;
  const auto& batch_traj_probs = batch_out_map.at("traj_probs");
  const auto& batch_startup_prob = batch_out_map.at("startup_prob");

  std::vector<AgentCentricObjectOut> agent_centric_objects;
  agent_centric_objects.resize(current_batch_size);

  ParallelFor(0, current_batch_size, thread_pool, [&](int i) {
    AgentCentricObjectProbTrajs object_prob_trajs;
    object_prob_trajs.reserve(kTrajectoryNum);
    const double inv_rot = NormalizeAngle(-agent_origins[i].rot_rad);
    double inv_rot_cos_sin[2];
    fast_math::CosSinNormalized(inv_rot, inv_rot_cos_sin);

    // Assign probabilities.
    // softmax prob
    std::vector<double> prob_mod;
    prob_mod.reserve(kTrajectoryNum);
    for (int j = 0; j < kTrajectoryNum; ++j) {
      prob_mod.push_back(batch_traj_probs->index_get_deqat(i, traj_size, 0, j));
    }
    prob_mod = SoftMax(prob_mod);
    double sum_prob = 0.0;
    for (int j = 0; j < kTrajectoryNum; ++j) {
      if (prob_mod[j] < kProbThreshold) {
        continue;
      }
      sum_prob += prob_mod[j];
      std::vector<NLLTrajPoint> traj_points;
      traj_points.reserve(kFutureNum);
      for (int k = 0; k < kFutureNum; ++k) {
        int query_idx = k * kOutCoords;
        // (x, y) coord convert.
        Vec2d pt(batch_traj_probs->index_get_deqat(i, query_idx, 0, j),
                 batch_traj_probs->index_get_deqat(i, query_idx + 1, 0, j));
        pt = pt.Rotate(inv_rot_cos_sin[0], inv_rot_cos_sin[1]) +
             agent_origins[i].ref_position;
        traj_points.push_back(NLLTrajPoint{
            pt.x(),
            pt.y(),
            batch_traj_probs->index_get_deqat(i, query_idx + 2, 0, j),
            batch_traj_probs->index_get_deqat(i, query_idx + 3, 0, j),
            batch_traj_probs->index_get_deqat(i, query_idx + 4, 0, j),
        });
      }

      object_prob_trajs.push_back(AgentCentricObjectProbTraj({
          .mode_prob = prob_mod[j],
          .relation_probs = {0.0, 0.0, 0.0},
          .traj_points = std::move(traj_points),
          // Angle between ac coord and smooth coord.
          .rot_rad = inv_rot,
      }));
    }
    // Normalize mod prob will be done in predictor.
    QCHECK_GT(sum_prob, 0.0);
    // Sort prob_trajs by probability descending.
    if (object_prob_trajs.size() > 0) {
      std::sort(object_prob_trajs.begin(), object_prob_trajs.end(),
                [](const auto& a, const auto& b) {
                  return a.mode_prob > b.mode_prob;
                });
      // only out kActNetJ5ModelTrajNum trajs.
      if (object_prob_trajs.size() > kActNetJ5ModelTrajNum) {
        object_prob_trajs.resize(kActNetJ5ModelTrajNum);
      }
      agent_centric_objects[i] = AgentCentricObjectOut({
          .prob_trajs = std::move(object_prob_trajs),
          .startup_prob =
              Sigmoid(batch_startup_prob->index_get_deqat(i, 0, 0, 0)),
      });
    }
  });
  for (int i = 0; i < current_batch_size; ++i) {
    if (agent_centric_objects[i].prob_trajs.size() > 0) {
      objs_out[agent_origins[i].agent_id] = agent_centric_objects[i];
    }
  }
  return objs_out;
}

int ActNetLocalJ5Inferencer::GetBatchInputs(
    absl::Span<const ObjectHistory* const> objects_history,
    const ObjectHistorySampler& obj_sampler, MapSampler* map_sampler_ptr,
    std::vector<AgentRef>& agent_origins) const {
  SCOPED_QTRACE_ARG1("PrepareActNetLocalJ5Inputs",
                     "objs_num:", objects_history.size());
  act_net_local_j5_->ResetInputs();

  if (objects_history.size() == 0) {
    return 0;
  }

  // 1. Find all related objects and build cache for absolute features.
  agent_origins.resize(objects_history.size());
  std::unordered_map<ObjectIDType, ActNetQcObjectFeature> id_obj_abs_feat_map;
  // This is the largest search box defined by all objects in the scene.
  Box2d search_box;
  BuildActNetObjectAbsoluteFeatureMap(objects_history, obj_sampler,
                                      &id_obj_abs_feat_map, &search_box,
                                      agent_origins);

  // 2. Find all related lcs and lbs. Build cache. Extract all polyline features
  // in search box.
  const auto* av_history = obj_sampler.GetResampledAVMotionHistory();
  const auto& av_pos = av_history->states.back().pos;
  constexpr double kAvMapRadius = 210.0;
  const auto lane_centers =
      map_sampler_ptr->GetLaneCentersWithRadius(av_pos, kAvMapRadius);
  const auto lane_boundaries =
      map_sampler_ptr->GetSolidLaneBoundariesWithRadius(av_pos, kAvMapRadius);
  const auto all_lcs =
      map_sampler_ptr->GetNearestLaneCenterPolylinesInBox2dWithLanes(
          search_box, /*max_num=*/kActNetLocalJ5Config.max_lc_num, lane_centers,
          /*compute_boundary_distance=*/false);
  const auto all_lbs =
      map_sampler_ptr->GetNearestLaneBoundaryPolylinesInBox2dWithLaneBoundaries(
          search_box, /*max_num=*/kActNetLocalJ5Config.max_lb_num,
          lane_boundaries);

  std::unordered_map<mapping::ElementId,
                     std::vector<ActNetQcPolylineLocalFeature>>
      id_polyfeat_map;
  BuildPolylineFeatureMapFromPolylines(search_box, absl::MakeSpan(all_lcs),
                                       kActNetLocalJ5Config.max_lane_seg_num,
                                       kActNetLocalJ5Config.coord_num,
                                       &id_polyfeat_map);
  BuildPolylineFeatureMapFromPolylines(search_box, absl::MakeSpan(all_lbs),
                                       kActNetLocalJ5Config.max_lane_seg_num,
                                       kActNetLocalJ5Config.coord_num,
                                       &id_polyfeat_map);

  // 3. Iterate all objects (to predict), fill tensors with cached feature and
  // calculated relative position.
  const auto& batch_map = act_net_local_j5_->GetInputMap();
  auto& actor_info_hist = *batch_map.at("actor_info_hist");
  auto& actor_info_attr = *batch_map.at("actor_info_attr");
  auto& actor_ctrs = *batch_map.at("actor_ctrs");
  auto& lc_pos = *batch_map.at("lc_pos");
  auto& lc_seg_info = *batch_map.at("lc_seg_info");
  auto& lc_attr_info = *batch_map.at("lc_attr_info");
  auto& lb_pos = *batch_map.at("lb_pos");
  auto& lb_seg_info = *batch_map.at("lb_seg_info");
  auto& lb_attr_info = *batch_map.at("lb_attr_info");
  // act net sc specific inputs.
  auto& actor_rel_pos = *batch_map.at("actor_rel_pos");
  auto& lb_rel_pos = *batch_map.at("lb_rel_pos");
  auto& lc_rel_pos = *batch_map.at("lc_rel_pos");

  const auto& coord_num = kActNetLocalJ5Config.coord_num;
  const auto& hist_num = kActNetLocalJ5Config.hist_num;
  const auto hist_idx_offset = prediction::kFeatureV2HistoryStepNum - hist_num;

  for (int batch_idx = 0; batch_idx < objects_history.size(); ++batch_idx) {
    // 1.0 Agent + context objects.
    const auto& agent_hist = *objects_history[batch_idx];
    const auto& agent_id = agent_hist.id();
    const auto& agent_motion_hist =
        obj_sampler.GetResampledMotionHistoryById(agent_id);
    const auto& latest_state = agent_motion_hist.states.back();
    // Used to calculate relative position.
    const auto agent_coord = ObjectMotionStateToQcNetLocalCoord(latest_state);
    // Used to find related map and objects.
    const auto agent_region_box = GetRegionBoxForObject(
        latest_state.pos, latest_state.heading, latest_state.vel.Length(),
        agent_motion_hist.type);
    // Find all related objects' absolute act net feature including agent
    // itself.
    const auto objs_in_box = obj_sampler.GetResampledMotionHistoryWithAVInBox2d(
        agent_id, agent_region_box, kActNetLocalJ5Config.max_other_objs_num);
    const auto& agent_feat = id_obj_abs_feat_map[agent_id];
    // Fill agent hist info.
    for (int hist_idx = 0; hist_idx < hist_num; ++hist_idx) {
      const int query_idx = (hist_idx_offset + hist_idx) * coord_num;
      // pos_diff.
      actor_info_hist.index_put_qat(batch_idx, 0, hist_idx, 0,
                                    agent_feat.pos_diff[query_idx]);
      actor_info_hist.index_put_qat(batch_idx, 1, hist_idx, 0,
                                    agent_feat.pos_diff[query_idx + 1]);
      // Scaled speed.
      actor_info_hist.index_put_qat(
          batch_idx, 2, hist_idx, 0,
          agent_feat.speed[query_idx] * kInvSpeedScale);
      actor_info_hist.index_put_qat(
          batch_idx, 3, hist_idx, 0,
          agent_feat.speed[query_idx + 1] * kInvSpeedScale);
      // heading.
      actor_info_hist.index_put_qat(batch_idx, 4, hist_idx, 0,
                                    agent_feat.heading[query_idx]);
      actor_info_hist.index_put_qat(batch_idx, 5, hist_idx, 0,
                                    agent_feat.heading[query_idx + 1]);
      // Scaled pos.
      actor_ctrs.index_put_qat(batch_idx, 0, hist_idx, 0,
                               agent_feat.pos[query_idx] * kInvCoordScale);
      actor_ctrs.index_put_qat(batch_idx, 1, hist_idx, 0,
                               agent_feat.pos[query_idx + 1] * kInvCoordScale);
    }
    // Fill agent attr info: type, scaled lws.
    const int type = static_cast<int>(agent_feat.type);
    actor_info_attr.index_put_qat(batch_idx, type, 0, 0, 1.0f);
    actor_info_attr.index_put_qat(
        batch_idx, 16, 0, 0,
        std::clamp(static_cast<float>(latest_state.bbox.length()) *
                       kInvObjectLengthScale,
                   0.0f, 1.0f));
    actor_info_attr.index_put_qat(
        batch_idx, 17, 0, 0,
        std::clamp(static_cast<float>(latest_state.bbox.width()) *
                       kInvObjectWidthScale,
                   0.0f, 1.0f));

    {  // context objects.
      for (int obj_idx = 0; obj_idx < objs_in_box.size(); ++obj_idx) {
        const auto* obj_motion_hist = objs_in_box[obj_idx];
        const auto& obj_feat = id_obj_abs_feat_map[obj_motion_hist->id];
        const auto& latest_state = obj_motion_hist->states.back();
        // Calculate relative position for this objects.
        float obj_rel_pos[3];
        CalculateRelativePosition(agent_coord, latest_state.pos,
                                  latest_state.heading,
                                  absl::MakeSpan(obj_rel_pos));
        // Fill actor_rel_pos tensor.
        actor_rel_pos.index_put_qat(batch_idx, 0, 0, obj_idx + 1,
                                    obj_rel_pos[0] * kInvCoordScale);
        actor_rel_pos.index_put_qat(batch_idx, 1, 0, obj_idx + 1,
                                    obj_rel_pos[1]);
        actor_rel_pos.index_put_qat(batch_idx, 2, 0, obj_idx + 1,
                                    obj_rel_pos[2]);
        // Fill tensors.
        for (int hist_idx = 0; hist_idx < hist_num; ++hist_idx) {
          const int query_idx = (hist_idx_offset + hist_idx) * coord_num;
          // pos diff.
          actor_info_hist.index_put_qat(batch_idx, 0, hist_idx, obj_idx + 1,
                                        obj_feat.pos_diff[query_idx]);
          actor_info_hist.index_put_qat(batch_idx, 1, hist_idx, obj_idx + 1,
                                        obj_feat.pos_diff[query_idx + 1]);
          // Scaled speed.
          actor_info_hist.index_put_qat(
              batch_idx, 2, hist_idx, obj_idx + 1,
              obj_feat.speed[query_idx] * kInvSpeedScale);
          actor_info_hist.index_put_qat(
              batch_idx, 3, hist_idx, obj_idx + 1,
              obj_feat.speed[query_idx + 1] * kInvSpeedScale);
          // heading.
          actor_info_hist.index_put_qat(batch_idx, 4, hist_idx, obj_idx + 1,
                                        obj_feat.heading[query_idx]);
          actor_info_hist.index_put_qat(batch_idx, 5, hist_idx, obj_idx + 1,
                                        obj_feat.heading[query_idx + 1]);
          // Scaled pos.
          actor_ctrs.index_put_qat(batch_idx, 0, hist_idx, obj_idx + 1,
                                   obj_feat.pos[query_idx] * kInvCoordScale);
          actor_ctrs.index_put_qat(
              batch_idx, 1, hist_idx, obj_idx + 1,
              obj_feat.pos[query_idx + 1] * kInvCoordScale);
        }
        // Fill object attr (type and scaled lws).
        const int type = static_cast<int>(obj_feat.type);
        actor_info_attr.index_put_qat(batch_idx, type, 0, obj_idx + 1, 1.0f);
        actor_info_attr.index_put_qat(
            batch_idx, 16, 0, obj_idx + 1,
            std::clamp(static_cast<float>(latest_state.bbox.length()) *
                           kInvObjectLengthScale,
                       0.0f, 1.0f));
        actor_info_attr.index_put_qat(
            batch_idx, 17, 0, obj_idx + 1,
            std::clamp(static_cast<float>(latest_state.bbox.width()) *
                           kInvObjectWidthScale,
                       0.0f, 1.0f));
      }
    }

    // 2.0 lc and lb features.
    std::vector<ActNetQcPolylineFeature> poly_feats;
    // lcs.
    int lc_num = 0;
    // Method 2: Gather lane ids only.
    const auto lcs_info = map_sampler_ptr->GetLaneCentersWithRadius(
        agent_region_box.center(), agent_region_box.diagonal() * 0.5);
    absl::flat_hash_set<mapping::ElementId> lc_ids;
    for (const auto* lc_info : lcs_info) {
      lc_ids.insert(lc_info->id);
    }
    SortAndCollectPolylineFeaturesWithCacheAndIds(
        agent_coord.pos, lc_ids, id_polyfeat_map,
        kActNetLocalJ5Config.max_lane_seg_num, kActNetLocalJ5Config.max_lc_num,
        kActNetLocalJ5Config.coord_num, &poly_feats, &lc_num);
    ExtractPolylineFeatureToTensor(
        agent_coord, absl::MakeSpan(&poly_feats[0], lc_num), batch_idx,
        coord_num, &lc_pos, &lc_seg_info, &lc_attr_info, &lc_rel_pos);

    // lbs.
    int lb_num = 0;
    const auto lbs_info = map_sampler_ptr->GetSolidLaneBoundariesWithRadius(
        agent_region_box.center(), agent_region_box.diagonal() * 0.5);
    absl::flat_hash_set<mapping::ElementId> lb_ids;
    for (const auto* lb_info : lbs_info) {
      lb_ids.insert(lb_info->id);
    }
    SortAndCollectPolylineFeaturesWithCacheAndIds(
        agent_coord.pos, lb_ids, id_polyfeat_map,
        kActNetLocalJ5Config.max_lane_seg_num, kActNetLocalJ5Config.max_lb_num,
        kActNetLocalJ5Config.coord_num, &poly_feats, &lb_num);
    ExtractPolylineFeatureToTensor(
        agent_coord, absl::MakeSpan(&poly_feats[lc_num], lb_num), batch_idx,
        coord_num, &lb_pos, &lb_seg_info, &lb_attr_info, &lb_rel_pos);
  }
  return objects_history.size();
}

}  // namespace actnetlocalj5
}  // namespace prediction
}  // namespace qcraft
