#include "onboard/prediction/feature_extractor/act_net_feature_extractor.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <initializer_list>
#include <iterator>
#include <limits>
#include <optional>  // for optional, nullopt
#include <utility>
#include <vector>

#include "absl/status/statusor.h"
#include "absl/types/span.h"

#include "onboard/global/buffered_logger.h"
#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/math/fast_math.h"
#include "onboard/math/frenet_common.h"  // for FrenetCoordinate
#include "onboard/math/frenet_frame.h"
#include "onboard/math/geometry/segment2d.h"
#include "onboard/math/geometry/util.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/prediction/feature_extractor/map_sampler.h"
#include "onboard/prediction/feature_extractor/object_history_sampler.h"
#include "onboard/prediction/prediction_defs.h"
namespace qcraft {
namespace prediction {
namespace {
constexpr int kBoxCornerNum = 4;
constexpr double kEpsilonS = 1e-1;
constexpr double kStopTimeInfoClamp = 60.0;
constexpr double kMinPathLength = 0.5;

ActNetPolylineFeature ExtractMapFeatureFromPolyline(
    const SampledPolylineWithConstSegRef& polyline,
    const AgentCoordTransform& agent_coord_transform, int max_lane_seg_num) {
  const int& k_coord_num = kActNetConfig.coord_num;
  const int k_light_num = 4;
  const auto& orig = agent_coord_transform.orig;
  const double rot_cos = agent_coord_transform.rot_cos;
  const double rot_sin = agent_coord_transform.rot_sin;

  ActNetPolylineFeature feat;
  const auto& seg_ptrs = polyline.seg_ptrs;
  const int segs_size = seg_ptrs.size();
  QCHECK_GT(segs_size, 0);
  feat.id = polyline.id;
  feat.seg.resize(max_lane_seg_num * k_coord_num * 2, 0.0);
  feat.unit_seg_vec.resize(max_lane_seg_num * k_coord_num, 0.0);
  feat.seg_len.resize(max_lane_seg_num, 0.0);
  feat.unit_tangent.resize(max_lane_seg_num * k_coord_num, 0.0);
  feat.dist.resize(max_lane_seg_num, 0.0);
  feat.unit_dist_vec.resize(max_lane_seg_num * k_coord_num, 0.0);
  feat.nearest_to_end_dist.resize(max_lane_seg_num, 0.0);
  feat.yaw_diff.resize(max_lane_seg_num * k_coord_num, 0.0);
  feat.type.resize(max_lane_seg_num, 0.0);
  feat.light.resize(max_lane_seg_num * k_light_num, 0.0);
  feat.mask.resize(max_lane_seg_num, 0.0);
  feat.speed_limit_kph.resize(max_lane_seg_num, 0.0);
  feat.lb_seg.resize(max_lane_seg_num * k_coord_num * 2, 0.0);
  feat.seg_width.resize(max_lane_seg_num, 0.0);
  feat.boundary_type.resize(max_lane_seg_num * 2, 0.0);
  for (int i = 0; i < segs_size; ++i) {
    const auto& cur = seg_ptrs[i]->seg.start();
    const auto& next = seg_ptrs[i]->seg.end();
    const auto cur_pos = (cur - orig).Rotate(rot_cos, rot_sin);
    const auto next_pos = (next - orig).Rotate(rot_cos, rot_sin);

    feat.seg[i * k_coord_num * 2] = cur_pos.x();
    feat.seg[i * k_coord_num * 2 + 1] = cur_pos.y();
    feat.seg[i * k_coord_num * 2 + 2] = next_pos.x();
    feat.seg[i * k_coord_num * 2 + 3] = next_pos.y();

    const auto seg_vec = next_pos - cur_pos;
    const auto seg_len = seg_vec.norm();
    const auto unit_seg_vec = seg_vec.normalized();
    feat.unit_seg_vec[i * k_coord_num] = unit_seg_vec.x();
    feat.unit_seg_vec[i * k_coord_num + 1] = unit_seg_vec.y();
    feat.seg_len[i] = seg_len;

    const auto unit_tangent =
        seg_ptrs[i]->unit_tangent.Rotate(rot_cos, rot_sin);
    feat.unit_tangent[i * k_coord_num] = unit_tangent.x();
    feat.unit_tangent[i * k_coord_num + 1] = unit_tangent.y();

    Vec2d nearest_pt;
    const Segment2d rotated_seg(cur_pos, next_pos);
    const Vec2d agent_cur_pos(0.0, 0.0);
    const double dist = rotated_seg.DistanceTo(agent_cur_pos, &nearest_pt);
    feat.dist[i] = dist;
    const Vec2d unit_dist_vec = nearest_pt != agent_cur_pos
                                    ? (nearest_pt - agent_cur_pos).Unit()
                                    : rotated_seg.unit_direction().Perp();
    feat.unit_dist_vec[i * k_coord_num] = unit_dist_vec.x();
    feat.unit_dist_vec[i * k_coord_num + 1] = unit_dist_vec.y();

    const double nearest_to_end_dist = (nearest_pt - next_pos).norm();
    feat.nearest_to_end_dist[i] = nearest_to_end_dist;

    const auto yaw_diff =
        Vec2d::UnitFromAngle(seg_vec.Angle() - unit_dist_vec.Angle());
    feat.yaw_diff[i * k_coord_num] = yaw_diff.x();
    feat.yaw_diff[i * k_coord_num + 1] = yaw_diff.y();

    feat.type[i] = static_cast<float>(seg_ptrs[i]->type);
    for (int k = 0; k < k_light_num; ++k) {
      feat.light[i * k_light_num + k] = seg_ptrs[i]->lights[k];
    }
    feat.speed_limit_kph[i] = seg_ptrs[i]->speed_limit_kph;
    feat.mask[i] = 1.0;
    feat.boundary_type[i * 2] =
        static_cast<float>(seg_ptrs[i]->boundary_types[0]);
    feat.boundary_type[i * 2 + 1] =
        static_cast<float>(seg_ptrs[i]->boundary_types[1]);
    if (seg_ptrs[i]->distance_to_boundary.has_value()) {
      const double dist = *seg_ptrs[i]->distance_to_boundary;
      feat.seg_width[i] = dist * 2.0;
      const auto perp = unit_seg_vec.Perp();
      const auto lb_point = cur_pos + perp * dist;
      const auto rb_point = cur_pos - perp * dist;
      feat.lb_seg[i * k_coord_num * 2] = lb_point.x();
      feat.lb_seg[i * k_coord_num * 2 + 1] = lb_point.y();
      feat.lb_seg[i * k_coord_num * 2 + 2] = rb_point.x();
      feat.lb_seg[i * k_coord_num * 2 + 3] = rb_point.y();
    }
  }
  return feat;
}
std::vector<SampledPolylineWithConstSegRef> BreakAndFilterPolylines(
    absl::Span<const SampledPolyline* const> polylines, const Box2d& region_box,
    int max_lane_seg_num) {
  SCOPED_QTRACE("BreakAndFilterPolylines");
  QCHECK_GT(max_lane_seg_num, 0);
  std::vector<SampledPolylineWithConstSegRef> breaked_polylines;
  breaked_polylines.reserve(polylines.size() * 2);
  for (const auto* poly_ptr : polylines) {
    const auto& polyline = *poly_ptr;
    const auto& segs = polyline.segs;
    const int segs_size = segs.size();
    if (segs_size == 0) {
      continue;
    }
    int num_breaks = 0;
    if (segs_size % max_lane_seg_num == 0) {
      num_breaks = segs_size / max_lane_seg_num;
    } else {
      num_breaks = segs_size / max_lane_seg_num + 1;
    }
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
      breaked_polylines.push_back(std::move(breaked_poly_segments));
    }
  }
  return breaked_polylines;
}

std::optional<ActNetCutInInfo> ExtractActNetCutinInfo(
    const planner::DrivePassage* dp, const Vec2d& av_pos,
    const AgentCoordTransform& agent_coord_transform) {
  const Vec2d& ref_position = agent_coord_transform.orig;
  const double rot_rad = agent_coord_transform.rot_rad;
  const double rot_cos = agent_coord_transform.rot_cos;
  const double rot_sin = agent_coord_transform.rot_sin;
  if (dp == nullptr) {
    return std::nullopt;
  }
  const auto av_sl_or = dp->QueryFrenetCoordinateAt(av_pos);
  if (!av_sl_or.ok()) {
    return std::nullopt;
  }
  const double path_len =
      std::min(dp->end_s() - av_sl_or->s, kActNetConfig.max_path_length);
  if (path_len <= kMinPathLength) {
    return std::nullopt;
  }
  ActNetCutInInfo info;
  QCHECK_GT(kActNetConfig.max_path_num, 1);
  const double ds = path_len / (kActNetConfig.max_path_num - 1);
  std::vector<Vec2d> path_points;
  path_points.reserve(kActNetConfig.max_path_num);
  info.path_xy.reserve(kActNetConfig.max_path_num * kActNetConfig.coord_num);
  info.headings.reserve(kActNetConfig.max_path_num * kActNetConfig.coord_num);
  info.dist.reserve(kActNetConfig.max_path_num);
  info.unit_dist_vec.reserve(kActNetConfig.max_path_num *
                             kActNetConfig.coord_num);
  for (int i = 0; i < kActNetConfig.max_path_num; ++i) {
    const double cur_s = av_sl_or->s + i * ds;
    const auto pos_or = dp->QueryPointXYAtS(cur_s);
    const auto tangent_angle_or = dp->QueryTangentAngleAtS(cur_s);
    if (!(pos_or.ok() && tangent_angle_or.ok())) {
      return std::nullopt;
    }
    const auto pos = (pos_or.value() - ref_position).Rotate(rot_cos, rot_sin);
    path_points.push_back(pos);
    const auto angle = tangent_angle_or.value() + rot_rad;
    double dis = pos.norm();
    const auto dis_vec = pos.Unit();
    info.path_xy.push_back(pos.x());
    info.path_xy.push_back(pos.y());
    info.headings.push_back(fast_math::Cos(angle));
    info.headings.push_back(fast_math::Sin(angle));
    info.dist.push_back(dis);
    info.unit_dist_vec.push_back(dis_vec.x());
    info.unit_dist_vec.push_back(dis_vec.y());
  }
  info.already_on_path = false;
  const auto ff_or =
      BuildBruteForceFrenetFrame(path_points, /*down_sample_raw_points=*/true);
  if (ff_or.ok()) {
    // Project
    FrenetCoordinate sl = ff_or->XYToSL(Vec2d(0.0, 0.0));
    // If projected to the first or last idx, ignore.
    if (sl.s >= ff_or->start_s() && sl.s <= ff_or->end_s()) {
      // If projected distance is less than Cutin distance, meaning that
      // object's
      // center is witin the AV's range.
      if (std::fabs(sl.l) <= kActNetConfig.cutin_distance) {
        info.already_on_path = true;
      }
    }
  }
  return info;
}

void SortPolysByDist(
    const Vec2d& origin, const int max_feat_num,
    std::vector<const SampledPolylineWithConstSegRef*>* filtered_polys) {
  if (filtered_polys->size() > max_feat_num) {
    std::vector<std::pair<const SampledPolylineWithConstSegRef*, float>>
        polys_dist;
    polys_dist.reserve(filtered_polys->size());
    for (const auto* poly : *filtered_polys) {
      double dist = std::numeric_limits<double>::max();
      for (const auto& seg : poly->seg_ptrs) {
        const auto& cur = seg->seg.start();
        const auto& next = seg->seg.end();
        const Segment2d line_segment(cur, next);
        Vec2d nearest_pt;
        dist = std::min(dist, line_segment.DistanceTo(origin, &nearest_pt));
      }
      polys_dist.push_back({poly, dist});
    }
    std::sort(polys_dist.begin(), polys_dist.end(),
              [](const auto& feat1, const auto feat2) {
                return feat1.second < feat2.second;
              });
    filtered_polys->clear();
    for (int i = 0; i < max_feat_num; ++i) {
      filtered_polys->push_back(polys_dist[i].first);
    }
  }
}
}  // namespace

std::vector<ActNetPolylineFeature> ExtractMapFeatureFromPolylines(
    absl::Span<const SampledPolyline* const> polylines, const Box2d& cur_box,
    const AgentCoordTransform& agent_coord_transform, int max_lane_seg_num,
    int max_feat_num) {
  const std::vector<SampledPolylineWithConstSegRef> filtered_polys =
      BreakAndFilterPolylines(polylines, cur_box, max_lane_seg_num);
  std::vector<const SampledPolylineWithConstSegRef*> filtered_poly_ptrs;
  filtered_poly_ptrs.reserve(filtered_polys.size());
  for (int i = 0; i < filtered_polys.size(); ++i) {
    filtered_poly_ptrs.push_back(&filtered_polys[i]);
  }
  // sorted by dist
  SortPolysByDist(agent_coord_transform.orig, max_feat_num,
                  &filtered_poly_ptrs);
  if (filtered_poly_ptrs.size() > max_feat_num) {
    filtered_poly_ptrs.resize(max_feat_num);
  }

  std::vector<ActNetPolylineFeature> combined_feats;
  combined_feats.reserve(filtered_poly_ptrs.size());
  for (const auto* poly_ptr : filtered_poly_ptrs) {
    combined_feats.push_back(ExtractMapFeatureFromPolyline(
        *poly_ptr, agent_coord_transform, max_lane_seg_num));
  }
  return combined_feats;
}

ActNetFeature ExtractActNetFeature(
    const ObjectIDType& agent_id, const ObjectHistorySampler& obj_sampler,
    const StopTimeInfo& stop_time_info,
    const planner::DrivePassage* drive_passage_ptr,
    MapSampler* const map_sampler_ptr, const AvMapCache& av_map_cache,
    const Box2d& region_box) {
  auto& map_sampler = *map_sampler_ptr;
  const auto& agent_history =
      obj_sampler.GetResampledMotionHistoryById(agent_id);
  const double heading = agent_history.states.back().heading;
  auto ref_position = agent_history.states.back().pos;
  const double rot_rad = -heading;

  const double normal_rot_rad = NormalizeAngle(rot_rad);
  const auto agent_coord_transform =
      AgentCoordTransform{.orig = ref_position,
                          .rot_rad = rot_rad,
                          .rot_sin = fast_math::SinNormalized(normal_rot_rad),
                          .rot_cos = fast_math::CosNormalized(normal_rot_rad)};

  // 1. Get agent feature.
  auto agent_feat = GetActNetObjectAbsoluteFeature(
      agent_history, agent_coord_transform, kActNetConfig.coord_num,
      kFeatureV2HistoryStepLen, kFeatureV2HistoryStepNum);

  // 2. Get objects feature.
  const auto objects_in_box2d =
      obj_sampler.GetResampledMotionHistoryWithAVInBox2d(
          agent_id, region_box, kActNetConfig.max_other_objects_num);
  std::vector<ActNetObjectFeature> context_obj_features;
  context_obj_features.reserve(objects_in_box2d.size());
  for (const auto* obj_hist : objects_in_box2d) {
    auto abs_feat = GetActNetObjectAbsoluteFeature(
        *obj_hist, agent_coord_transform, kActNetConfig.coord_num,
        kFeatureV2HistoryStepLen, kFeatureV2HistoryStepNum);

    auto rel_feat =
        GetActNetObjectRelFeature(agent_feat, abs_feat, kActNetConfig.coord_num,
                                  kFeatureV2HistoryStepNum);
    context_obj_features.push_back(ActNetObjectFeature{
        .id = obj_hist->id,
        .abs_feat = std::move(abs_feat),
        .rel_feat = std::move(rel_feat),
    });
  }
  std::vector<float> context_obj_masks(kActNetConfig.max_other_objects_num,
                                       0.0);
  for (int i = 0; i < context_obj_features.size(); ++i) {
    context_obj_masks[i] = 1.0;
  }

  // 3. Get map feature.
  const auto lcs = map_sampler.GetNearestLaneCenterPolylinesInBox2dWithLanes(
      region_box, kActNetConfig.max_lc_num, av_map_cache.lane_centers,
      /*compute_boundary_distance=*/true);
  auto lc_feats = ExtractMapFeatureFromPolylines(
      lcs, region_box, agent_coord_transform, kActNetConfig.max_lane_seg_num,
      kActNetConfig.max_lc_num);
  std::vector<float> lc_masks(kActNetConfig.max_lc_num, 0.0);
  for (int i = 0; i < lc_feats.size(); ++i) {
    lc_masks[i] = 1.0;
  }

  const auto solid_lbs =
      map_sampler.GetNearestLaneBoundaryPolylinesInBox2dWithLaneBoundaries(
          region_box, kActNetConfig.max_solid_lb_num,
          av_map_cache.solid_lane_boundarys);
  auto solid_lb_feats = ExtractMapFeatureFromPolylines(
      solid_lbs, region_box, agent_coord_transform,
      kActNetConfig.max_lane_seg_num, kActNetConfig.max_solid_lb_num);
  std::vector<float> solid_lb_masks(kActNetConfig.max_solid_lb_num, 0.0);
  for (int i = 0; i < solid_lb_feats.size(); ++i) {
    solid_lb_masks[i] = 1.0;
  }

  const auto cws =
      map_sampler.GetNearestCrosswalkPolylinesInBox2dWithCrosswalks(
          region_box, kActNetConfig.max_crosswalk_num,
          av_map_cache.cross_walks);
  auto cw_feats = ExtractMapFeatureFromPolylines(
      cws, region_box, agent_coord_transform, kActNetConfig.max_lane_seg_num,
      kActNetConfig.max_crosswalk_num);
  std::vector<float> cw_masks(kActNetConfig.max_crosswalk_num, 0.0);
  for (int i = 0; i < cw_feats.size(); ++i) {
    cw_masks[i] = 1.0;
  }

  // Get stop time info.
  std::vector<float> stop_time_info_feats;
  stop_time_info_feats.reserve(3);
  stop_time_info_feats.push_back(
      std::clamp(stop_time_info.time_duration_since_stop(), 0.0,
                 kStopTimeInfoClamp) /
      kStopTimeInfoClamp);
  stop_time_info_feats.push_back(
      std::clamp(stop_time_info.last_move_time_duration(), 0.0,
                 kStopTimeInfoClamp) /
      kStopTimeInfoClamp);
  stop_time_info_feats.push_back(
      std::clamp(stop_time_info.previous_stop_time_duration(), 0.0,
                 kStopTimeInfoClamp) /
      kStopTimeInfoClamp);

  const auto& av_pos =
      obj_sampler.GetResampledMotionHistoryById(kAvObjectId).states.back().pos;
  auto info_or =
      ExtractActNetCutinInfo(drive_passage_ptr, av_pos, agent_coord_transform);
  auto actnet_feature = ActNetFeature{
      .agent_id = agent_id,
      .ref_position = std::move(ref_position),
      .rot_rad = static_cast<float>(rot_rad),
      .stop_time_info = std::move(stop_time_info_feats),
      .agent_feature = std::move(agent_feat),
      .context_obj_features = std::move(context_obj_features),
      .context_obj_masks = std::move(context_obj_masks),
      .lc_features = std::move(lc_feats),
      .solid_lb_features = std::move(solid_lb_feats),
      .cw_features = std::move(cw_feats),
      .lc_masks = std::move(lc_masks),
      .solid_lb_masks = std::move(solid_lb_masks),
      .cw_masks = std::move(cw_masks),
  };

  if (info_or.has_value()) {
    actnet_feature.cutin_info = std::move(info_or).value();
  }

  if (!av_map_cache.broken_lane_boundarys.empty()) {
    const auto broken_lbs =
        map_sampler.GetNearestLaneBoundaryPolylinesInBox2dWithLaneBoundaries(
            region_box, kActNetConfig.max_broken_lb_num,
            av_map_cache.broken_lane_boundarys);
    auto broken_lb_feats = ExtractMapFeatureFromPolylines(
        broken_lbs, region_box, agent_coord_transform,
        kActNetConfig.max_lane_seg_num, kActNetConfig.max_broken_lb_num);
    std::vector<float> broken_lb_masks(kActNetConfig.max_broken_lb_num, 0.0);
    for (int i = 0; i < broken_lb_feats.size(); ++i) {
      broken_lb_masks[i] = 1.0;
    }

    actnet_feature.broken_lb_features = std::move(broken_lb_feats);
    actnet_feature.broken_lb_masks = std::move(broken_lb_masks);
  }
  return actnet_feature;
}

ActNetDumpedFeatureProto ToActNetDumpedFeatureProto(
    const ActNetFeature& act_feature, double ts, const Vec2d& ref_pos,
    double rot_rad, const Vec2d& av_pos, double av_angle) {
  ActNetDumpedFeatureProto feature_proto;
  feature_proto.set_agent_id(act_feature.agent_id);
  feature_proto.set_timestamp(ts);

  Vec2dToProto(ref_pos, feature_proto.mutable_ref_pos());
  Vec2dToProto(av_pos, feature_proto.mutable_av_pos());

  feature_proto.set_rot_angle(rot_rad);
  feature_proto.set_av_angle(av_angle);

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
  std::vector<const std::vector<ActNetPolylineFeature>*> poly_features = {
      &act_feature.lc_features, &act_feature.solid_lb_features,
      &act_feature.cw_features};

  if (!act_feature.broken_lb_features.empty()) {
    poly_features.push_back(&act_feature.broken_lb_features);
    *feature_proto.mutable_broken_lbs_mask() = {
        act_feature.broken_lb_masks.begin(), act_feature.broken_lb_masks.end()};
  }

  auto* mutable_lc_feature = feature_proto.mutable_lane_centers_feature();
  auto* mutable_lb_feature = feature_proto.mutable_lane_boundaries_feature();
  auto* mutable_cw_feature = feature_proto.mutable_crosswalks_feature();
  auto* mutable_broken_lb_feature = feature_proto.mutable_broken_lbs_feature();

  std::vector<ActNetDumpedFeatureProto::MapDumpedFeature*> feature_protos = {
      mutable_lc_feature, mutable_lb_feature, mutable_cw_feature,
      mutable_broken_lb_feature};
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
      mutable_feat.mutable_boundary_type()->Add(feat.boundary_type.begin(),
                                                feat.boundary_type.end());
      mutable_feat.mutable_seg_width()->Add(feat.seg_width.begin(),
                                            feat.seg_width.end());
      mutable_feat.mutable_lb_seg()->Add(feat.lb_seg.begin(),
                                         feat.lb_seg.end());
    }
  }

  *feature_proto.mutable_lcs_mask() = {act_feature.lc_masks.begin(),
                                       act_feature.lc_masks.end()};
  *feature_proto.mutable_lbs_mask() = {act_feature.solid_lb_masks.begin(),
                                       act_feature.solid_lb_masks.end()};

  *feature_proto.mutable_cws_mask() = {act_feature.cw_masks.begin(),
                                       act_feature.cw_masks.end()};

  auto* cutin_info = feature_proto.mutable_cutin_info();
  if (act_feature.cutin_info.path_xy.empty()) {
    cutin_info->set_has_path(false);
  } else {
    cutin_info->set_has_path(true);
    *(cutin_info->mutable_path_xy()) = {act_feature.cutin_info.path_xy.begin(),
                                        act_feature.cutin_info.path_xy.end()};
    *(cutin_info->mutable_headings()) = {
        act_feature.cutin_info.headings.begin(),
        act_feature.cutin_info.headings.end()};
    *(cutin_info->mutable_dist()) = {act_feature.cutin_info.dist.begin(),
                                     act_feature.cutin_info.dist.end()};
    *(cutin_info->mutable_unit_dist_vec()) = {
        act_feature.cutin_info.unit_dist_vec.begin(),
        act_feature.cutin_info.unit_dist_vec.end()};
  }
  cutin_info->set_already_on_path(act_feature.cutin_info.already_on_path);
  return feature_proto;
}

ActNetObjectAbsoluteFeature GetActNetObjectAbsoluteFeature(
    const ObjectMotionHistory& obj_history,
    const AgentCoordTransform& agent_coord_transform, int coord_num,
    double time_step, int max_steps) {
  const auto& orig = agent_coord_transform.orig;
  const double rot_rad = agent_coord_transform.rot_rad;
  const double rot_cos = agent_coord_transform.rot_cos;
  const double rot_sin = agent_coord_transform.rot_sin;
  ActNetObjectAbsoluteFeature feat;
  feat.type = static_cast<float>(ObjectTypeToActNetType(obj_history.type));
  const auto& last_state = obj_history.states.back();
  const double len = last_state.bbox.length();
  const double width = last_state.bbox.width();
  feat.lw = {static_cast<float>(len), static_cast<float>(width)};
  const auto state_num = obj_history.states.size();
  const int start_idx = max_steps - state_num;
  std::vector<float> pos_vec, speed_vec, yaw_vec, shape_vec, ts_vec, mask_vec;
  pos_vec.reserve(max_steps * coord_num);
  speed_vec.reserve(max_steps * coord_num);
  yaw_vec.reserve(max_steps * coord_num);
  shape_vec.reserve(max_steps * coord_num * kBoxCornerNum);
  ts_vec.reserve(max_steps);
  mask_vec.reserve(max_steps);
  const double hist_time_len = time_step * (max_steps - 1);
  for (int i = 0; i < max_steps; ++i) {
    ts_vec.push_back(i * time_step - hist_time_len);
    if (i < start_idx) {
      const auto empty_vec2d = {0.0, 0.0};
      std::copy(empty_vec2d.begin(), empty_vec2d.end(),
                std::back_inserter(pos_vec));
      std::copy(empty_vec2d.begin(), empty_vec2d.end(),
                std::back_inserter(speed_vec));
      std::copy(empty_vec2d.begin(), empty_vec2d.end(),
                std::back_inserter(yaw_vec));
      for (int j = 0; j < kBoxCornerNum; ++j) {
        std::copy(empty_vec2d.begin(), empty_vec2d.end(),
                  std::back_inserter(shape_vec));
      }
      mask_vec.push_back(0.0);
      continue;
    }
    const auto& state = obj_history.states[i - start_idx];
    const auto pos = (state.pos - orig).Rotate(rot_cos, rot_sin);
    const auto vel = state.vel.Rotate(rot_cos, rot_sin);
    const auto yaw = state.heading + rot_rad;
    const Box2d bbox(pos, yaw, len, width);
    pos_vec.push_back(pos.x());
    pos_vec.push_back(pos.y());
    speed_vec.push_back(vel.x());
    speed_vec.push_back(vel.y());
    yaw_vec.push_back(fast_math::Cos(yaw));
    yaw_vec.push_back(fast_math::Sin(yaw));
    for (const auto& pt : bbox.GetCornersCounterClockwise()) {
      shape_vec.push_back(pt.x());
      shape_vec.push_back(pt.y());
    }
    mask_vec.push_back(1.0);
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
  feat.yaw = std::move(yaw_vec);
  feat.shape = std::move(shape_vec);
  feat.ts = std::move(ts_vec);
  feat.mask = std::move(mask_vec);
  return feat;
}

ActNetObjectRelFeature GetActNetObjectRelFeature(
    const ActNetObjectAbsoluteFeature& agent_feat,
    const ActNetObjectAbsoluteFeature& obj_feat, int coord_num, int max_steps) {
  std::vector<float> rel_pos_vec, rel_dist_vec, rel_yaw_vec, yaw_diff_vec,
      rel_speed_vec, rel_shape_vec, rel_mask_vec;
  rel_pos_vec.reserve(max_steps * coord_num);
  rel_dist_vec.reserve(max_steps);
  rel_yaw_vec.reserve(max_steps * coord_num);
  yaw_diff_vec.reserve(max_steps * coord_num);
  rel_speed_vec.reserve(max_steps * coord_num);
  rel_shape_vec.reserve(max_steps * coord_num * kBoxCornerNum);
  rel_mask_vec.reserve(max_steps);

  for (int i = 0; i < max_steps; ++i) {
    if (static_cast<int>(agent_feat.mask[i]) == 0 ||
        static_cast<int>(obj_feat.mask[i]) == 0) {
      // no info at this place.
      const auto empty_vec2d = {0.0, 0.0};
      std::copy(empty_vec2d.begin(), empty_vec2d.end(),
                std::back_inserter(rel_pos_vec));
      rel_dist_vec.push_back(0.0);
      std::copy(empty_vec2d.begin(), empty_vec2d.end(),
                std::back_inserter(rel_yaw_vec));
      std::copy(empty_vec2d.begin(), empty_vec2d.end(),
                std::back_inserter(yaw_diff_vec));
      std::copy(empty_vec2d.begin(), empty_vec2d.end(),
                std::back_inserter(rel_speed_vec));
      for (int j = 0; j < kBoxCornerNum; ++j) {
        std::copy(empty_vec2d.begin(), empty_vec2d.end(),
                  std::back_inserter(rel_shape_vec));
      }
      rel_mask_vec.push_back(0.0);
      continue;
    }

    const Vec2d agent_pos(agent_feat.pos[i * coord_num],
                          agent_feat.pos[i * coord_num + 1]);
    const Vec2d obj_pos(obj_feat.pos[i * coord_num],
                        obj_feat.pos[i * coord_num + 1]);
    const Vec2d rel_pos = obj_pos - agent_pos;
    rel_pos_vec.push_back(rel_pos.x());
    rel_pos_vec.push_back(rel_pos.y());
    const double rel_dist = rel_pos.norm();
    rel_dist_vec.push_back(rel_dist);
    const Vec2d rel_yaw = rel_pos / std::max(rel_dist, kEpsilonS);
    rel_yaw_vec.push_back(rel_yaw.x());
    rel_yaw_vec.push_back(rel_yaw.y());

    const Vec2d agent_tangent(agent_feat.yaw[i * coord_num],
                              agent_feat.yaw[i * coord_num + 1]);
    const Vec2d obj_tangent(obj_feat.yaw[i * coord_num],
                            obj_feat.yaw[i * coord_num + 1]);
    const Vec2d yaw_diff =
        Vec2d::UnitFromAngle(obj_tangent.Angle() - agent_tangent.Angle());
    yaw_diff_vec.push_back(yaw_diff.x());
    yaw_diff_vec.push_back(yaw_diff.y());

    const Vec2d agent_speed(agent_feat.speed[i * coord_num],
                            agent_feat.speed[i * coord_num + 1]);
    const Vec2d obj_speed(obj_feat.speed[i * coord_num],
                          obj_feat.speed[i * coord_num + 1]);
    const Vec2d speed_diff = obj_speed - agent_speed;
    rel_speed_vec.push_back(speed_diff.x());
    rel_speed_vec.push_back(speed_diff.y());

    for (int j = 0; j < kBoxCornerNum; ++j) {
      const Vec2d corner_pt(
          obj_feat.shape[i * coord_num * kBoxCornerNum + j * coord_num],
          obj_feat.shape[i * coord_num * kBoxCornerNum + j * coord_num + 1]);
      const Vec2d rel_pt = corner_pt - agent_pos;
      rel_shape_vec.push_back(rel_pt.x());
      rel_shape_vec.push_back(rel_pt.y());
    }
    rel_mask_vec.push_back(1.0);
  }
  return ActNetObjectRelFeature{
      .rel_pos = std::move(rel_pos_vec),
      .rel_dist = std::move(rel_dist_vec),
      .rel_yaw = std::move(rel_yaw_vec),
      .yaw_diff = std::move(yaw_diff_vec),
      .rel_speed = std::move(rel_speed_vec),
      .rel_shape = std::move(rel_shape_vec),
      .rel_mask = std::move(rel_mask_vec),
  };
}

}  // namespace prediction
}  // namespace qcraft
