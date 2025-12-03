#include "onboard/prediction/net/horizon/act_net_j5_inferencer.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type
#include <algorithm>  // for clamp, max, min, sort
#include <cstddef>
#include <limits>
#include <unordered_map>  // for unordered_map, unordered_map<>::mapped_type
#include <utility>        // for pair, move

#include "onboard/async/parallel_for.h"  // for ParallelFor
#include "onboard/global/buffered_logger.h"
#include "onboard/global/trace.h"  // for ScopedTrace, SCOPED_QTRACE, SCOPED_QTRACE...
#include "onboard/lite/check.h"  // for QCHECK_GT
#include "onboard/math/fast_math.h"
#include "onboard/math/geometry/box2d.h"      // for Box2d
#include "onboard/math/geometry/segment2d.h"  // for Segment2d
#include "onboard/math/util.h"                // for Sigmoid, SoftMax
#include "onboard/prediction/feature_extractor/act_net_feature.h"  // for ObjectTypeToActNetType
#include "onboard/prediction/net/horizon/act_net_j5.h"  // for kCoordScale, kHistoryNum, kHistoryStepLen
#include "onboard/prediction/net/horizon/horizon_tensor_wrapper.h"  // for HorizonTensorWrapper
#include "onboard/prediction/prediction_defs.h"
#include "onboard/prediction/prediction_util.h"

namespace qcraft {
namespace prediction {
namespace actnetj5 {
namespace {
constexpr double kProbThreshold = 0.15;
constexpr double kEpsilonS = 1e-1;
constexpr double kAvMapRadius = 210.0;
// TODO(yinbao): duplicated with
// onboard/prediction/feature_extractor/act_net_feature_extractor.cc
using SampledPolylineSegRefs = std::vector<const SampledPolylineSeg*>;

std::vector<SampledPolylineSegRefs> BreakAndFilterPolylines(
    absl::Span<const SampledPolyline* const> polylines, const Box2d& region_box,
    int max_lane_seg_num) {
  QCHECK_GT(max_lane_seg_num, 0);
  std::vector<SampledPolylineSegRefs> breaked_polylines;
  breaked_polylines.reserve(polylines.size() * 2);
  for (const auto* poly_ptr : polylines) {
    const auto& polyline = *poly_ptr;
    const auto& segs = polyline.segs;
    const int segs_size = segs.size();
    if (segs_size == 0) {
      continue;
    }
    int num_breaks = (segs_size + max_lane_seg_num - 1) / max_lane_seg_num;
    // LOG(INFO) << "num_breaks: " << num_breaks;
    int j = 0;
    for (int i = 0; i < num_breaks; ++i) {
      SampledPolylineSegRefs breaked_poly_segments;
      breaked_poly_segments.reserve(max_lane_seg_num);
      bool is_in_box = false;
      for (; j < std::min((i + 1) * max_lane_seg_num, segs_size); ++j) {
        if (!is_in_box && region_box.IsPointIn(segs[j].seg.start())) {
          is_in_box = true;
        }
        breaked_poly_segments.push_back(&segs[j]);
      }
      if (!is_in_box) continue;
      breaked_polylines.push_back(std::move(breaked_poly_segments));
    }
  }
  return breaked_polylines;
}

void SortPolysByDist(
    const Vec2d& origin, const int max_feat_num,
    std::vector<const SampledPolylineSegRefs*>* filtered_polys) {
  if (filtered_polys->size() > max_feat_num) {
    std::vector<std::pair<const SampledPolylineSegRefs*, float>> polys_dist;
    polys_dist.reserve(filtered_polys->size());
    for (const auto* poly : *filtered_polys) {
      double dist = std::numeric_limits<double>::max();
      for (const auto& seg : *poly) {
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

void ExtractLaneFeatureByFilteredPolys(
    const int batch_idx, const double rot_rad, const Vec2d& origin,
    absl::Span<const SampledPolylineSegRefs* const> filtered_polys,
    HorizonTensorWrapper* line_pos, HorizonTensorWrapper* line_seg_info,
    HorizonTensorWrapper* line_attr_info) {
  double rot_cos_sin[2];
  fast_math::CosAndSin(rot_rad, rot_cos_sin);
  auto size = filtered_polys.size();
#if (defined(__ARM_NEON) && defined(__J5__))
  auto originx4 = vld1q_f64(&origin.x());
  double cos_sin_d_[3] = {rot_cos_sin[1], rot_cos_sin[0], -rot_cos_sin[1]};
  auto sin_cos = vld1q_f64(cos_sin_d_);
  auto cos_sin = vld1q_f64(cos_sin_d_ + 1);
#else
  Vec2Rotater vec2_rotater;
  vec2_rotater.set_cos_sin(rot_cos_sin[0], rot_cos_sin[1]);
#endif
  for (int poly_idx = 0; poly_idx < size; ++poly_idx) {
    const auto& segs = *filtered_polys[poly_idx];
    auto segs_size = segs.size();

    for (int i = 0; i < segs_size; ++i) {
      const auto& cur = segs[i]->seg.start();
      const auto& next = segs[i]->seg.end();
#if (defined(__ARM_NEON) && defined(__J5__))
      auto curx4 = vld1q_f64(&cur.x());
      auto nextx4 = vld1q_f64(&next.x());
      auto cp = vsubq_f64(curx4, originx4);
      auto np = vsubq_f64(nextx4, originx4);
      Vec2f cur_pos = {static_cast<float>(vaddvq_f64(vmulq_f64(cp, cos_sin))),
                       static_cast<float>(vaddvq_f64(vmulq_f64(cp, sin_cos)))};
      Vec2f next_pos = {static_cast<float>(vaddvq_f64(vmulq_f64(np, cos_sin))),
                        static_cast<float>(vaddvq_f64(vmulq_f64(np, sin_cos)))};
      float value[2] = {cur_pos.x() * kInvCoordScale,
                        kInvCoordScale * cur_pos.y()};
      line_pos->index_put_qat_neon64(batch_idx, 0, i, poly_idx, value);
      auto seg_vec = vsub_f32(vld1_f32(&next_pos.x()), vld1_f32(&cur_pos.x()));
      np = vld1q_f64(&segs[i]->unit_tangent.x());
      Vec2f unit_tangent = {
          static_cast<float>(vaddvq_f64(vmulq_f64(np, cos_sin))),
          static_cast<float>(vaddvq_f64(vmulq_f64(np, sin_cos)))};
      auto seg_vec_len = std::sqrt(vaddv_f32(vmul_f32(seg_vec, seg_vec)));
      float32x2_t nearest_pt;
      float dist;
      float nearest_to_end_dist;
      auto cur_posx4 = vld1_f32(&cur_pos.x());
      if (seg_vec_len <= kEpsilon) {
        dist = std::sqrt(vaddv_f32(vmul_f32(cur_posx4, cur_posx4)));
        nearest_to_end_dist = seg_vec_len;
        nearest_pt = vdiv_f32(cur_posx4, vdup_n_f32(dist));
      } else {
        const float proj =
            -vaddv_f32(vmul_f32(cur_posx4, seg_vec)) / seg_vec_len;
        if (proj < 0.0) {
          dist = std::sqrt(vaddv_f32(vmul_f32(cur_posx4, cur_posx4)));
          nearest_to_end_dist = seg_vec_len;
          nearest_pt = vdiv_f32(cur_posx4, vdup_n_f32(dist));
        } else if (proj > seg_vec_len) {
          cur_posx4 = vld1_f32(&next_pos.x());
          dist = std::sqrt(vaddv_f32(vmul_f32(cur_posx4, cur_posx4)));
          nearest_to_end_dist = 0;
          nearest_pt = vdiv_f32(cur_posx4, vdup_n_f32(dist));
        } else {
          nearest_to_end_dist = seg_vec_len - proj;
          auto ss = proj / seg_vec_len;
          dist = std::abs(cur_pos.x() * vget_lane_f32(seg_vec, 1) -
                          cur_pos.y() * vget_lane_f32(seg_vec, 0)) /
                 seg_vec_len;
          nearest_pt = vmla_f32(cur_posx4, seg_vec, vdup_n_f32(ss));
          nearest_pt = vdiv_f32(nearest_pt, vdup_n_f32(dist));
        }
      }
      float value3[8] = {
          seg_vec_len * kInvMaxSegLen,
          unit_tangent.x(),
          unit_tangent.y(),
          dist * kInvCoordScale,
          vget_lane_f32(nearest_pt, 0),
          vget_lane_f32(nearest_pt, 1),
          nearest_to_end_dist * kInvMaxSegLen,
          std::clamp(segs[i]->speed_limit_kph * kInvSpeedLimitScale, 0.0f,
                     1.0f)};
      line_seg_info->index_put_qat_neon128(batch_idx, 0, i, poly_idx, value3);
      line_seg_info->index_put_qat_neon128(batch_idx, 4, i, poly_idx,
                                           value3 + 4);
#else
      const auto cur_pos = vec2_rotater.SubAndRotate(cur, origin);
      const auto next_pos = vec2_rotater.SubAndRotate(next, origin);
      float values1[2] = {static_cast<float>(cur_pos.x() * kInvCoordScale),
                          static_cast<float>(cur_pos.y() * kInvCoordScale)};
      line_pos->index_put_qat(batch_idx, 0, i, poly_idx, values1[0]);
      line_pos->index_put_qat(batch_idx, 1, i, poly_idx, values1[1]);
      // line_seg_info batch,8,5,160
      // (seg_len, unit_tangent, dist, nearest_to_end_dist, speed_limits)
      // seg_len
      const auto seg_vec = next_pos - cur_pos;
      const auto unit_tangent = vec2_rotater.Rotate(segs[i]->unit_tangent);
      Vec2d nearest_pt;
      const Segment2d rotated_seg(cur_pos, next_pos);
      const double dist = rotated_seg.DistanceTo(Vec2d(0.0, 0.0), &nearest_pt);
      Vec2d unit_dist_vec(0.0, 0.0);
      if (nearest_pt != Vec2d(0.0, 0.0)) unit_dist_vec = nearest_pt.Unit();

      const double nearest_to_end_dist = (nearest_pt - next_pos).norm();
      float values2[8] = {
          static_cast<float>(seg_vec.norm() * kInvMaxSegLen),
          static_cast<float>(unit_tangent.x()),
          static_cast<float>(unit_tangent.y()),
          static_cast<float>(dist * kInvCoordScale),
          static_cast<float>(unit_dist_vec.x()),
          static_cast<float>(unit_dist_vec.y()),
          static_cast<float>(nearest_to_end_dist * kInvMaxSegLen),
          std::clamp(segs[i]->speed_limit_kph * kInvSpeedLimitScale, 0.0f,
                     1.0f)};
      line_seg_info->index_put_qat(batch_idx, 0, i, poly_idx, values2[0]);
      line_seg_info->index_put_qat(batch_idx, 1, i, poly_idx, values2[1]);
      line_seg_info->index_put_qat(batch_idx, 2, i, poly_idx, values2[2]);
      line_seg_info->index_put_qat(batch_idx, 3, i, poly_idx, values2[3]);
      line_seg_info->index_put_qat(batch_idx, 4, i, poly_idx, values2[4]);
      line_seg_info->index_put_qat(batch_idx, 5, i, poly_idx, values2[5]);
      line_seg_info->index_put_qat(batch_idx, 6, i, poly_idx, values2[6]);
      line_seg_info->index_put_qat(batch_idx, 7, i, poly_idx, values2[7]);
#endif
    }
    // light
#if (defined(__ARM_NEON) && defined(__J5__))
    line_attr_info->index_put_qat_neon128(
        batch_idx, 0, 0, poly_idx,
        reinterpret_cast<const float*>(segs.back()->lights.data()));

#else
    for (int k = 0; k < kLaneLightDim; ++k) {
      line_attr_info->index_put_qat(batch_idx, k, 0, poly_idx,
                                    segs.back()->lights[k]);
    }
#endif
    // line_type
    int line_type = static_cast<int>(segs.back()->type);
    // Remap kBrokenxx to kNone to avoid index out of boundary.
    if (line_type > kLaneTypeDim - 1) line_type = 0;
    line_attr_info->index_put_qat(batch_idx, line_type + kLaneLightDim, 0,
                                  poly_idx, 1.0f);
  }
}

__attribute__((always_inline)) inline void ExtractLaneFeature(
    const int batch_idx, const double rot_rad, const Vec2d& origin,
    const Box2d& region_box, absl::Span<const SampledPolyline* const> polylines,
    const int max_feat_num, const int max_lane_seg_num,
    HorizonTensorWrapper* line_pos, HorizonTensorWrapper* line_seg_info,
    HorizonTensorWrapper* line_attr_info) {
  // 1. break and filter Plylines
  const auto filtered_polys =
      BreakAndFilterPolylines(polylines, region_box, max_lane_seg_num);
  std::vector<const SampledPolylineSegRefs*> filtered_poly_ptrs;
  filtered_poly_ptrs.reserve(filtered_polys.size());
  for (int i = 0; i < filtered_polys.size(); ++i) {
    filtered_poly_ptrs.push_back(&filtered_polys[i]);
  }
  // sorted by dist
  SortPolysByDist(origin, max_feat_num, &filtered_poly_ptrs);
  // 2. extract feature
  ExtractLaneFeatureByFilteredPolys(batch_idx, rot_rad, origin,
                                    filtered_poly_ptrs, line_pos, line_seg_info,
                                    line_attr_info);
}

}  // namespace

ActNetJ5Inferencer::ActNetJ5Inferencer(const NetParam& net_param)
    : net_param_(net_param) {
  act_net_j5_ = std::make_unique<PredictionJ5QNN>(net_param_);
}

// GetInput from sampler.
int ActNetJ5Inferencer::GenBatchInputs(
    absl::Span<const ObjectHistory* const> objects_history,
    const ObjectHistorySampler& obj_sampler, MapSampler* map_sampler_ptr,
    std::vector<AgentRef>& agent_origins) const {
  SCOPED_QTRACE_ARG1("PrepareActNetJ5Inputs",
                     "objs_num:", objects_history.size());
  act_net_j5_->ResetInputs();
  const std::unordered_map<std::string, std::shared_ptr<HorizonTensorWrapper>>&
      batch_map = act_net_j5_->GetInputMap();
  agent_origins.resize(objects_history.size());

  auto& actor_info_hist = *batch_map.at("actor_info_hist");
  auto& actor_info_attr = *batch_map.at("actor_info_attr");
  auto& actor_ctrs = *batch_map.at("actor_ctrs");
  auto& lc_pos = *batch_map.at("lc_pos");
  auto& lc_seg_info = *batch_map.at("lc_seg_info");
  auto& lc_attr_info = *batch_map.at("lc_attr_info");
  auto& lb_pos = *batch_map.at("lb_pos");
  auto& lb_seg_info = *batch_map.at("lb_seg_info");
  auto& lb_attr_info = *batch_map.at("lb_attr_info");
  Vec2Rotater vec2_rotater;

  const auto* av_history = obj_sampler.GetResampledAVMotionHistory();
  const auto& av_pos = av_history->states.back().pos;
  const auto lane_centers =
      map_sampler_ptr->GetLaneCentersWithRadius(av_pos, kAvMapRadius);
  const auto lane_boundaries =
      map_sampler_ptr->GetSolidLaneBoundariesWithRadius(av_pos, kAvMapRadius);
  for (size_t batch_idx = 0; batch_idx < objects_history.size(); ++batch_idx) {
    const auto& agent_hist = *objects_history[batch_idx];
    const auto& agent_id = agent_hist.id();
    const auto& agent_history =
        obj_sampler.GetResampledMotionHistoryById(agent_id);
    const double heading = agent_history.states.back().heading;
    const auto& origin = agent_history.states.back().pos;
    const double rot_rad = NormalizeAngle(-heading);
    const auto region_box = GetDynamicRegionBox(origin, agent_history, heading);

    agent_origins[batch_idx].ref_position = origin;
    agent_origins[batch_idx].rot_rad = rot_rad;
    agent_origins[batch_idx].agent_id = agent_id;
    double rot_cos_sin[2];
    fast_math::CosSinNormalized(rot_rad, rot_cos_sin);
    vec2_rotater.set_cos_sin(rot_cos_sin[0], rot_cos_sin[1]);
    // agent
    {
      // actor_info_hist (batch,11,10,32)
      // (obj_pos_diff, obj_speed, obj_heading, rel_dist, rel_yaw, rel_speed)
      const int state_num = agent_history.states.size();
      const int start_idx = kHistoryNum - state_num;
      const int start = std::max(0, start_idx);
      float val[8];
      for (int hist_idx = start; hist_idx < kHistoryNum; ++hist_idx) {
        const auto& state = agent_history.states[hist_idx - start_idx];
        // const auto pos = vec2_rotater.SubAndRotate(state.pos , origin);
        const auto vel = vec2_rotater.Rotate(state.vel);
        const auto yaw = state.heading + rot_rad;

        // pos_diff
        if (hist_idx == start_idx) {
          val[0] = vel.x() * kHistoryStepLen;
          val[1] = vel.y() * kHistoryStepLen;
        } else {
          auto last_pos = vec2_rotater.SubAndRotate(
              state.pos, agent_history.states[hist_idx - start_idx - 1].pos);
          val[0] = last_pos.x();
          val[1] = last_pos.y();
        }
        val[2] = vel.x() * kInvSpeedScale;
        val[3] = vel.y() * kInvSpeedScale;
        double yaw_cos_sin[2];
        fast_math::CosSinNormalized(yaw, yaw_cos_sin);
        val[4] = yaw_cos_sin[0];
        val[5] = yaw_cos_sin[1];
#if (defined(__ARM_NEON) && defined(__J5__))
        actor_info_hist.index_put_qat_neon128(batch_idx, 0, hist_idx, 0, val);
        actor_info_hist.index_put_qat_neon64(batch_idx, 4, hist_idx, 0,
                                             val + 4);
#else
        actor_info_hist.index_put_qat(batch_idx, 0, hist_idx, 0, val[0]);
        actor_info_hist.index_put_qat(batch_idx, 1, hist_idx, 0, val[1]);
        actor_info_hist.index_put_qat(batch_idx, 2, hist_idx, 0, val[2]);
        actor_info_hist.index_put_qat(batch_idx, 3, hist_idx, 0, val[3]);
        actor_info_hist.index_put_qat(batch_idx, 4, hist_idx, 0, val[4]);
        actor_info_hist.index_put_qat(batch_idx, 5, hist_idx, 0, val[5]);
#endif
      }
      // actor_info_attr batch,18,1,32 (agent_type, agent_lw)
      const auto& last_state = agent_history.states.back();
      // agent_type onehot
      const int type =
          static_cast<int>(ObjectTypeToActNetType(agent_history.type));
      actor_info_attr.index_put_qat(batch_idx, type, 0, 0, 1.0f);
      // agent_lw
      actor_info_attr.index_put_qat(
          batch_idx, 16, 0, 0,
          std::clamp(static_cast<float>(last_state.bbox.length()) *
                         kInvObjectLengthScale,
                     0.0f, 1.0f));
      actor_info_attr.index_put_qat(
          batch_idx, 17, 0, 0,
          std::clamp(static_cast<float>(last_state.bbox.width()) *
                         kInvObjectWidthScale,
                     0.0f, 1.0f));
      // actor_ctrs batch,2,1,32 (ctrs)
      const auto ctrs = vec2_rotater.SubAndRotate(last_state.pos, origin);
      //    (last_state.pos - origin).Rotate(rot_cos_sin[0], rot_cos_sin[1]);
      actor_ctrs.index_put_qat(batch_idx, 0, 0, 0, ctrs.x() * kInvCoordScale);
      actor_ctrs.index_put_qat(batch_idx, 1, 0, 0, ctrs.y() * kInvCoordScale);
    }
    // other actors
    {
      const auto objects_in_box2d =
          obj_sampler.GetResampledMotionHistoryWithAVInBox2d(
              agent_id, region_box, kMaxOtherObjsNum);
      float val[12];
      for (int obj_idx = 0; obj_idx < objects_in_box2d.size(); ++obj_idx) {
        const auto& obj_history = *objects_in_box2d[obj_idx];
        // actor_info_hist (batch,11,10,32)
        // (obj_pos_diff, obj_speed, obj_heading, rel_dist, rel_yaw, rel_speed)
        const int state_num = obj_history.states.size();
        const int start_idx = kHistoryNum - state_num;
        const int agent_start_idx = kHistoryNum - agent_history.states.size();
        const int start = std::max(0, start_idx);

        for (int hist_idx = start; hist_idx < kHistoryNum; ++hist_idx) {
          const auto& state = obj_history.states[hist_idx - start_idx];
          const auto vel = vec2_rotater.Rotate(state.vel);
          const auto yaw = state.heading + rot_rad;
          // pos_diff
          if (hist_idx == start_idx) {
            val[0] = vel.x() * kHistoryStepLen;
            val[1] = vel.y() * kHistoryStepLen;
          } else {
            auto last_pos = vec2_rotater.SubAndRotate(
                state.pos, obj_history.states[hist_idx - start_idx - 1].pos);
            val[0] = last_pos.x();
            val[1] = last_pos.y();
          }
          val[2] = vel.x() * kInvSpeedScale;
          val[3] = vel.y() * kInvSpeedScale;
          double yaw_cos_sin[2];
          fast_math::CosSinNormalized(yaw, yaw_cos_sin);
          val[4] = yaw_cos_sin[0];
          val[5] = yaw_cos_sin[1];
#if (defined(__ARM_NEON) && defined(__J5__))
          actor_info_hist.index_put_qat_neon128(batch_idx, 0, hist_idx,
                                                obj_idx + 1, val);
          actor_info_hist.index_put_qat_neon64(batch_idx, 4, hist_idx,
                                               obj_idx + 1, val + 4);
#else
          actor_info_hist.index_put_qat(batch_idx, 0, hist_idx, obj_idx + 1,
                                        val[0]);
          actor_info_hist.index_put_qat(batch_idx, 1, hist_idx, obj_idx + 1,
                                        val[1]);
          actor_info_hist.index_put_qat(batch_idx, 2, hist_idx, obj_idx + 1,
                                        val[2]);
          actor_info_hist.index_put_qat(batch_idx, 3, hist_idx, obj_idx + 1,
                                        val[3]);
          actor_info_hist.index_put_qat(batch_idx, 4, hist_idx, obj_idx + 1,
                                        val[4]);
          actor_info_hist.index_put_qat(batch_idx, 5, hist_idx, obj_idx + 1,
                                        val[5]);
#endif
          if (hist_idx - agent_start_idx >= 0) {
            const auto& agent_state =
                agent_history.states[hist_idx - agent_start_idx];
            auto rel_pos =
                vec2_rotater.SubAndRotate(state.pos, agent_state.pos);
            auto nm = rel_pos.norm();
            val[6] = nm * kInvCoordScale;
            const double rel_dist_inv = 1.0 / std::max(nm, kEpsilonS);
            val[7] = rel_pos.x() * rel_dist_inv;
            val[8] = rel_pos.y() * rel_dist_inv;
            const auto rel_speed =
                vec2_rotater.SubAndRotate(state.vel, agent_state.vel);
            val[9] = rel_speed.x() * kInvSpeedScale;
            val[10] = rel_speed.y() * kInvSpeedScale;
#if (defined(__ARM_NEON) && defined(__J5__))
            actor_info_hist.index_put_qat(batch_idx, 6, hist_idx, obj_idx + 1,
                                          val[6]);
            actor_info_hist.index_put_qat_neon128(batch_idx, 7, hist_idx,
                                                  obj_idx + 1, val + 7);
#else
            actor_info_hist.index_put_qat(batch_idx, 6, hist_idx, obj_idx + 1,
                                          val[6]);
            actor_info_hist.index_put_qat(batch_idx, 7, hist_idx, obj_idx + 1,
                                          val[7]);
            actor_info_hist.index_put_qat(batch_idx, 8, hist_idx, obj_idx + 1,
                                          val[8]);
            actor_info_hist.index_put_qat(batch_idx, 9, hist_idx, obj_idx + 1,
                                          val[9]);
            actor_info_hist.index_put_qat(batch_idx, 10, hist_idx, obj_idx + 1,
                                          val[10]);
#endif
          }
        }
        // actor_info_attr batch,18,1,32 (obj_type, obj_lw)
        const auto& last_state = obj_history.states.back();
        // obj_type onehot
        const int type =
            static_cast<int>(ObjectTypeToActNetType(obj_history.type));
        actor_info_attr.index_put_qat(batch_idx, type, 0, obj_idx + 1, 1.0f);
        // obj_lw
        actor_info_attr.index_put_qat(
            batch_idx, 16, 0, obj_idx + 1,
            std::clamp(static_cast<float>(last_state.bbox.length()) *
                           kInvObjectLengthScale,
                       0.0f, 1.0f));
        actor_info_attr.index_put_qat(
            batch_idx, 17, 0, obj_idx + 1,
            std::clamp(static_cast<float>(last_state.bbox.width()) *
                           kInvObjectWidthScale,
                       0.0f, 1.0f));
        // actor_ctrs batch,2,1,32 (ctrs)
        const auto ctrs = vec2_rotater.SubAndRotate(last_state.pos, origin);
        actor_ctrs.index_put_qat(batch_idx, 0, 0, obj_idx + 1,
                                 ctrs.x() * kInvCoordScale);
        actor_ctrs.index_put_qat(batch_idx, 1, 0, obj_idx + 1,
                                 ctrs.y() * kInvCoordScale);
      }
    }

    // lc
    // batch_inputs: lc_pos, lc_seg_info, lc_attr_info
    {
      const auto lcs =
          map_sampler_ptr->GetNearestLaneCenterPolylinesInBox2dWithLanes(
              region_box, kMaxLaneCenterNum, lane_centers,
              /*compute_boundary_distance=*/false);
      ExtractLaneFeature(batch_idx, rot_rad, origin, region_box, lcs,
                         kMaxLaneCenterNum, kLaneSegsNum, &lc_pos, &lc_seg_info,
                         &lc_attr_info);
    }

    // lb
    // batch_inputs: lb_pos, lb_seg_info, lb_attr_info
    {
      const auto lbs =
          map_sampler_ptr
              ->GetNearestLaneBoundaryPolylinesInBox2dWithLaneBoundaries(
                  region_box, kMaxLaneBoundaryNum, lane_boundaries);
      ExtractLaneFeature(batch_idx, rot_rad, origin, region_box, lbs,
                         kMaxLaneBoundaryNum, kLaneSegsNum, &lb_pos,
                         &lb_seg_info, &lb_attr_info);
    }
  }
  // TODO(yinbao): skip some agents without feature.
  return objects_history.size();
}

// Run ActNet model, and get the result.
AgentCentricObjectsOut ActNetJ5Inferencer::PredictForObjects(
    absl::Span<const ObjectHistory* const> objects_history,
    const ObjectHistorySampler& obj_sampler, MapSampler* map_sampler_ptr,
    ThreadPool* thread_pool) const {
  SCOPED_QTRACE("ActNetJ5inferencer::Prediction for Objects");

  AgentCentricObjectsOut objs_out;

  std::vector<AgentRef> agent_origins;
  int current_batch_size = 0;
  // 1. Feature to Batch inputs.
  {
    SCOPED_QTRACE("ActNetJ5inferencer::GenBatchInputs from obj sampler");
    current_batch_size = GenBatchInputs(objects_history, obj_sampler,
                                        map_sampler_ptr, agent_origins);
    if (current_batch_size == 0) {
      return objs_out;
    }
  }

  // 2. Run ActNet forward.
  {
    SCOPED_QTRACE("ActNetJ5inferencer::ProcessInEngine");
    act_net_j5_->Run(current_batch_size);
  }

  // 3. Set objects
  {
    SCOPED_QTRACE("ActNetJ5Inferencer::AssembleProbTrajs");
    objs_out = ExtractOutputs(current_batch_size, agent_origins, thread_pool);
  }
  return objs_out;
}

__attribute__((always_inline)) inline AgentCentricObjectsOut
ActNetJ5Inferencer::ExtractOutputs(int current_batch_size,
                                   const std::vector<AgentRef>& agent_origins,
                                   ThreadPool* thread_pool) const {
  const auto& batch_out_map = act_net_j5_->GetOutputMap();
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

}  // namespace actnetj5
}  // namespace prediction
}  // namespace qcraft
