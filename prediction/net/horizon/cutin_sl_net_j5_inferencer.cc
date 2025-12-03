#include "onboard/prediction/net/horizon/cutin_sl_net_j5_inferencer.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type
#include <algorithm>  // for clamp, max, min, sort
#include <cmath>
#include <limits>
#include <string>
#include <unordered_map>  // for unordered_map, unordered_map<>::mapped_type
#include <utility>        // for pair, move
#include <vector>

#include "absl/status/statusor.h"
#include "absl/types/span.h"

#include "onboard/global/buffered_logger.h"
#include "onboard/global/trace.h"  // for ScopedTrace, SCOPED_QTRACE, SCOPED_QTRACE...
#include "onboard/lite/check.h"  // for QCHECK_GT
#include "onboard/math/frenet_common.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/geometry/segment2d.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/math/util.h"  // for Sigmoid, SoftMax
#include "onboard/math/vec.h"
#include "onboard/prediction/feature_extractor/cutin_sl_feature.h"
#include "onboard/prediction/net/horizon/cutin_sl_net_j5.h"
#include "onboard/prediction/net/horizon/horizon_tensor_wrapper.h"  // for HorizonTensorWrapper
#include "onboard/prediction/prediction_defs.h"
#include "onboard/prediction/prediction_util.h"

namespace qcraft {
namespace prediction {
namespace cutin_sl_net_j5 {
namespace {
constexpr double kSLCorMax = 500;

struct AgentHistInfo {
  float s;
  float l;
  float speed_s;
  float speed_l;
};

using SampledPolylineSegRefs = std::vector<const SampledPolylineSeg*>;

inline std::vector<double> GetCutinScanBoxInfo(double speed) {
  return {kScanBoxFrontPlf(speed), kScanBoxBack,
          kScanBoxFrontHalfWidthPlf(speed)};
}

inline float ShiftAndScale(float value) { return (value - 0.5) * 2.0; }

std::vector<SampledPolylineSegRefs> BreakAndFilterPolylines(
    absl::Span<const SampledPolyline* const> polylines, const Box2d& region_box,
    int max_lane_seg_num) {
  SCOPED_QTRACE("BreakAndFilterPolylines");
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
    int num_breaks = 0;
    if (segs_size % max_lane_seg_num == 0) {
      num_breaks = segs_size / max_lane_seg_num;
    } else {
      num_breaks = segs_size / max_lane_seg_num + 1;
    }
    for (int i = 0; i < num_breaks; ++i) {
      SampledPolylineSegRefs breaked_poly_segments;
      breaked_poly_segments.reserve(max_lane_seg_num);
      bool is_in_box = false;
      for (int j = i * max_lane_seg_num;
           j < std::min((i + 1) * max_lane_seg_num, segs_size); ++j) {
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
  const int poly_num =
      std::min(max_feat_num, static_cast<int>(polys_dist.size()));
  for (int i = 0; i < poly_num; ++i) {
    filtered_polys->push_back(polys_dist[i].first);
  }
}

void ExtractLaneFeatureByFilteredPolys(
    const int batch_idx, const double rot_rad, const Vec2d& origin,
    absl::Span<const SampledPolylineSegRefs* const> filtered_polys,
    const int max_feat_num, const int max_lane_seg_num,
    const planner::DrivePassage& drive_passage, HorizonTensorWrapper* line_pos,
    HorizonTensorWrapper* line_seg_info, HorizonTensorWrapper* line_attr_info) {
  // init feature
  for (int i = 0; i < max_feat_num; ++i) {
    for (int j = 0; j < max_lane_seg_num; ++j) {
      line_seg_info->index_put_qat(batch_idx, 0, j, i, ShiftAndScale(0.0f));
      line_seg_info->index_put_qat(batch_idx, 3, j, i, ShiftAndScale(0.0f));
      line_seg_info->index_put_qat(batch_idx, 4, j, i, ShiftAndScale(0.0f));
    }
    for (int k = 0; k < kLaneLightDim; ++k) {
      line_attr_info->index_put_qat(batch_idx, k, 0, i, ShiftAndScale(0.0f));
    }
    for (int k = 0; k < 16; ++k) {
      line_attr_info->index_put_qat(batch_idx, k + kLaneLightDim, 0, i,
                                    ShiftAndScale(0.0f));
    }
  }
  for (int poly_idx = 0; poly_idx < filtered_polys.size(); ++poly_idx) {
    if (poly_idx >= max_feat_num) {
      break;
    }
    const auto& segs = *filtered_polys[poly_idx];
    const int segs_size =
        std::min(static_cast<int>(segs.size()), max_lane_seg_num);
    QCHECK_GT(segs_size, 0);

    for (int i = 0; i < segs_size; ++i) {
      const auto& cur = segs[i]->seg.start();
      const auto& next = segs[i]->seg.end();
      if (!std::isfinite(cur.x()) || !std::isfinite(cur.y())) {
        continue;
      }
      // Extract sl pos
      const auto cur_sl_pos =
          drive_passage.QueryUnboundedFrenetCoordinateAt(cur);
      if (!cur_sl_pos.ok() || std::abs(cur_sl_pos->s) > kSLCorMax ||
          std::abs(cur_sl_pos->l) > kSLCorMax) {
        continue;
      }
      const float cur_sl_pos_s = std::clamp(
          static_cast<float>(cur_sl_pos->s) * kInvLaneCoordScale, -1.0f, 1.0f);
      const float cur_sl_pos_l = std::clamp(
          static_cast<float>(cur_sl_pos->l) * kInvLaneCoordScale, -1.0f, 1.0f);
      line_pos->index_put_qat(batch_idx, 0, i, poly_idx, cur_sl_pos_s);
      line_pos->index_put_qat(batch_idx, 1, i, poly_idx, cur_sl_pos_l);
      // Extract seg length
      const auto seg_vec = next - cur;
      const float seg_len =
          std::clamp(ShiftAndScale(seg_vec.norm() * kInvLaneSegmentLengthScale),
                     -1.0f, 1.0f);
      line_seg_info->index_put_qat(batch_idx, 0, i, poly_idx, seg_len);
      // Extract yaw diff
      const auto yaw_diff = Vec2d::UnitFromAngle(seg_vec.Angle() + rot_rad);
      line_seg_info->index_put_qat(batch_idx, 1, i, poly_idx, yaw_diff.x());
      line_seg_info->index_put_qat(batch_idx, 2, i, poly_idx, yaw_diff.y());
      // Extract dist
      Vec2d nearest_pt;
      const Segment2d cur_seg(cur, next);
      const float dist =
          std::clamp(ShiftAndScale(cur_seg.DistanceTo(origin, &nearest_pt) *
                                   kInvLaneCoordScale),
                     -1.0f, 1.0f);
      line_seg_info->index_put_qat(batch_idx, 3, i, poly_idx, dist);
      // Extract nearest_to_end_dist
      const float nearest_to_end_dist =
          std::clamp(ShiftAndScale((nearest_pt - next).norm() *
                                   kInvLaneSegmentLengthScale),
                     -1.0f, 1.0f);
      line_seg_info->index_put_qat(batch_idx, 4, i, poly_idx,
                                   nearest_to_end_dist);
    }
    if (segs_size > 0) {
      for (int k = 0; k < kLaneLightDim; ++k) {
        const float light = std::clamp(
            ShiftAndScale(segs[0]->lights[k] * kInvLaneLightsNumScale), -1.0f,
            1.0f);
        line_attr_info->index_put_qat(batch_idx, k, 0, poly_idx, light);
      }
      int line_type = static_cast<int>(segs[0]->type);
      // Remap kBrokenxx to kNone to avoid index out of boundary.
      if (line_type > kLaneTypeDim - 1) line_type = 0;

      line_attr_info->index_put_qat(
          batch_idx, line_type + kLaneLightDim, 0, poly_idx,
          std::clamp(ShiftAndScale(1.0f), -1.0f, 1.0f));
    }
  }
}

void ExtractLaneFeature(const int batch_idx, const double rot_rad,
                        const Vec2d& origin, const Box2d& region_box,
                        absl::Span<const SampledPolyline* const> polylines,
                        const int max_feat_num, const int max_lane_seg_num,
                        const planner::DrivePassage& drive_passage,
                        HorizonTensorWrapper* line_pos,
                        HorizonTensorWrapper* line_seg_info,
                        HorizonTensorWrapper* line_attr_info) {
  SCOPED_QTRACE("ExtractLaneFeature");
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
  ExtractLaneFeatureByFilteredPolys(
      batch_idx, rot_rad, origin, filtered_poly_ptrs, max_feat_num,
      max_lane_seg_num, drive_passage, line_pos, line_seg_info, line_attr_info);
}

void ExtractActorFeature(const ObjectIDType& agent_id, int batch_idx,
                         int agent_hist_num, const Box2d& region_box,
                         const ObjectHistorySampler& obj_sampler,
                         const planner::DrivePassage& drive_passage,
                         const std::vector<AgentHistInfo>& agent_hist_info_vec,
                         HorizonTensorWrapper* obj_info_hist,
                         HorizonTensorWrapper* actor_info_attr,
                         HorizonTensorWrapper* actor_ctrs) {
  const auto& agent_history =
      obj_sampler.GetResampledMotionHistoryById(agent_id);
  const auto& agent_states = agent_history.states;
  int agent_state_num = agent_states.size();

  const auto objects_in_box2d =
      obj_sampler.GetResampledMotionHistoryWithAVInBox2d(
          agent_id, region_box, cutin_sl_net_j5::kMaxOtherObjsNum);
  // init feature
  for (int i = 0; i < cutin_sl_net_j5::kMaxOtherObjsNum; ++i) {
    for (int j = 0; j < kHistoryNum; ++j) {
      obj_info_hist->index_put_qat(batch_idx, 20, j, i, ShiftAndScale(0.0f));
    }
    for (int k = 0; k < 16; ++k) {
      actor_info_attr->index_put_qat(batch_idx, k, 0, i + 1,
                                     ShiftAndScale(0.0f));
    }
  }
  for (int i = 0, obj_idx = 0; i < objects_in_box2d.size(); ++i) {
    const auto& obj_history = *objects_in_box2d[i];
    // obj_info_hist agent
    const auto& last_state = obj_history.states.back();
    const double length = last_state.bbox.length();
    const double width = last_state.bbox.width();
    const int state_num = obj_history.states.size();
    const int hist_num = std::min(state_num, kHistoryNum);
    FrenetCoordinate last_sl_pos;
    bool obj_valid = true;
    {
      for (int j = 0; j < hist_num; ++j) {
        const auto& state = obj_history.states[state_num - hist_num + j];
        const int hist_idx = kHistoryNum - hist_num + j;
        const auto& pos = state.pos;
        // Extract sl pos
        const auto sl_pos_or =
            drive_passage.QueryUnboundedFrenetCoordinateAt(pos);
        if (!sl_pos_or.ok() || std::abs(sl_pos_or->s) > kSLCorMax ||
            std::abs(sl_pos_or->l) > kSLCorMax) {
          obj_valid = false;
          break;
        }
        // Extract yaw diff to drive passage
        const auto dp_angle_or =
            drive_passage.QueryTangentAngleAtS(sl_pos_or->s);
        if (!dp_angle_or.ok()) {
          obj_valid = false;
          break;
        }
        const auto yaw = state.heading;
        // Extract sl shape
        const Box2d bbox(pos, yaw, length, width);
        std::vector<FrenetCoordinate> sl_shape_pt_vec;
        sl_shape_pt_vec.reserve(4);
        for (const auto& pt : bbox.GetCornersCounterClockwise()) {
          const auto sl_shape_pt_or =
              drive_passage.QueryUnboundedFrenetCoordinateAt(pt);
          if (!sl_shape_pt_or.ok() || std::abs(sl_shape_pt_or->s) > kSLCorMax ||
              std::abs(sl_shape_pt_or->l) > kSLCorMax) {
            break;
          }
          sl_shape_pt_vec.push_back(*sl_shape_pt_or);
        }
        if (sl_shape_pt_vec.size() != 4) {
          obj_valid = false;
          break;
        }
        const float sl_pos_s =
            std::clamp(sl_pos_or->s * kInvActorSLPosSScale, -1.0, 1.0);
        const float sl_pos_l =
            std::clamp(sl_pos_or->l * kInvActorSLPosLScale, -1.0, 1.0);
        obj_info_hist->index_put_qat(batch_idx, 0, hist_idx, obj_idx, sl_pos_s);
        obj_info_hist->index_put_qat(batch_idx, 1, hist_idx, obj_idx, sl_pos_l);
        if (j > 0) {
          const float sl_speed_s = std::clamp(
              (sl_pos_or->s - last_sl_pos.s) * kInvActorSLSpeedSScale, -1.0,
              1.0);
          const float sl_speed_l = std::clamp(
              (sl_pos_or->l - last_sl_pos.l) * kInvActorSLSpeedLScale, -1.0,
              1.0);
          obj_info_hist->index_put_qat(batch_idx, 2, hist_idx, obj_idx,
                                       sl_speed_s);
          obj_info_hist->index_put_qat(batch_idx, 3, hist_idx, obj_idx,
                                       sl_speed_l);
          if (j == 1) {
            obj_info_hist->index_put_qat(batch_idx, 2, hist_idx - 1, obj_idx,
                                         sl_speed_s);
            obj_info_hist->index_put_qat(batch_idx, 3, hist_idx - 1, obj_idx,
                                         sl_speed_l);
          }
        }
        int pt_idx = 0;
        for (const auto& sl_shape_pt : sl_shape_pt_vec) {
          // 4-11
          const float sl_shape_s =
              std::clamp(sl_shape_pt.s * kInvActorSLShapeSScale, -1.0, 1.0);
          const float sl_shape_l =
              std::clamp(sl_shape_pt.l * kInvActorSLShapeLScale, -1.0, 1.0);
          obj_info_hist->index_put_qat(batch_idx, 4 + pt_idx * 2, hist_idx,
                                       obj_idx, sl_shape_s);
          obj_info_hist->index_put_qat(batch_idx, 4 + pt_idx * 2 + 1, hist_idx,
                                       obj_idx, sl_shape_l);
          ++pt_idx;
        }
        const Vec2d yaw_diff_dp = Vec2d::UnitFromAngle(*dp_angle_or - yaw);
        obj_info_hist->index_put_qat(batch_idx, 12, hist_idx, obj_idx,
                                     yaw_diff_dp.x());
        obj_info_hist->index_put_qat(batch_idx, 13, hist_idx, obj_idx,
                                     yaw_diff_dp.y());
        const int min_hist_num = std::min(agent_hist_num, hist_num);

        // Calculate relative features
        if (j >= hist_num - min_hist_num) {
          // rel_sl_pos
          const float rel_agent_s =
              std::clamp((sl_pos_or->s - agent_hist_info_vec.at(hist_idx).s) *
                             kInvActorSLPosSScale,
                         -1.0, 1.0);
          const float rel_agent_l =
              std::clamp((sl_pos_or->l - agent_hist_info_vec.at(hist_idx).l) *
                             kInvActorSLPosLScale,
                         -1.0, 1.0);
          obj_info_hist->index_put_qat(batch_idx, 14, hist_idx, obj_idx,
                                       rel_agent_s);
          obj_info_hist->index_put_qat(batch_idx, 15, hist_idx, obj_idx,
                                       rel_agent_l);
          // rel_sl_speed
          const float rel_speed_s =
              std::clamp(((sl_pos_or->s - last_sl_pos.s) -
                          agent_hist_info_vec.at(hist_idx).speed_s) *
                             kInvActorRelSLSpeedSScale,
                         -1.0, 1.0);
          const float rel_speed_l =
              std::clamp(((sl_pos_or->l - last_sl_pos.l) -
                          agent_hist_info_vec.at(hist_idx).speed_l) *
                             kInvActorRelSLSpeedLScale,
                         -1.0, 1.0);
          obj_info_hist->index_put_qat(batch_idx, 16, hist_idx, obj_idx,
                                       rel_speed_s);
          obj_info_hist->index_put_qat(batch_idx, 17, hist_idx, obj_idx,
                                       rel_speed_l);
          if (j == 1) {
            obj_info_hist->index_put_qat(batch_idx, 14, hist_idx - 1, obj_idx,
                                         rel_agent_s);
            obj_info_hist->index_put_qat(batch_idx, 15, hist_idx - 1, obj_idx,
                                         rel_agent_l);
            obj_info_hist->index_put_qat(batch_idx, 16, hist_idx - 1, obj_idx,
                                         rel_speed_s);
            obj_info_hist->index_put_qat(batch_idx, 17, hist_idx - 1, obj_idx,
                                         rel_speed_l);
          }

          // yaw_diff
          const auto& agent_heading =
              agent_states[agent_state_num - hist_num + j].heading;
          const Vec2d yaw_diff = Vec2d::UnitFromAngle(yaw - agent_heading);
          obj_info_hist->index_put_qat(batch_idx, 18, hist_idx, obj_idx,
                                       yaw_diff.x());
          obj_info_hist->index_put_qat(batch_idx, 19, hist_idx, obj_idx,
                                       yaw_diff.y());

          // rel_dist
          const auto& agent_pos =
              agent_states[agent_state_num - hist_num + j].pos;
          const Vec2d rel_pos = pos - agent_pos;
          const float rel_dist =
              std::clamp(ShiftAndScale(rel_pos.norm() * kInvActorRelDistScale),
                         -1.0f, 1.0f);
          obj_info_hist->index_put_qat(batch_idx, 20, hist_idx, obj_idx,
                                       rel_dist);
        }
        last_sl_pos = *sl_pos_or;
      }
      if (!obj_valid) {
        for (int j = 0; j < hist_num; ++j) {
          const int hist_idx = kHistoryNum - hist_num + j;
          for (int channel_idx = 0; channel_idx < 16; ++channel_idx) {
            obj_info_hist->index_put_qat(batch_idx, channel_idx, hist_idx,
                                         obj_idx, 0.0);
          }
        }
        continue;
      }
      // actor_info_attr agent
      int type_idx = std::max(
          std::min<int>(
              static_cast<int>(ObjectTypeToCutinSLNetType(obj_history.type)),
              kObjectTypeDim - 1),
          0);
      for (int k = 0; k < 16; ++k) {
        actor_info_attr->index_put_qat(batch_idx, k, 0, obj_idx + 1,
                                       ShiftAndScale(0.0f));
      }
      actor_info_attr->index_put_qat(batch_idx, type_idx, 0, obj_idx + 1,
                                     ShiftAndScale(1.0f));
      actor_info_attr->index_put_qat(
          batch_idx, 16, 0, obj_idx + 1,
          std::clamp(ShiftAndScale(length * kInvObjectLengthScale), -1.0f,
                     1.0f));
      actor_info_attr->index_put_qat(
          batch_idx, 17, 0, obj_idx + 1,
          std::clamp(ShiftAndScale(width * kInvObjectWidthScale), -1.0f, 1.0f));
      // actor_ctrs agent
      actor_ctrs->index_put_qat(
          batch_idx, 0, 0, obj_idx + 1,
          std::clamp(last_sl_pos.s * kInvActorSLPosSScale, -1.0, 1.0));
      actor_ctrs->index_put_qat(
          batch_idx, 1, 0, obj_idx + 1,
          std::clamp(last_sl_pos.l * kInvActorSLPosLScale, -1.0, 1.0));
    }
    ++obj_idx;
  }
}
}  // namespace

CutinNetJ5Inferencer::CutinNetJ5Inferencer(const NetParam& net_param)
    : net_param_(net_param) {
#if (defined(Q_CPU_ONLY) && defined(__X86_64__)) || defined(__J5__)
  // hack: tensor shape should be aligned shape on j5.
  auto& input_tensor_list = *net_param_.mutable_input_tensor_list();
  (*input_tensor_list[0].mutable_shape())[3] = 16;
  (*input_tensor_list[1].mutable_shape())[3] = 16;
  (*input_tensor_list[2].mutable_shape())[3] = 16;
  (*input_tensor_list[3].mutable_shape())[3] = 16;
  net_param_.mutable_output_tensor_list(0)->set_shape(3, 4);
#endif
  cutin_sl_net_j5_ = std::make_unique<PredictionJ5QNN>(net_param_);
}

std::vector<AgentValidAndL> CutinNetJ5Inferencer::GetBatchInputs(
    const std::vector<ObjectIDType>& objs_to_predict_ids,
    const ObjectHistorySampler& obj_sampler, MapSampler* const map_sampler,
    const planner::DrivePassage& drive_passage) const {
  SCOPED_QTRACE_ARG1("GetBatchInputs", "objs_num:", objs_to_predict_ids.size());
  std::vector<AgentValidAndL> agent_valid_l_vec(objs_to_predict_ids.size(),
                                                {true, 0.0});
  cutin_sl_net_j5_->ResetInputs();
  const std::unordered_map<std::string, std::shared_ptr<HorizonTensorWrapper>>&
      batch_map = cutin_sl_net_j5_->GetInputMap();
  auto& agent_info_hist = *batch_map.at("agent_info_hist");  // [8, 16, 10, 1]
  auto& obj_info_hist = *batch_map.at("obj_info_hist");      // [8, 21, 10, 7]
  auto& actor_info_attr = *batch_map.at("actor_info_attr");  // [8, 18, 1, 8]
  auto& actor_ctrs = *batch_map.at("actor_ctrs");            // [8, 2, 1, 8]
  auto& lc_pos = *batch_map.at("lc_pos");                    // [8, 2, 5, 160]
  auto& lc_seg_info = *batch_map.at("lc_seg_info");          // [8, 5, 5, 160]
  auto& lc_attr_info = *batch_map.at("lc_attr_info");        // [8, 20, 1, 160]
  auto& lb_pos = *batch_map.at("lb_pos");                    // [8, 2, 5, 64]
  auto& lb_seg_info = *batch_map.at("lb_seg_info");          // [8, 5, 5, 64]
  auto& lb_attr_info = *batch_map.at("lb_attr_info");        // [8, 20, 1, 64]

  const auto* av_history = obj_sampler.GetResampledAVMotionHistory();
  const auto& av_pos = av_history->states.back().pos;
  const auto lane_centers =
      map_sampler->GetLaneCentersWithRadius(av_pos, kAvMapRadius);
  const auto lane_boundaries =
      map_sampler->GetSolidLaneBoundariesWithRadius(av_pos, kAvMapRadius);

  for (int batch_idx = 0; batch_idx < objs_to_predict_ids.size(); ++batch_idx) {
    const auto& agent_history = obj_sampler.GetResampledMotionHistoryById(
        objs_to_predict_ids[batch_idx]);
    const auto& last_state = agent_history.states.back();
    const double length = last_state.bbox.length();
    const double width = last_state.bbox.width();
    const int state_num = agent_history.states.size();
    const int hist_num = std::min(state_num, kHistoryNum);
    const double heading = agent_history.states.back().heading;
    const double rot_rad = -heading;
    const auto& ref_position = agent_history.states.back().pos;
    const double speed = agent_history.states.back().vel.Length();
    const auto scan_box_info = GetCutinScanBoxInfo(speed);
    const double scan_box_front = scan_box_info.at(0);
    const double scan_box_back = scan_box_info.at(1);
    const double scan_box_front_half_width = scan_box_info.at(2);
    const Box2d region_box =
        GetRegionBox(ref_position, heading, scan_box_front, scan_box_back,
                     scan_box_front_half_width);
    FrenetCoordinate last_sl_pos;
    bool agent_valid = true;
    std::vector<AgentHistInfo> agent_hist_info_vec(10, {0.0, 0.0, 0.0, 0.0});
    {
      for (int j = 0; j < hist_num; ++j) {
        const auto& state = agent_history.states[state_num - hist_num + j];
        const int hist_idx = kHistoryNum - hist_num + j;
        const auto& pos = state.pos;
        // Extract sl pos
        const auto sl_pos_or =
            drive_passage.QueryUnboundedFrenetCoordinateAt(pos);
        if (!sl_pos_or.ok() || std::abs(sl_pos_or->s) > kSLCorMax ||
            std::abs(sl_pos_or->l) > kSLCorMax) {
          agent_valid = false;
          continue;
        }
        const float scale_s =
            std::clamp(sl_pos_or->s * kInvAgentSLPosSScale, -1.0, 1.0);
        const float scale_l =
            std::clamp(sl_pos_or->l * kInvAgentSLPosLScale, -1.0, 1.0);
        agent_info_hist.index_put_qat(batch_idx, 0, hist_idx, 0, scale_s);
        agent_info_hist.index_put_qat(batch_idx, 1, hist_idx, 0, scale_l);
        agent_hist_info_vec[hist_idx].s = sl_pos_or->s;
        agent_hist_info_vec[hist_idx].l = sl_pos_or->l;
        if (j > 0) {
          const float speed_s = std::clamp(
              (sl_pos_or->s - last_sl_pos.s) * kInvAgentSLSpeedSScale, -1.0,
              1.0);
          const float speed_l = std::clamp(
              (sl_pos_or->l - last_sl_pos.l) * kInvAgentSLSpeedLScale, -1.0,
              1.0);
          agent_info_hist.index_put_qat(batch_idx, 2, hist_idx, 0, speed_s);
          agent_info_hist.index_put_qat(batch_idx, 3, hist_idx, 0, speed_l);
          agent_hist_info_vec[hist_idx].speed_s = sl_pos_or->s - last_sl_pos.s;
          agent_hist_info_vec[hist_idx].speed_l = sl_pos_or->l - last_sl_pos.l;
          if (j == 1) {
            agent_info_hist.index_put_qat(batch_idx, 2, hist_idx - 1, 0,
                                          speed_s);
            agent_info_hist.index_put_qat(batch_idx, 3, hist_idx - 1, 0,
                                          speed_l);
            agent_hist_info_vec[hist_idx - 1].speed_s =
                agent_hist_info_vec[hist_idx].speed_s;
            agent_hist_info_vec[hist_idx - 1].speed_l =
                agent_hist_info_vec[hist_idx].speed_l;
          }
        }
        last_sl_pos = *sl_pos_or;
        const auto yaw = state.heading;
        // Extract sl shape
        const Box2d bbox(pos, yaw, length, width);
        int pt_idx = 0;
        for (const auto& pt : bbox.GetCornersCounterClockwise()) {
          const auto sl_shape_pt_or =
              drive_passage.QueryUnboundedFrenetCoordinateAt(pt);
          if (!sl_shape_pt_or.ok() || std::abs(sl_shape_pt_or->s) > kSLCorMax ||
              std::abs(sl_shape_pt_or->l) > kSLCorMax) {
            agent_valid = false;
            break;
          }
          // 4-11
          const float sl_shape_s =
              std::clamp(sl_shape_pt_or->s * kInvAgentSLShapeSScale, -1.0, 1.0);
          const float sl_shape_l =
              std::clamp(sl_shape_pt_or->l * kInvAgentSLShapeLScale, -1.0, 1.0);
          agent_info_hist.index_put_qat(batch_idx, 4 + pt_idx * 2, hist_idx, 0,
                                        sl_shape_s);
          agent_info_hist.index_put_qat(batch_idx, 4 + pt_idx * 2 + 1, hist_idx,
                                        0, sl_shape_l);
          ++pt_idx;
        }
        // Extract yaw diff to drive passage
        const auto dp_angle_or =
            drive_passage.QueryTangentAngleAtS(sl_pos_or->s);
        if (!dp_angle_or.ok()) {
          agent_valid = false;
          continue;
        }
        const Vec2d yaw_diff_dp = Vec2d::UnitFromAngle(*dp_angle_or - yaw);
        agent_info_hist.index_put_qat(batch_idx, 12, hist_idx, 0,
                                      yaw_diff_dp.x());
        agent_info_hist.index_put_qat(batch_idx, 13, hist_idx, 0,
                                      yaw_diff_dp.y());
        const auto& agent_center_dist_to_lr_lane_boundary =
            QueryDistanceToLeftAndRightAvLaneBoundary(
                drive_passage, sl_pos_or->s, sl_pos_or->l);
        const float agent_center_dist_to_lr_lane_boundary_left =
            std::clamp(agent_center_dist_to_lr_lane_boundary.first *
                           kInvAgentDistToLaneBoundaryScale,
                       -1.0, 1.0);
        const float agent_center_dist_to_lr_lane_boundary_right =
            std::clamp(agent_center_dist_to_lr_lane_boundary.second *
                           kInvAgentDistToLaneBoundaryScale,
                       -1.0, 1.0);
        agent_info_hist.index_put_qat(
            batch_idx, 14, hist_idx, 0,
            agent_center_dist_to_lr_lane_boundary_left);
        agent_info_hist.index_put_qat(
            batch_idx, 15, hist_idx, 0,
            agent_center_dist_to_lr_lane_boundary_right);
      }
      // actor_info_attr agent
      int type_idx = std::max(
          std::min<int>(
              static_cast<int>(ObjectTypeToCutinSLNetType(agent_history.type)),
              kObjectTypeDim - 1),
          0);
      // init type feature
      for (int i = 0; i < 16; ++i) {
        actor_info_attr.index_put_qat(batch_idx, i, 0, 0, ShiftAndScale(0.0f));
      }
      actor_info_attr.index_put_qat(batch_idx, type_idx, 0, 0,
                                    ShiftAndScale(1.0f));
      actor_info_attr.index_put_qat(
          batch_idx, 16, 0, 0,
          std::clamp(
              ShiftAndScale(static_cast<float>(length) * kInvObjectLengthScale),
              -1.0f, 1.0f));
      actor_info_attr.index_put_qat(
          batch_idx, 17, 0, 0,
          std::clamp(
              ShiftAndScale(static_cast<float>(width) * kInvObjectWidthScale),
              -1.0f, 1.0f));
      // actor_ctrs agent
      const float last_sl_pos_s =
          std::clamp(last_sl_pos.s * kInvAgentSLPosSScale, -1.0, 1.0);
      const float last_sl_pos_l =
          std::clamp(last_sl_pos.l * kInvAgentSLPosLScale, -1.0, 1.0);
      actor_ctrs.index_put_qat(batch_idx, 0, 0, 0, last_sl_pos_s);
      actor_ctrs.index_put_qat(batch_idx, 1, 0, 0, last_sl_pos_l);
      agent_valid_l_vec[batch_idx].l = last_sl_pos.l;
      if (!agent_valid) {
        agent_valid_l_vec[batch_idx].valid = false;
        continue;
      }
    }
    {
      SCOPED_QTRACE("PrepareActorsFeature");
      ExtractActorFeature(objs_to_predict_ids[batch_idx], batch_idx, hist_num,
                          region_box, obj_sampler, drive_passage,
                          agent_hist_info_vec, &obj_info_hist, &actor_info_attr,
                          &actor_ctrs);
    }

    {
      SCOPED_QTRACE("PrepareLaneCentersFeature");
      const auto lcs =
          map_sampler->GetNearestLaneCenterPolylinesInBox2dWithLanes(
              region_box, kMaxLaneCenterNum, lane_centers,
              /*compute_boundary_distance=*/false);
      ExtractLaneFeature(batch_idx, rot_rad, ref_position, region_box, lcs,
                         kMaxLaneCenterNum, kLaneSegsNum, drive_passage,
                         &lc_pos, &lc_seg_info, &lc_attr_info);
    }

    {
      SCOPED_QTRACE("PrepareLaneBoundariesFeature");
      const auto lbs =
          map_sampler->GetNearestLaneBoundaryPolylinesInBox2dWithLaneBoundaries(
              region_box, kMaxLaneBoundaryNum, lane_boundaries);
      ExtractLaneFeature(batch_idx, rot_rad, ref_position, region_box, lbs,
                         kMaxLaneBoundaryNum, kLaneSegsNum, drive_passage,
                         &lb_pos, &lb_seg_info, &lb_attr_info);
    }
  }
  return agent_valid_l_vec;
}

// Run Cutin net model, and get the result.
prediction::CutinSLObjectsOut CutinNetJ5Inferencer::PredictForObjects(
    const std::vector<ObjectIDType>& objs_to_predict_ids,
    const ObjectHistorySampler& obj_sampler, MapSampler* const map_sampler,
    const planner::DrivePassage& drive_passage) const {
  SCOPED_QTRACE("CutinNetJ5inferencer::Prediction for Objects");

  prediction::CutinSLObjectsOut objs_out;

  std::vector<AgentValidAndL> agent_valid_l_vec;

  // 1. Feature to Batch inputs.
  {
    SCOPED_QTRACE("CutinNetJ5inferencer::GetBatchInputs from obj sampler");
    agent_valid_l_vec = GetBatchInputs(objs_to_predict_ids, obj_sampler,
                                       map_sampler, drive_passage);
  }
  const int current_batch_size = agent_valid_l_vec.size();
  const int agent_valid_num = std::count_if(
      agent_valid_l_vec.begin(), agent_valid_l_vec.end(),
      [](const AgentValidAndL& valid_l) { return valid_l.valid; });
  if (agent_valid_num <= 0) {
    return objs_out;
  }

  // 2. Run CutinNet forward.
  {
    SCOPED_QTRACE("CutinNetJ5inferencer::ProcessInEngine");
    cutin_sl_net_j5_->Run(current_batch_size);
  }
  // 3. Set objects
  {
    SCOPED_QTRACE("CutinNetJ5Inferencer::AssembleProbTrajs");
    const auto& batch_out_map = cutin_sl_net_j5_->GetOutputMap();

    const auto& batch_channel_probs = batch_out_map.at("channel_probs");

    std::vector<std::pair<double, double>> channel_bucket = {
        std::make_pair(std::numeric_limits<double>::lowest(), -3),
        std::make_pair(-3, -1), std::make_pair(-1, 1), std::make_pair(1, 3),
        std::make_pair(1, std::numeric_limits<double>::max())};
    constexpr int kCutinChannelNum = 5;
    for (int i = 0; i < current_batch_size; ++i) {
      if (!agent_valid_l_vec[i].valid) {
        continue;
      }
      // Assign probabilities.
      // softmax prob
      std::vector<double> channel_probs;
      channel_probs.reserve(kCutinChannelNum);

      for (int j = 0; j < kCutinChannelNum; ++j) {
        channel_probs.push_back(
            batch_channel_probs->index_get_deqat(i, j, 0, 0));
      }
      auto channel_probs_softmax = SoftMax(channel_probs);

      const auto max_position_it = max_element(channel_probs_softmax.begin(),
                                               channel_probs_softmax.end());
      const auto cur_pos_l = agent_valid_l_vec[i].l;
      int cur_channel = 0;
      for (int k = 0; k < channel_bucket.size(); ++k) {
        if (cur_pos_l > channel_bucket[k].first &&
            cur_pos_l < channel_bucket[k].second) {
          cur_channel = k;
          break;
        }
      }
      int predicted_channel = max_position_it - channel_probs_softmax.begin();
      objs_out[objs_to_predict_ids[i]] = prediction::CutinSLObjectOut({
          .channle_probs = std::move(channel_probs_softmax),
          .predicted_channel = predicted_channel,
          .cur_channel = cur_channel,
      });
    }
  }
  return objs_out;
}

}  // namespace cutin_sl_net_j5
}  // namespace prediction
}  // namespace qcraft
