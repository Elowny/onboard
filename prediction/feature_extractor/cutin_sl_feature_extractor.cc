#include "onboard/prediction/feature_extractor/cutin_sl_feature_extractor.h"

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <iterator>
#include <utility>

#include "absl/status/statusor.h"
#include "absl/types/span.h"

#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/math/fast_math.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/geometry/segment2d.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/math/vec.h"
#include "onboard/prediction/feature_extractor/feature_extraction_util.h"
#include "onboard/prediction/prediction_util.h"

namespace qcraft {
namespace prediction {
namespace {
constexpr double kCutinFeatureHistoryStepNum = 21;
constexpr int kBoxCornerNum = 4;
constexpr double kSLCorMax = 500;
constexpr double kSLSpeedMax = 50;

std::optional<CutinSLObjectAbsoluteFeature> GetCutinSLObjectAbsoluteFeature(
    const ObjectMotionHistory& obj_history,
    const planner::DrivePassage& drive_passage) {
  CutinSLObjectAbsoluteFeature feat;
  feat.type = static_cast<float>(ObjectTypeToCutinSLNetType(obj_history.type));
  const auto& last_state = obj_history.states.back();
  const double length = last_state.bbox.length();
  const double width = last_state.bbox.width();
  feat.length_width = {static_cast<float>(length), static_cast<float>(width)};
  const int coord_num = kCutinSLConfig.coord_num;
  const auto state_num = obj_history.states.size();
  const int start_idx = kCutinFeatureHistoryStepNum - state_num;
  std::vector<float> pos_vec, sl_pos_vec, yaw_diff_dp_vec, sl_shape_vec,
      yaw_vec, mask_vec, agent_center_dist_to_av_left_lane_boundary_vec,
      agent_center_dist_to_av_right_lane_boundary_vec,
      agent_corners_dist_to_av_left_lane_boundary_vec,
      agent_corners_dist_to_av_right_lane_boundary_vec;
  pos_vec.reserve(kCutinFeatureHistoryStepNum * coord_num);
  sl_pos_vec.reserve(kCutinFeatureHistoryStepNum * coord_num);
  agent_center_dist_to_av_left_lane_boundary_vec.reserve(
      kCutinFeatureHistoryStepNum);
  agent_center_dist_to_av_right_lane_boundary_vec.reserve(
      kCutinFeatureHistoryStepNum);
  agent_corners_dist_to_av_left_lane_boundary_vec.reserve(
      kCutinFeatureHistoryStepNum * kBoxCornerNum);
  agent_corners_dist_to_av_right_lane_boundary_vec.reserve(
      kCutinFeatureHistoryStepNum * kBoxCornerNum);
  yaw_diff_dp_vec.reserve(kCutinFeatureHistoryStepNum * coord_num);
  sl_shape_vec.reserve(kCutinFeatureHistoryStepNum * coord_num * kBoxCornerNum);
  yaw_vec.reserve(kCutinFeatureHistoryStepNum * coord_num);
  mask_vec.reserve(kCutinFeatureHistoryStepNum);
  for (int i = 0; i < kCutinFeatureHistoryStepNum; ++i) {
    if (i < start_idx) {
      const auto empty_vec2d = {0.0, 0.0};
      std::copy(empty_vec2d.begin(), empty_vec2d.end(),
                std::back_inserter(pos_vec));
      std::copy(empty_vec2d.begin(), empty_vec2d.end(),
                std::back_inserter(sl_pos_vec));
      std::copy(empty_vec2d.begin(), empty_vec2d.end(),
                std::back_inserter(yaw_diff_dp_vec));
      std::copy(empty_vec2d.begin(), empty_vec2d.end(),
                std::back_inserter(yaw_vec));
      for (int j = 0; j < kBoxCornerNum; ++j) {
        agent_corners_dist_to_av_left_lane_boundary_vec.push_back(0.0);
        agent_corners_dist_to_av_right_lane_boundary_vec.push_back(0.0);
        std::copy(empty_vec2d.begin(), empty_vec2d.end(),
                  std::back_inserter(sl_shape_vec));
      }
      agent_center_dist_to_av_left_lane_boundary_vec.push_back(0.0);
      agent_center_dist_to_av_right_lane_boundary_vec.push_back(0.0);
      mask_vec.push_back(0.0);
      continue;
    }

    const auto& state = obj_history.states[i - start_idx];
    const auto& pos = state.pos;

    // Extract pos for distance calculation
    pos_vec.push_back(pos.x());
    pos_vec.push_back(pos.y());

    // Extract sl pos
    const auto sl_pos_or = drive_passage.QueryUnboundedFrenetCoordinateAt(pos);
    if (!sl_pos_or.ok() || std::abs(sl_pos_or->s) > kSLCorMax ||
        std::abs(sl_pos_or->l) > kSLCorMax) {
      return std::nullopt;
    }

    sl_pos_vec.push_back(sl_pos_or->s);
    sl_pos_vec.push_back(sl_pos_or->l);

    const auto& agent_center_dist_to_lr_lane_boundary =
        QueryDistanceToLeftAndRightAvLaneBoundary(drive_passage, sl_pos_or->s,
                                                  sl_pos_or->l);

    agent_center_dist_to_av_left_lane_boundary_vec.push_back(
        agent_center_dist_to_lr_lane_boundary.first);
    agent_center_dist_to_av_right_lane_boundary_vec.push_back(
        agent_center_dist_to_lr_lane_boundary.second);

    // Extract yaw
    const auto yaw = state.heading;
    yaw_vec.push_back(fast_math::Cos(yaw));
    yaw_vec.push_back(fast_math::Sin(yaw));

    // Extract yaw diff to drive passage
    const auto dp_angle_or = drive_passage.QueryTangentAngleAtS(sl_pos_or->s);
    if (!dp_angle_or.ok()) {
      return std::nullopt;
    }
    const Vec2d yaw_diff_dp = Vec2d::UnitFromAngle(*dp_angle_or - yaw);
    yaw_diff_dp_vec.push_back(yaw_diff_dp.x());
    yaw_diff_dp_vec.push_back(yaw_diff_dp.y());

    // Extract sl shape
    const Box2d bbox(pos, yaw, length, width);
    for (const auto& pt : bbox.GetCornersCounterClockwise()) {
      const auto sl_shape_pt_or =
          drive_passage.QueryUnboundedFrenetCoordinateAt(pt);
      if (!sl_shape_pt_or.ok() || std::abs(sl_shape_pt_or->s) > kSLCorMax ||
          std::abs(sl_shape_pt_or->l) > kSLCorMax) {
        return std::nullopt;
      }
      const auto& agent_corners_dist_to_lr_lane_boundary =
          QueryDistanceToLeftAndRightAvLaneBoundary(
              drive_passage, sl_shape_pt_or->s, sl_shape_pt_or->l);
      agent_corners_dist_to_av_left_lane_boundary_vec.push_back(
          agent_corners_dist_to_lr_lane_boundary.first);
      agent_corners_dist_to_av_right_lane_boundary_vec.push_back(
          agent_corners_dist_to_lr_lane_boundary.second);
      sl_shape_vec.push_back(sl_shape_pt_or->s);
      sl_shape_vec.push_back(sl_shape_pt_or->l);
    }
    mask_vec.push_back(1.0);
  }

  std::vector<float> sl_speed_vec;
  sl_speed_vec.reserve(kCutinFeatureHistoryStepNum * coord_num);
  for (int i = 0; i < kCutinFeatureHistoryStepNum; ++i) {
    if (i < start_idx || start_idx == kCutinFeatureHistoryStepNum - 1) {
      sl_speed_vec.push_back(0.0);
      sl_speed_vec.push_back(0.0);
      continue;
    }
    if (i == start_idx) {
      sl_speed_vec.push_back(
          (sl_pos_vec[(i + 1) * coord_num] - sl_pos_vec[i * coord_num]) /
          kPredictionTimeStep);
      sl_speed_vec.push_back((sl_pos_vec[(i + 1) * coord_num + 1] -
                              sl_pos_vec[i * coord_num + 1]) /
                             kPredictionTimeStep);
    } else {
      sl_speed_vec.push_back(
          (sl_pos_vec[i * coord_num] - sl_pos_vec[(i - 1) * coord_num]) /
          kPredictionTimeStep);
      sl_speed_vec.push_back((sl_pos_vec[i * coord_num + 1] -
                              sl_pos_vec[(i - 1) * coord_num + 1]) /
                             kPredictionTimeStep);
    }
  }

  for (const auto& speed : sl_speed_vec) {
    if (std::abs(speed) > kSLSpeedMax) {
      return std::nullopt;
    }
  }

  feat.sl_pos = std::move(sl_pos_vec);
  feat.sl_speed = std::move(sl_speed_vec);
  feat.yaw_diff_dp = std::move(yaw_diff_dp_vec);
  feat.sl_shape = std::move(sl_shape_vec);
  feat.yaw = std::move(yaw_vec);
  feat.mask = std::move(mask_vec);
  feat.pos = std::move(pos_vec);
  feat.agent_center_dist_to_av_left_lane_boundary =
      std::move(agent_center_dist_to_av_left_lane_boundary_vec);
  feat.agent_center_dist_to_av_right_lane_boundary =
      std::move(agent_center_dist_to_av_right_lane_boundary_vec);
  feat.agent_corners_dist_to_av_left_lane_boundary =
      std::move(agent_corners_dist_to_av_left_lane_boundary_vec);
  feat.agent_corners_dist_to_av_right_lane_boundary =
      std::move(agent_corners_dist_to_av_right_lane_boundary_vec);

  return feat;
}

std::optional<CutinSLObjectFeatures> GetCutinSLObjectFeatures(
    const CutinSLObjectAbsoluteFeature& agent_feat,
    const CutinSLObjectAbsoluteFeature& obj_feat) {  // NOLINT
  const int& coord_num = kCutinSLConfig.coord_num;
  std::vector<float> rel_sl_pos_agent_vec, rel_sl_speed_agent_vec,
      yaw_diff_agent_vec, rel_dist_agent_vec;

  for (int i = 0; i < kCutinFeatureHistoryStepNum; ++i) {
    if (static_cast<int>(agent_feat.mask[i]) == 0 ||
        static_cast<int>(obj_feat.mask[i]) == 0) {
      const auto empty_vec2d = {0.0, 0.0};
      std::copy(empty_vec2d.begin(), empty_vec2d.end(),
                std::back_inserter(rel_sl_pos_agent_vec));
      std::copy(empty_vec2d.begin(), empty_vec2d.end(),
                std::back_inserter(rel_sl_speed_agent_vec));
      std::copy(empty_vec2d.begin(), empty_vec2d.end(),
                std::back_inserter(yaw_diff_agent_vec));
      rel_dist_agent_vec.push_back(0.0);
      continue;
    }

    // Extracet relative sl coordinates.
    const Vec2d obj_sl_pos(obj_feat.sl_pos[i * coord_num],
                           obj_feat.sl_pos[i * coord_num + 1]);
    const Vec2d agent_sl_pos(agent_feat.sl_pos[i * coord_num],
                             agent_feat.sl_pos[i * coord_num + 1]);
    const Vec2d rel_sl_pos = obj_sl_pos - agent_sl_pos;
    rel_sl_pos_agent_vec.push_back(rel_sl_pos.x());
    rel_sl_pos_agent_vec.push_back(rel_sl_pos.y());

    // Extract dist from object to agent.
    const Vec2d obj_pos(obj_feat.pos[i * coord_num],
                        obj_feat.pos[i * coord_num + 1]);
    const Vec2d agent_pos(agent_feat.pos[i * coord_num],
                          agent_feat.pos[i * coord_num + 1]);
    const Vec2d rel_pos = obj_pos - agent_pos;
    const double rel_dist = rel_pos.norm();
    rel_dist_agent_vec.push_back(rel_dist);

    // Extract relative speed from object to agent.
    const Vec2d obj_sl_speed(obj_feat.sl_speed[i * coord_num],
                             obj_feat.sl_speed[i * coord_num + 1]);
    const Vec2d agent_sl_speed(agent_feat.sl_speed[i * coord_num],
                               agent_feat.sl_speed[i * coord_num + 1]);
    const Vec2d rel_sl_speed = obj_sl_speed - agent_sl_speed;
    rel_sl_speed_agent_vec.push_back(rel_sl_speed.x());
    rel_sl_speed_agent_vec.push_back(rel_sl_speed.y());

    // Extract yaw difference from object to agent.
    const Vec2d agent_tangent(agent_feat.yaw[i * coord_num],
                              agent_feat.yaw[i * coord_num + 1]);
    const Vec2d obj_tangent(obj_feat.yaw[i * coord_num],
                            obj_feat.yaw[i * coord_num + 1]);
    const Vec2d yaw_diff =
        Vec2d::UnitFromAngle(obj_tangent.Angle() - agent_tangent.Angle());
    yaw_diff_agent_vec.push_back(yaw_diff.x());
    yaw_diff_agent_vec.push_back(yaw_diff.y());
  }

  return CutinSLObjectFeatures{
      .sl_pos = obj_feat.sl_pos,
      .sl_speed = obj_feat.sl_speed,
      .yaw_diff_dp = obj_feat.yaw_diff_dp,
      .length_width = obj_feat.length_width,
      .sl_shape = obj_feat.sl_shape,
      .type = obj_feat.type,

      .rel_sl_pos_agent = std::move(rel_sl_pos_agent_vec),
      .rel_sl_speed_agent = std::move(rel_sl_speed_agent_vec),
      .yaw_diff_agent = std::move(yaw_diff_agent_vec),
      .rel_dist_agent = std::move(rel_dist_agent_vec),
      .obj_center_dist_to_av_left_lane_boundary =
          obj_feat.agent_center_dist_to_av_left_lane_boundary,
      .obj_center_dist_to_av_right_lane_boundary =
          obj_feat.agent_center_dist_to_av_right_lane_boundary,
      .obj_corners_dist_to_av_left_lane_boundary =
          obj_feat.agent_corners_dist_to_av_left_lane_boundary,
      .obj_corners_dist_to_av_right_lane_boundary =
          obj_feat.agent_corners_dist_to_av_right_lane_boundary};
}

std::vector<CutinSLPolylineFeatures> ExtractMapSLFeatureFromPolyline(
    const SampledPolyline& polyline, const planner::DrivePassage& drive_passage,
    const Vec2d& orig, double rot_rad, int max_lane_seg_num) {
  constexpr int kCoordNum = kCutinSLConfig.coord_num;
  constexpr int kLightNum = 4;

  std::vector<CutinSLPolylineFeatures> feats;
  const auto& segs = polyline.segs;
  const int segs_size = segs.size();
  if (segs_size == 0) {
    return feats;
  }
  QCHECK_GT(max_lane_seg_num, 0);
  int num_feat = 0;
  if (segs_size % max_lane_seg_num == 0) {
    num_feat = segs_size / max_lane_seg_num;
  } else {
    num_feat = segs_size / max_lane_seg_num + 1;
  }
  feats.resize(num_feat);
  for (int i = 0; i < feats.size(); ++i) {
    auto& feat = feats[i];
    feat.id = polyline.id;
    if (segs_size % max_lane_seg_num == 0 || i < feats.size() - 1) {
      feat.seg.resize(max_lane_seg_num * kCoordNum * 2, 0.0);
      feat.sl_seg.resize(max_lane_seg_num * kCoordNum * 2, 0.0);
      feat.seg_len.resize(max_lane_seg_num, 0.0);
      feat.dist.resize(max_lane_seg_num, 0.0);
      feat.nearest_to_end_dist.resize(max_lane_seg_num, 0.0);
      feat.yaw_diff.resize(max_lane_seg_num * kCoordNum, 0.0);
      feat.type.resize(max_lane_seg_num, 0.0);
      feat.light.resize(max_lane_seg_num * kLightNum, 0.0);
      feat.mask.resize(max_lane_seg_num, 0.0);
    } else {
      feat.seg.resize((segs_size % max_lane_seg_num) * kCoordNum * 2, 0.0);
      feat.sl_seg.resize((segs_size % max_lane_seg_num) * kCoordNum * 2, 0.0);
      feat.seg_len.resize((segs_size % max_lane_seg_num), 0.0);
      feat.dist.resize((segs_size % max_lane_seg_num), 0.0);
      feat.nearest_to_end_dist.resize((segs_size % max_lane_seg_num), 0.0);
      feat.yaw_diff.resize((segs_size % max_lane_seg_num) * kCoordNum, 0.0);
      feat.type.resize((segs_size % max_lane_seg_num), 0.0);
      feat.light.resize((segs_size % max_lane_seg_num) * kLightNum, 0.0);
      feat.mask.resize((segs_size % max_lane_seg_num), 0.0);
    }
  }
  for (int i = 0; i < segs_size; ++i) {
    const int feat_idx = i / max_lane_seg_num;
    auto& feat = feats[feat_idx];
    const auto& cur = segs[i].seg.start();
    const auto& next = segs[i].seg.end();
    const int seg_idx = i - feat_idx * max_lane_seg_num;

    // Extract xy pos for region box filtering operation.
    const auto cur_pos = (cur - orig).Rotate(rot_rad);
    const auto next_pos = (next - orig).Rotate(rot_rad);

    feat.seg[seg_idx * kCoordNum * 2] = cur_pos.x();
    feat.seg[seg_idx * kCoordNum * 2 + 1] = cur_pos.y();
    feat.seg[seg_idx * kCoordNum * 2 + 2] = next_pos.x();
    feat.seg[seg_idx * kCoordNum * 2 + 3] = next_pos.y();

    // Extract sl pos
    const auto cur_sl_pos = drive_passage.QueryUnboundedFrenetCoordinateAt(cur);
    const auto next_sl_pos =
        drive_passage.QueryUnboundedFrenetCoordinateAt(next);

    if (!cur_sl_pos.ok() || !next_sl_pos.ok() ||
        std::abs(cur_sl_pos->s) > kSLCorMax ||
        std::abs(cur_sl_pos->l) > kSLCorMax ||
        std::abs(next_sl_pos->s) > kSLCorMax ||
        std::abs(next_sl_pos->l) > kSLCorMax) {
      continue;
    }

    feat.sl_seg[seg_idx * kCoordNum * 2] = cur_sl_pos->s;
    feat.sl_seg[seg_idx * kCoordNum * 2 + 1] = cur_sl_pos->l;
    feat.sl_seg[seg_idx * kCoordNum * 2 + 2] = next_sl_pos->s;
    feat.sl_seg[seg_idx * kCoordNum * 2 + 3] = next_sl_pos->l;

    // Extract seg length
    const auto seg_vec = next - cur;
    const auto seg_len = seg_vec.norm();
    feat.seg_len[seg_idx] = seg_len;

    // Extract dist
    Vec2d nearest_pt;
    const Segment2d cur_seg(cur, next);
    const double dist = cur_seg.DistanceTo(orig, &nearest_pt);
    feat.dist[seg_idx] = dist;

    // Extract nearest_to_end_dist
    const double nearest_to_end_dist = (nearest_pt - next).norm();
    feat.nearest_to_end_dist[seg_idx] = nearest_to_end_dist;

    // Extract yaw diff
    const auto yaw_diff = Vec2d::UnitFromAngle(seg_vec.Angle() + rot_rad);
    feat.yaw_diff[seg_idx * kCoordNum] = yaw_diff.x();
    feat.yaw_diff[seg_idx * kCoordNum + 1] = yaw_diff.y();

    // Extract type
    feat.type[seg_idx] = static_cast<float>(segs[i].type);

    // Extract light
    for (int k = 0; k < kLightNum; ++k) {
      feat.light[seg_idx * kLightNum + k] = segs[i].lights[k];
    }

    // Extract mask
    feat.mask[seg_idx] = 1.0;
  }
  return feats;
}

std::vector<CutinSLPolylineFeatures> ExtractMapSLFeatureFromPolylines(
    absl::Span<const SampledPolyline* const> polylines,
    const planner::DrivePassage& drive_passage, double front, double back,
    double half_width, const Vec2d& orig, double rot_rad, int max_lane_seg_num,
    int max_feat_num) {
  constexpr int kCoordNum = kCutinSLConfig.coord_num;
  std::vector<CutinSLPolylineFeatures> combined_feats;
  constexpr int kNominalSegNum = 3;
  combined_feats.reserve(polylines.size() * kNominalSegNum);
  for (const auto* polyline : polylines) {
    auto feats = ExtractMapSLFeatureFromPolyline(*polyline, drive_passage, orig,
                                                 rot_rad, max_lane_seg_num);
    std::move(feats.begin(), feats.end(), std::back_inserter(combined_feats));
  }
  std::sort(combined_feats.begin(), combined_feats.end(),
            [](const auto& feat1, const auto feat2) {
              return feat1.dist.front() < feat2.dist.front();
            });

  // Filter again polylines to find polylines within the region box.
  const Box2d cur_box =
      GetRegionBox(Vec2d(0.0, 0.0), 0.0, front, back, half_width);
  std::vector<CutinSLPolylineFeatures> filtered_feats;
  filtered_feats.reserve(max_feat_num);
  for (int i = 0; i < combined_feats.size(); ++i) {
    // Check if any point of polyline is inside the region box.
    for (int j = 0; j < combined_feats[i].mask.size(); ++j) {
      const int mask = static_cast<int>(combined_feats[i].mask[j]);
      if (mask == 0) {
        break;
      }
      const Vec2d pt(combined_feats[i].seg[j * kCoordNum * 2],
                     combined_feats[i].seg[j * kCoordNum * 2 + 1]);
      if (cur_box.IsPointIn(pt)) {
        filtered_feats.push_back(std::move(combined_feats[i]));
        break;
      }
    }
    if (filtered_feats.size() >= max_feat_num) {
      break;
    }
  }
  return filtered_feats;
}

}  // namespace

std::optional<std::vector<FrenetCoordinate>> TransformXYGroundTruthToSL(
    const TrajectoryGroundTruth& gt, const planner::DrivePassage& drive_passage,
    int gt_size) {
  std::vector<FrenetCoordinate> res;
  for (int i = 0; i < gt_size; ++i) {
    const auto& point = gt.gt_points(i);
    const auto point_sl_or =
        drive_passage.QueryUnboundedFrenetCoordinateAt(Vec2d(point.pos()));
    if (!point_sl_or.ok() || std::abs(point_sl_or->s) > kSLCorMax ||
        std::abs(point_sl_or->l) > kSLCorMax) {
      return std::nullopt;
    }
    res.push_back(*point_sl_or);
  }
  return res;
}

std::optional<CutinSLFeature> ExtractCutinSLFeature(
    const ObjectIDType& agent_id, const ObjectHistorySampler& obj_sampler,
    const planner::DrivePassage& drive_passage,
    const std::vector<const mapping::LaneInfo*>& candi_lane_centers,
    const std::vector<const mapping::LaneBoundaryInfo*>& candi_lane_boundaries,
    const std::vector<const mapping::CrosswalkInfo*>& candi_crosswalks,
    MapSampler* const map_sampler_ptr) {
  constexpr int kCutinSLConsideredObjsNum = 7;
  auto& map_sampler = *map_sampler_ptr;
  const auto& agent_history =
      obj_sampler.GetResampledMotionHistoryById(agent_id);
  const double heading = agent_history.states.back().heading;
  const auto& ref_position = agent_history.states.back().pos;
  const double rot_rad = -heading;
  const double speed = agent_history.states.back().vel.Length();
  const auto agent_feat =
      GetCutinSLObjectAbsoluteFeature(agent_history, drive_passage);

  if (agent_feat == std::nullopt) {
    return std::nullopt;
  }
  const double scan_box_front_dist = kScanBoxFrontPlf(speed);
  const double scan_box_back_dist = kScanBoxBack;
  const double scan_box_front_half_width = kScanBoxFrontHalfWidthPlf(speed);
  // Extract objects features
  const Box2d region_box =
      GetRegionBox(ref_position, heading, scan_box_front_dist,
                   scan_box_back_dist, scan_box_front_half_width);

  const auto objects_in_box2d =
      obj_sampler.GetResampledMotionHistoryWithAVInBox2d(
          agent_id, region_box, kCutinSLConfig.max_other_objects_num);
  std::vector<CutinSLObjectFeatures> objs_features;
  objs_features.reserve(kCutinSLConsideredObjsNum);

  // Only consider limited number of objects.
  if (objects_in_box2d.size() > kCutinSLConsideredObjsNum) {
    // Copy the elements in a new vector since const elements can not be sorted
    std::vector<ObjectMotionHistory> objects_in_box2d_sorted;
    objects_in_box2d_sorted.reserve(objects_in_box2d.size());
    for (const auto& obj : objects_in_box2d) {
      objects_in_box2d_sorted.emplace_back(*obj);
    }
    // Sort the objects according to its distance to agent
    std::sort(objects_in_box2d_sorted.begin() + 1,
              objects_in_box2d_sorted.end(),
              [&ref_position](const ObjectMotionHistory& x,
                              const ObjectMotionHistory& y) {
                const auto x_pos = x.states.back().pos;
                const auto y_pos = y.states.back().pos;
                return x_pos.DistanceSquareTo(ref_position) <
                       y_pos.DistanceSquareTo(ref_position);
              });

    for (int i = 0, obj_num = 0; i < objects_in_box2d_sorted.size() &&
                                 obj_num < kCutinSLConsideredObjsNum;
         ++i) {
      const auto obj_hist = objects_in_box2d_sorted[i];
      const auto obj_abs_features =
          GetCutinSLObjectAbsoluteFeature(obj_hist, drive_passage);

      if (obj_abs_features == std::nullopt) {
        continue;
      }

      const auto obj_features =
          GetCutinSLObjectFeatures(*agent_feat, *obj_abs_features);
      if (obj_features == std::nullopt) {
        continue;
      }
      objs_features.push_back(*obj_features);
      ++obj_num;
    }

  } else {
    for (int i = 0; i < objects_in_box2d.size(); ++i) {
      const auto* obj_hist = objects_in_box2d[i];
      const auto obj_abs_features =
          GetCutinSLObjectAbsoluteFeature(*obj_hist, drive_passage);

      if (obj_abs_features == std::nullopt) {
        continue;
      }

      const auto obj_features =
          GetCutinSLObjectFeatures(*agent_feat, *obj_abs_features);

      if (obj_features == std::nullopt) {
        continue;
      }
      objs_features.push_back(*obj_features);
    }
  }

  // Extract map features.

  // Lane center sl features.
  const auto lcs = map_sampler.GetNearestLaneCenterPolylinesInBox2dWithLanes(
      region_box, kCutinSLConfig.max_lc_num, candi_lane_centers,
      /*compute_boundary_distance=*/false);
  auto lc_sl_feats = ExtractMapSLFeatureFromPolylines(
      lcs, drive_passage, scan_box_front_dist, scan_box_back_dist,
      scan_box_front_half_width, ref_position, rot_rad,
      kCutinSLConfig.max_lane_seg_num, kCutinSLConfig.max_lc_num);
  // If the extracted feature does not contain any lane center
  // polyline, do not make prediction.
  if (lc_sl_feats.size() == 0) {
    return std::nullopt;
  }

  std::vector<float> lc_masks(kCutinSLConfig.max_lc_num, 0.0);
  for (int i = 0; i < lc_sl_feats.size(); ++i) {
    lc_masks[i] = 1.0;
  }

  // Lane boundry sl features
  const auto lbs =
      map_sampler.GetNearestLaneBoundaryPolylinesInBox2dWithLaneBoundaries(
          region_box, kCutinSLConfig.max_lb_num, candi_lane_boundaries);
  auto lb_sl_feats = ExtractMapSLFeatureFromPolylines(
      lbs, drive_passage, scan_box_front_dist, scan_box_back_dist,
      scan_box_front_half_width, ref_position, rot_rad,
      kCutinSLConfig.max_lane_seg_num, kCutinSLConfig.max_lb_num);

  std::vector<float> lb_masks(kCutinSLConfig.max_lb_num, 0.0);
  for (int i = 0; i < lb_sl_feats.size(); ++i) {
    lb_masks[i] = 1.0;
  }

  // Crosswalk sl features
  const auto cws =
      map_sampler.GetNearestCrosswalkPolylinesInBox2dWithCrosswalks(
          region_box, kCutinSLConfig.max_crosswalk_num, candi_crosswalks);
  auto cw_sl_feats = ExtractMapSLFeatureFromPolylines(
      cws, drive_passage, scan_box_front_dist, scan_box_back_dist,
      scan_box_front_half_width, ref_position, rot_rad,
      kCutinSLConfig.max_lane_seg_num, kCutinSLConfig.max_crosswalk_num);

  std::vector<float> cw_masks(kCutinSLConfig.max_crosswalk_num, 0.0);
  for (int i = 0; i < cw_sl_feats.size(); ++i) {
    cw_masks[i] = 1.0;
  }

  // Extract drive passage xy seg info
  const auto& drive_passage_segs = drive_passage.segments();
  std::vector<float> dp_xy_seg;
  for (const auto& seg : drive_passage_segs) {
    dp_xy_seg.push_back(seg.start().x());
    dp_xy_seg.push_back(seg.start().y());
    dp_xy_seg.push_back(seg.end().x());
    dp_xy_seg.push_back(seg.end().y());
  }

  auto left_right_lbs = GetLeftRightLaneBoundaries(drive_passage);
  auto& left_lane_lb = left_right_lbs.left_boundary;
  auto& right_lane_lb = left_right_lbs.right_boundary;

  return CutinSLFeature{.agent_id = agent_id,
                        .agent_sl_feature = *agent_feat,
                        .objs_sl_features = std::move(objs_features),
                        .lc_sl_features = std::move(lc_sl_feats),
                        .lb_sl_features = std::move(lb_sl_feats),
                        .cw_sl_features = std::move(cw_sl_feats),
                        .lc_masks = std::move(lc_masks),
                        .lb_masks = std::move(lb_masks),
                        .cw_masks = std::move(cw_masks),
                        .dp_xy_seg = std::move(dp_xy_seg),
                        .left_lane_boundary = std::move(left_lane_lb),
                        .right_lane_boundary = std::move(right_lane_lb)};
}

CutinSLDumpedFeatureProto ToCutinSLDumpedFeatureProto(
    const CutinSLFeature& cutin_sl_feature,
    const absl::Span<const FrenetCoordinate> agent_gt,
    const absl::Span<const CrossType> if_cross_gts, double ts) {
  CutinSLDumpedFeatureProto feature_proto;
  feature_proto.set_agent_id(cutin_sl_feature.agent_id);
  feature_proto.set_timestamp(ts);

  // Fill in agent feature.
  auto* mutable_agent_feature = feature_proto.mutable_agent_sl_feature();
  const auto& agent_sl_feature = cutin_sl_feature.agent_sl_feature;
  *mutable_agent_feature->mutable_sl_pos() = {agent_sl_feature.sl_pos.begin(),
                                              agent_sl_feature.sl_pos.end()};
  *mutable_agent_feature->mutable_sl_speed() = {
      agent_sl_feature.sl_speed.begin(), agent_sl_feature.sl_speed.end()};
  *mutable_agent_feature->mutable_yaw_diff_dp() = {
      agent_sl_feature.yaw_diff_dp.begin(), agent_sl_feature.yaw_diff_dp.end()};
  *mutable_agent_feature->mutable_sl_shape() = {
      agent_sl_feature.sl_shape.begin(), agent_sl_feature.sl_shape.end()};
  *mutable_agent_feature->mutable_length_width() = {
      agent_sl_feature.length_width.begin(),
      agent_sl_feature.length_width.end()};
  mutable_agent_feature->set_type(agent_sl_feature.type);
  *mutable_agent_feature->mutable_agent_center_dist_to_av_left_lane_boundary() =
      {agent_sl_feature.agent_center_dist_to_av_left_lane_boundary.begin(),
       agent_sl_feature.agent_center_dist_to_av_left_lane_boundary.end()};
  *mutable_agent_feature
       ->mutable_agent_center_dist_to_av_right_lane_boundary() = {
      agent_sl_feature.agent_center_dist_to_av_right_lane_boundary.begin(),
      agent_sl_feature.agent_center_dist_to_av_right_lane_boundary.end()};
  *mutable_agent_feature
       ->mutable_agent_corners_dist_to_av_left_lane_boundary() = {
      agent_sl_feature.agent_corners_dist_to_av_left_lane_boundary.begin(),
      agent_sl_feature.agent_corners_dist_to_av_left_lane_boundary.end()};
  *mutable_agent_feature
       ->mutable_agent_corners_dist_to_av_right_lane_boundary() = {
      agent_sl_feature.agent_corners_dist_to_av_right_lane_boundary.begin(),
      agent_sl_feature.agent_corners_dist_to_av_right_lane_boundary.end()};
  // Fill in objects feature.
  const auto& objs_sl_features = cutin_sl_feature.objs_sl_features;
  auto* mutable_objects_sl_feature = feature_proto.mutable_objects_sl_feature();
  for (const auto& obj_sl_feature : objs_sl_features) {
    mutable_objects_sl_feature->mutable_sl_pos()->Add(
        obj_sl_feature.sl_pos.begin(), obj_sl_feature.sl_pos.end());
    mutable_objects_sl_feature->mutable_sl_speed()->Add(
        obj_sl_feature.sl_speed.begin(), obj_sl_feature.sl_speed.end());
    mutable_objects_sl_feature->mutable_yaw_diff_dp()->Add(
        obj_sl_feature.yaw_diff_dp.begin(), obj_sl_feature.yaw_diff_dp.end());
    mutable_objects_sl_feature->mutable_sl_shape()->Add(
        obj_sl_feature.sl_shape.begin(), obj_sl_feature.sl_shape.end());
    mutable_objects_sl_feature->mutable_length_width()->Add(
        obj_sl_feature.length_width.begin(), obj_sl_feature.length_width.end());

    mutable_objects_sl_feature->mutable_rel_sl_pos_agent()->Add(
        obj_sl_feature.rel_sl_pos_agent.begin(),
        obj_sl_feature.rel_sl_pos_agent.end());
    mutable_objects_sl_feature->mutable_rel_sl_speed_agent()->Add(
        obj_sl_feature.rel_sl_speed_agent.begin(),
        obj_sl_feature.rel_sl_speed_agent.end());
    mutable_objects_sl_feature->mutable_yaw_diff_agent()->Add(
        obj_sl_feature.yaw_diff_agent.begin(),
        obj_sl_feature.yaw_diff_agent.end());
    mutable_objects_sl_feature->mutable_rel_dist_agent()->Add(
        obj_sl_feature.rel_dist_agent.begin(),
        obj_sl_feature.rel_dist_agent.end());
    mutable_objects_sl_feature->add_type(obj_sl_feature.type);
  }

  // Fill in map features
  std::vector<const std::vector<CutinSLPolylineFeatures>*> poly_features = {
      &cutin_sl_feature.lc_sl_features, &cutin_sl_feature.lb_sl_features,
      &cutin_sl_feature.cw_sl_features};

  auto* mutable_lc_sl_feature =
      feature_proto.mutable_lane_centers_sl_features();
  auto* mutable_lb_sl_feature =
      feature_proto.mutable_lane_boundaries_sl_features();
  auto* mutable_cw_sl_feature = feature_proto.mutable_crosswalks_sl_features();

  std::vector<CutinSLDumpedFeatureProto::MapDumpedFeature*> feature_protos = {
      mutable_lc_sl_feature, mutable_lb_sl_feature, mutable_cw_sl_feature};

  for (int i = 0; i < poly_features.size(); ++i) {
    const auto& feats = *poly_features[i];
    auto& mutable_feat = *feature_protos[i];
    for (const auto& feat : feats) {
      mutable_feat.mutable_sl_seg()->Add(feat.sl_seg.begin(),
                                         feat.sl_seg.end());
      mutable_feat.mutable_seg_len()->Add(feat.seg_len.begin(),
                                          feat.seg_len.end());
      mutable_feat.mutable_dist()->Add(feat.dist.begin(), feat.dist.end());
      mutable_feat.mutable_nearest_to_end_dist()->Add(
          feat.nearest_to_end_dist.begin(), feat.nearest_to_end_dist.end());
      mutable_feat.mutable_yaw_diff()->Add(feat.yaw_diff.begin(),
                                           feat.yaw_diff.end());
      mutable_feat.mutable_type()->Add(feat.type.begin(), feat.type.end());
      mutable_feat.mutable_light()->Add(feat.light.begin(), feat.light.end());
      mutable_feat.mutable_mask()->Add(feat.mask.begin(), feat.mask.end());
    }
  }
  *feature_proto.mutable_lcs_masks() = {cutin_sl_feature.lc_masks.begin(),
                                        cutin_sl_feature.lc_masks.end()};
  *feature_proto.mutable_lbs_masks() = {cutin_sl_feature.lb_masks.begin(),
                                        cutin_sl_feature.lb_masks.end()};
  *feature_proto.mutable_cws_masks() = {cutin_sl_feature.cw_masks.begin(),
                                        cutin_sl_feature.cw_masks.end()};

  // Dump gt:
  for (const auto& gt_point : agent_gt) {
    feature_proto.mutable_agent_sl_gt()->Add(gt_point.s);
    feature_proto.mutable_agent_sl_gt()->Add(gt_point.l);
  }

  feature_proto.set_if_center_cross_left_boundary_gt(
      static_cast<float>(if_cross_gts[0]));
  feature_proto.set_if_center_cross_right_boundary_gt(
      static_cast<float>(if_cross_gts[1]));
  feature_proto.set_if_corner_cross_left_boundary_gt(
      static_cast<float>(if_cross_gts[2]));
  feature_proto.set_if_corner_cross_right_boundary_gt(
      static_cast<float>(if_cross_gts[3]));

  // Dump dp:
  *feature_proto.mutable_dp_xy_seg() = {cutin_sl_feature.dp_xy_seg.begin(),
                                        cutin_sl_feature.dp_xy_seg.end()};
  *feature_proto.mutable_left_lane_boundary() = {
      cutin_sl_feature.left_lane_boundary.begin(),
      cutin_sl_feature.left_lane_boundary.end()};
  *feature_proto.mutable_right_lane_boundary() = {
      cutin_sl_feature.right_lane_boundary.begin(),
      cutin_sl_feature.right_lane_boundary.end()};
  return feature_proto;
}
}  // namespace prediction
}  // namespace qcraft
