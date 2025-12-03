#include "onboard/prediction/feature_extractor/lane_selection_feature_extractor.h"

#include <algorithm>
#include <utility>

#include "onboard/prediction/feature_extractor/feature_extraction_util.h"
#include "onboard/prediction/prediction_util.h"

namespace qcraft {
namespace prediction {
namespace {
constexpr double kLaneSelectionFeatureHistoryStepNum = 21;
constexpr int kBoxCornerNum = 4;
constexpr double kSLCorMax = 500;
constexpr double kSLSpeedMax = 50;
constexpr int kCoordNum = 2;

LaneSelectionAgentFeatures GetLaneSelectionAgentFeatures(
    const ObjectMotionHistory& agent_history,
    const planner::DrivePassage& drive_passage,
    const LaneSelectionObjectCommonFeatures& agent_common_feat) {
  constexpr int kLaneSelectionLeapStep = 10;
  constexpr int kLeapStartPos =
      kLaneSelectionFeatureHistoryStepNum - kLaneSelectionLeapStep;
  const auto state_num = agent_history.states.size();
  const int start_idx = kLaneSelectionFeatureHistoryStepNum - state_num;

  // Extract vec features
  std::vector<float> agent_center_dist_to_av_left_lane_boundary_vec,
      agent_center_dist_to_av_right_lane_boundary_vec,
      agent_corners_dist_to_av_left_lane_boundary_vec,
      agent_corners_dist_to_av_right_lane_boundary_vec, lane_width;

  agent_center_dist_to_av_left_lane_boundary_vec.reserve(
      kLaneSelectionFeatureHistoryStepNum);
  agent_center_dist_to_av_right_lane_boundary_vec.reserve(
      kLaneSelectionFeatureHistoryStepNum);
  agent_corners_dist_to_av_left_lane_boundary_vec.reserve(
      kLaneSelectionFeatureHistoryStepNum * kBoxCornerNum);
  agent_corners_dist_to_av_right_lane_boundary_vec.reserve(
      kLaneSelectionFeatureHistoryStepNum * kBoxCornerNum);
  lane_width.reserve(kLaneSelectionFeatureHistoryStepNum);
  for (int i = 0; i < kLaneSelectionFeatureHistoryStepNum; ++i) {
    if (i < start_idx) {
      agent_center_dist_to_av_left_lane_boundary_vec.push_back(0.0);
      agent_center_dist_to_av_right_lane_boundary_vec.push_back(0.0);
      lane_width.push_back(0.0);
      for (int j = 0; j < kBoxCornerNum; ++j) {
        agent_corners_dist_to_av_left_lane_boundary_vec.push_back(0.0);
        agent_corners_dist_to_av_right_lane_boundary_vec.push_back(0.0);
      }
      continue;
    }

    const auto& agent_center_dist_to_lr_lane_boundary =
        QueryDistanceToLeftAndRightAvLaneBoundary(
            drive_passage, agent_common_feat.sl_pos[2 * i],
            agent_common_feat.sl_pos[2 * i + 1]);

    agent_center_dist_to_av_left_lane_boundary_vec.push_back(
        agent_center_dist_to_lr_lane_boundary.first);
    agent_center_dist_to_av_right_lane_boundary_vec.push_back(
        agent_center_dist_to_lr_lane_boundary.second);
    lane_width.push_back(
        std::abs(agent_center_dist_to_lr_lane_boundary.first -
                 agent_center_dist_to_lr_lane_boundary.second));

    for (int j = 0; j < kBoxCornerNum * kCoordNum; j += 2) {
      const auto& agent_corners_dist_to_lr_lane_boundary =
          QueryDistanceToLeftAndRightAvLaneBoundary(
              drive_passage,
              agent_common_feat.sl_shape[i * kBoxCornerNum * kCoordNum + j],
              agent_common_feat
                  .sl_shape[i * kBoxCornerNum * kCoordNum + j + 1]);
      agent_corners_dist_to_av_left_lane_boundary_vec.push_back(
          agent_corners_dist_to_lr_lane_boundary.first);
      agent_corners_dist_to_av_right_lane_boundary_vec.push_back(
          agent_corners_dist_to_lr_lane_boundary.second);
    }
  }

  // Extract single features
  const int leap_idx =
      start_idx <= kLeapStartPos ? (kLeapStartPos * 2) : (start_idx * 2);
  // Extract l_leap
  const float l_leap =
      agent_common_feat.sl_pos.back() - agent_common_feat.sl_pos[leap_idx + 1];

  // Extract yaw_diff_self_leap
  const Vec2d self_start_tangent(agent_common_feat.yaw[leap_idx],
                                 agent_common_feat.yaw[leap_idx + 1]);
  const Vec2d self_end_tangent(*(agent_common_feat.yaw.end() - 2),
                               *(agent_common_feat.yaw.end() - 1));
  const Vec2d yaw_diff_self_leap = Vec2d::UnitFromAngle(
      self_end_tangent.Angle() - self_start_tangent.Angle());

  // Extract yaw_diff_lane_path_leap
  const Vec2d path_start_tangent(
      agent_common_feat.yaw_diff_lane_path[leap_idx],
      agent_common_feat.yaw_diff_lane_path[leap_idx + 1]);
  const Vec2d path_end_tangent(
      *(agent_common_feat.yaw_diff_lane_path.end() - 2),
      *(agent_common_feat.yaw_diff_lane_path.end() - 1));
  const Vec2d yaw_diff_lane_path_leap = Vec2d::UnitFromAngle(
      path_end_tangent.Angle() - path_start_tangent.Angle());

  const int leap_one_step_idx =
      start_idx <= kLeapStartPos ? kLeapStartPos : start_idx;

  // Extract dist_to_left_lane_boundary_leap
  const float dist_to_left_lane_boundary_leap =
      agent_center_dist_to_av_left_lane_boundary_vec.back() -
      agent_center_dist_to_av_left_lane_boundary_vec[leap_one_step_idx];

  // Extract dist_to_right_lane_boundary_leap
  const float dist_to_right_lane_boundary_leap =
      agent_center_dist_to_av_right_lane_boundary_vec.back() -
      agent_center_dist_to_av_right_lane_boundary_vec[leap_one_step_idx];

  // Extract sl_acc
  std::vector<float> sl_acc_vec;
  sl_acc_vec.reserve(kLaneSelectionFeatureHistoryStepNum * kCoordNum);
  for (int i = 0; i < kLaneSelectionFeatureHistoryStepNum; ++i) {
    if (i < start_idx || start_idx == kLaneSelectionFeatureHistoryStepNum - 1) {
      sl_acc_vec.push_back(0.0);
      sl_acc_vec.push_back(0.0);
      continue;
    }
    if (i == start_idx) {
      sl_acc_vec.push_back((agent_common_feat.sl_speed[(i + 1) * kCoordNum] -
                            agent_common_feat.sl_speed[i * kCoordNum]) /
                           kPredictionTimeStep);
      sl_acc_vec.push_back(
          (agent_common_feat.sl_speed[(i + 1) * kCoordNum + 1] -
           agent_common_feat.sl_speed[i * kCoordNum + 1]) /
          kPredictionTimeStep);
    } else {
      sl_acc_vec.push_back((agent_common_feat.sl_speed[i * kCoordNum] -
                            agent_common_feat.sl_speed[(i - 1) * kCoordNum]) /
                           kPredictionTimeStep);
      sl_acc_vec.push_back(
          (agent_common_feat.sl_speed[i * kCoordNum + 1] -
           agent_common_feat.sl_speed[(i - 1) * kCoordNum + 1]) /
          kPredictionTimeStep);
    }
  }

  return LaneSelectionAgentFeatures{
      .sl_pos = agent_common_feat.sl_pos,
      .sl_speed = agent_common_feat.sl_speed,
      .yaw_diff_lane_path = agent_common_feat.yaw_diff_lane_path,
      .length_width = agent_common_feat.length_width,
      .sl_shape = agent_common_feat.sl_shape,
      .mask = agent_common_feat.mask,
      .type = agent_common_feat.type,

      .l_leap = l_leap,
      .sl_acc = std::move(sl_acc_vec),
      .yaw_diff_self_leap = {static_cast<float>(yaw_diff_self_leap.x()),
                             static_cast<float>(yaw_diff_self_leap.y())},

      .yaw_diff_lane_path_leap =
          {static_cast<float>(yaw_diff_lane_path_leap.x()),
           static_cast<float>(yaw_diff_lane_path_leap.y())},
      .dist_to_left_lane_boundary =
          std::move(agent_center_dist_to_av_left_lane_boundary_vec),
      .dist_to_left_lane_boundary_leap = dist_to_left_lane_boundary_leap,

      .dist_to_right_lane_boundary =
          std::move(agent_center_dist_to_av_right_lane_boundary_vec),
      .dist_to_right_lane_boundary_leap = dist_to_right_lane_boundary_leap,

      .shape_dist_to_left_lane_boundary =
          std::move(agent_corners_dist_to_av_left_lane_boundary_vec),
      .shape_dist_to_right_lane_boundary =
          std::move(agent_corners_dist_to_av_right_lane_boundary_vec),

      .lane_width = std::move(lane_width),
  };
}

std::optional<LaneSelectionObjectFeatures> GetLaneChangeObjectFeatures(
    const LaneSelectionObjectCommonFeatures& agent_feat,
    const LaneSelectionObjectCommonFeatures& obj_feat,
    const planner::DrivePassage& /*drive_passage*/) {
  std::vector<float> rel_sl_pos_agent_vec, rel_sl_speed_agent_vec,
      yaw_diff_agent_vec, rel_dist_agent_vec;

  for (int i = 0; i < kLaneSelectionFeatureHistoryStepNum; ++i) {
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
    const Vec2d obj_sl_pos(obj_feat.sl_pos[i * kCoordNum],
                           obj_feat.sl_pos[i * kCoordNum + 1]);
    const Vec2d agent_sl_pos(agent_feat.sl_pos[i * kCoordNum],
                             agent_feat.sl_pos[i * kCoordNum + 1]);
    const Vec2d rel_sl_pos = obj_sl_pos - agent_sl_pos;
    rel_sl_pos_agent_vec.push_back(rel_sl_pos.x());
    rel_sl_pos_agent_vec.push_back(rel_sl_pos.y());

    // Extract dist from object to agent.
    const Vec2d obj_pos(obj_feat.pos[i * kCoordNum],
                        obj_feat.pos[i * kCoordNum + 1]);
    const Vec2d agent_pos(agent_feat.pos[i * kCoordNum],
                          agent_feat.pos[i * kCoordNum + 1]);
    const Vec2d rel_pos = obj_pos - agent_pos;
    const double rel_dist = rel_pos.norm();
    rel_dist_agent_vec.push_back(rel_dist);

    // Extract relative speed from object to agent.
    const Vec2d obj_sl_speed(obj_feat.sl_speed[i * kCoordNum],
                             obj_feat.sl_speed[i * kCoordNum + 1]);
    const Vec2d agent_sl_speed(agent_feat.sl_speed[i * kCoordNum],
                               agent_feat.sl_speed[i * kCoordNum + 1]);
    const Vec2d rel_sl_speed = obj_sl_speed - agent_sl_speed;
    rel_sl_speed_agent_vec.push_back(rel_sl_speed.x());
    rel_sl_speed_agent_vec.push_back(rel_sl_speed.y());

    // Extract yaw difference from object to agent.
    const Vec2d agent_tangent(agent_feat.yaw[i * kCoordNum],
                              agent_feat.yaw[i * kCoordNum + 1]);
    const Vec2d obj_tangent(obj_feat.yaw[i * kCoordNum],
                            obj_feat.yaw[i * kCoordNum + 1]);
    const Vec2d yaw_diff =
        Vec2d::UnitFromAngle(obj_tangent.Angle() - agent_tangent.Angle());
    yaw_diff_agent_vec.push_back(yaw_diff.x());
    yaw_diff_agent_vec.push_back(yaw_diff.y());
  }

  return LaneSelectionObjectFeatures{
      .sl_pos = obj_feat.sl_pos,
      .sl_speed = obj_feat.sl_speed,
      .yaw_diff_lane_path = obj_feat.yaw_diff_lane_path,
      .length_width = obj_feat.length_width,
      .sl_shape = obj_feat.sl_shape,
      .type = obj_feat.type,

      .rel_sl_pos_agent = std::move(rel_sl_pos_agent_vec),
      .rel_sl_speed_agent = std::move(rel_sl_speed_agent_vec),
      .yaw_diff_agent = std::move(yaw_diff_agent_vec),
      .rel_dist_agent = std::move(rel_dist_agent_vec),
  };
}

std::vector<LaneChangeSLPolylineFeatures> ExtractMapSLFeatureFromPolyline(
    const SampledPolyline& polyline, const planner::DrivePassage& drive_passage,
    const Vec2d& orig, double rot_rad, int max_lane_seg_num) {
  constexpr int kLightNum = 4;

  std::vector<LaneChangeSLPolylineFeatures> feats;
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

std::vector<LaneChangeSLPolylineFeatures> ExtractMapSLFeatureFromPolylines(
    absl::Span<const SampledPolyline* const> polylines,
    const planner::DrivePassage& drive_passage, double front, double back,
    double half_width, const Vec2d& orig, double rot_rad, int max_lane_seg_num,
    int max_feat_num) {
  std::vector<LaneChangeSLPolylineFeatures> combined_feats;
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
  std::vector<LaneChangeSLPolylineFeatures> filtered_feats;
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

std::optional<LaneSelectionObjectCommonFeatures>
GetLaneSelectionObjectCommonFeatures(const ObjectMotionHistory& obj_history,
                                     const planner::DrivePassage& drive_passage,
                                     int history_len) {
  LaneSelectionObjectCommonFeatures feat;
  feat.type =
      static_cast<float>(ObjectTypeToLaneSelectionNetType(obj_history.type));
  const auto& last_state = obj_history.states.back();
  const double length = last_state.bbox.length();
  const double width = last_state.bbox.width();
  feat.length_width = {static_cast<float>(length), static_cast<float>(width)};
  const auto state_num = obj_history.states.size();
  int start_idx = history_len - state_num;
  std::vector<float> pos_vec, sl_pos_vec, yaw_diff_lane_path_vec, sl_shape_vec,
      yaw_vec, mask_vec;
  pos_vec.reserve(history_len * kCoordNum);
  sl_pos_vec.reserve(history_len * kCoordNum);
  yaw_diff_lane_path_vec.reserve(history_len * kCoordNum);
  sl_shape_vec.reserve(history_len * kCoordNum * kBoxCornerNum);
  yaw_vec.reserve(history_len * kCoordNum);
  mask_vec.reserve(history_len);
  for (int i = 0; i < history_len; ++i) {
    if (i < start_idx) {
      const auto empty_vec2d = {0.0, 0.0};
      std::copy(empty_vec2d.begin(), empty_vec2d.end(),
                std::back_inserter(pos_vec));
      std::copy(empty_vec2d.begin(), empty_vec2d.end(),
                std::back_inserter(sl_pos_vec));
      std::copy(empty_vec2d.begin(), empty_vec2d.end(),
                std::back_inserter(yaw_diff_lane_path_vec));
      std::copy(empty_vec2d.begin(), empty_vec2d.end(),
                std::back_inserter(yaw_vec));
      for (int j = 0; j < kBoxCornerNum; ++j) {
        std::copy(empty_vec2d.begin(), empty_vec2d.end(),
                  std::back_inserter(sl_shape_vec));
      }
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

    // Extract yaw
    const auto yaw = state.heading;
    yaw_vec.push_back(fast_math::Cos(yaw));
    yaw_vec.push_back(fast_math::Sin(yaw));

    // Extract yaw diff to drive passage of current lane path
    const auto dp_angle_or = drive_passage.QueryTangentAngleAtS(sl_pos_or->s);
    if (!dp_angle_or.ok()) {
      return std::nullopt;
    }
    const Vec2d yaw_diff_dp = Vec2d::UnitFromAngle(*dp_angle_or - yaw);
    yaw_diff_lane_path_vec.push_back(yaw_diff_dp.x());
    yaw_diff_lane_path_vec.push_back(yaw_diff_dp.y());

    // Extract sl shape
    const Box2d bbox(pos, yaw, length, width);
    for (const auto& pt : bbox.GetCornersCounterClockwise()) {
      const auto sl_shape_pt_or =
          drive_passage.QueryUnboundedFrenetCoordinateAt(pt);
      if (!sl_shape_pt_or.ok() || std::abs(sl_shape_pt_or->s) > kSLCorMax ||
          std::abs(sl_shape_pt_or->l) > kSLCorMax) {
        return std::nullopt;
      }
      sl_shape_vec.push_back(sl_shape_pt_or->s);
      sl_shape_vec.push_back(sl_shape_pt_or->l);
    }
    mask_vec.push_back(1.0);
  }

  if (start_idx < 0) {
    start_idx = 0;
  }
  std::vector<float> sl_speed_vec;
  sl_speed_vec.reserve(history_len * kCoordNum);
  for (int i = 0; i < history_len; ++i) {
    if (i < start_idx || start_idx == history_len - 1) {
      sl_speed_vec.push_back(0.0);
      sl_speed_vec.push_back(0.0);
      continue;
    }
    if (i == start_idx) {
      sl_speed_vec.push_back(
          (sl_pos_vec[(i + 1) * kCoordNum] - sl_pos_vec[i * kCoordNum]) /
          kPredictionTimeStep);
      sl_speed_vec.push_back((sl_pos_vec[(i + 1) * kCoordNum + 1] -
                              sl_pos_vec[i * kCoordNum + 1]) /
                             kPredictionTimeStep);
    } else {
      sl_speed_vec.push_back(
          (sl_pos_vec[i * kCoordNum] - sl_pos_vec[(i - 1) * kCoordNum]) /
          kPredictionTimeStep);
      sl_speed_vec.push_back((sl_pos_vec[i * kCoordNum + 1] -
                              sl_pos_vec[(i - 1) * kCoordNum + 1]) /
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
  feat.yaw_diff_lane_path = std::move(yaw_diff_lane_path_vec);
  feat.sl_shape = std::move(sl_shape_vec);
  feat.yaw = std::move(yaw_vec);
  feat.mask = std::move(mask_vec);
  feat.pos = std::move(pos_vec);

  return feat;
}

std::optional<LaneSelectionFeature> ExtractLaneSelectionFeature(
    const ObjectIDType& agent_id, const ObjectHistorySampler& obj_sampler,
    const planner::DrivePassage& drive_passage,
    MapSampler* const map_sampler_ptr) {
  constexpr int kLaneSelectionConsideredObjsNum = 8;
  auto& map_sampler = *map_sampler_ptr;
  const auto& agent_history =
      obj_sampler.GetResampledMotionHistoryById(agent_id);
  const double heading = agent_history.states.back().heading;
  auto ref_position = agent_history.states.back().pos;
  const double rot_rad = -heading;

  // Extract agent features
  auto agent_common_feat = GetLaneSelectionObjectCommonFeatures(
      agent_history, drive_passage, kLaneSelectionFeatureHistoryStepNum);

  if (agent_common_feat == std::nullopt) {
    return std::nullopt;
  }

  const auto agent_feat = GetLaneSelectionAgentFeatures(
      agent_history, drive_passage, *agent_common_feat);

  // Extract objects features
  const Box2d region_box = GetRegionBox(
      ref_position, heading, kLaneSelectionConfig.scan_box_front_dist,
      kLaneSelectionConfig.scan_box_back_dist,
      kLaneSelectionConfig.scan_box_half_width);
  const auto objects_in_box2d =
      obj_sampler.GetResampledMotionHistoryWithAVInBox2d(
          agent_id, region_box, kLaneSelectionConfig.max_other_objects_num);
  std::vector<LaneSelectionObjectFeatures> objs_features;
  objs_features.reserve(kLaneSelectionConsideredObjsNum);

  // Only consider limited number of objects.
  if (objects_in_box2d.size() > kLaneSelectionConsideredObjsNum) {
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
                                 obj_num < kLaneSelectionConsideredObjsNum;
         ++i) {
      const auto obj_hist = objects_in_box2d_sorted[i];
      const auto obj_abs_features = GetLaneSelectionObjectCommonFeatures(
          obj_hist, drive_passage, kLaneSelectionFeatureHistoryStepNum);

      if (obj_abs_features == std::nullopt) {
        continue;
      }

      const auto obj_features = GetLaneChangeObjectFeatures(
          *agent_common_feat, *obj_abs_features, drive_passage);
      if (obj_features == std::nullopt) {
        continue;
      }
      objs_features.push_back(*obj_features);
      ++obj_num;
    }
  } else {
    for (int i = 0; i < objects_in_box2d.size(); ++i) {
      const auto* obj_hist = objects_in_box2d[i];
      const auto obj_abs_features = GetLaneSelectionObjectCommonFeatures(
          *obj_hist, drive_passage, kLaneSelectionFeatureHistoryStepNum);

      if (obj_abs_features == std::nullopt) {
        continue;
      }

      const auto obj_features = GetLaneChangeObjectFeatures(
          *agent_common_feat, *obj_abs_features, drive_passage);

      if (obj_features == std::nullopt) {
        continue;
      }
      objs_features.push_back(*obj_features);
    }
  }

  // Extract map features.

  // Lane center sl features.
  const auto lcs = map_sampler.GetNearestLaneCenterPolylinesInBox2d(
      region_box, kLaneSelectionConfig.max_lc_num);

  auto lc_sl_feats = ExtractMapSLFeatureFromPolylines(
      lcs, drive_passage, kLaneSelectionConfig.scan_box_front_dist,
      kLaneSelectionConfig.scan_box_back_dist,
      kLaneSelectionConfig.scan_box_half_width, ref_position, rot_rad,
      kLaneSelectionConfig.max_lane_seg_num, kLaneSelectionConfig.max_lc_num);
  // If the extracted feature does not contain any lane center
  // polyline, do not make prediction.
  if (lc_sl_feats.size() == 0) {
    return std::nullopt;
  }

  std::vector<float> lc_masks(kLaneSelectionConfig.max_lc_num, 0.0);
  for (int i = 0; i < lc_sl_feats.size(); ++i) {
    lc_masks[i] = 1.0;
  }

  // Lane boundry sl features
  const auto lbs = map_sampler.GetNearestSolidLaneBoundaryPolylinesInBox2d(
      region_box, kLaneSelectionConfig.max_lb_num);
  auto lb_sl_feats = ExtractMapSLFeatureFromPolylines(
      lbs, drive_passage, kLaneSelectionConfig.scan_box_front_dist,
      kLaneSelectionConfig.scan_box_back_dist,
      kLaneSelectionConfig.scan_box_half_width, ref_position, rot_rad,
      kLaneSelectionConfig.max_lane_seg_num, kLaneSelectionConfig.max_lb_num);

  std::vector<float> lb_masks(kLaneSelectionConfig.max_lb_num, 0.0);
  for (int i = 0; i < lb_sl_feats.size(); ++i) {
    lb_masks[i] = 1.0;
  }

  // Crosswalk sl features
  const auto cws = map_sampler.GetNearestCrosswalkPolylinesInBox2d(
      region_box, kLaneSelectionConfig.max_crosswalk_num);
  auto cw_sl_feats = ExtractMapSLFeatureFromPolylines(
      cws, drive_passage, kLaneSelectionConfig.scan_box_front_dist,
      kLaneSelectionConfig.scan_box_back_dist,
      kLaneSelectionConfig.scan_box_half_width, ref_position, rot_rad,
      kLaneSelectionConfig.max_lane_seg_num,
      kLaneSelectionConfig.max_crosswalk_num);

  std::vector<float> cw_masks(kLaneSelectionConfig.max_crosswalk_num, 0.0);
  for (int i = 0; i < cw_sl_feats.size(); ++i) {
    cw_masks[i] = 1.0;
  }

  // Extract current lane path drive passage xy seg info
  const auto& drive_passage_segs = drive_passage.segments();
  std::vector<float> lane_path_center_xy_seg;
  for (const auto& seg : drive_passage_segs) {
    lane_path_center_xy_seg.push_back(seg.start().x());
    lane_path_center_xy_seg.push_back(seg.start().y());
    lane_path_center_xy_seg.push_back(seg.end().x());
    lane_path_center_xy_seg.push_back(seg.end().y());
  }

  // Extract lane bondaries info
  auto left_right_lane_bondaries = GetLeftRightLaneBoundaries(drive_passage);

  return LaneSelectionFeature{
      .agent_id = agent_id,
      .agent_sl_feature = agent_feat,
      .objs_sl_features = std::move(objs_features),
      .lc_sl_features = std::move(lc_sl_feats),
      .lb_sl_features = std::move(lb_sl_feats),
      .cw_sl_features = std::move(cw_sl_feats),
      .lc_masks = std::move(lc_masks),
      .lb_masks = std::move(lb_masks),
      .cw_masks = std::move(cw_masks),
      .lane_path_center_xy_seg = std::move(lane_path_center_xy_seg),
      .left_lane_path_boundary_sl_pos =
          std::move(left_right_lane_bondaries.left_boundary),
      .right_lane_path_boundary_sl_pos =
          std::move(left_right_lane_bondaries.right_boundary),
  };
}
LaneSelectionDumpedFeatureProto ToLaneSelectionDumpedFeatureProto(
    const LaneSelectionFeature& lane_selection_feature,
    const std::vector<FrenetCoordinate>& agent_gt, double ts) {
  LaneSelectionDumpedFeatureProto lane_selection_dumped_feature_proto;
  lane_selection_dumped_feature_proto.set_agent_id(
      lane_selection_feature.agent_id);
  lane_selection_dumped_feature_proto.set_timestamp(ts);

  // Fill in common agent features.
  auto* mutable_agent_feature =
      lane_selection_dumped_feature_proto.mutable_agent_sl_feature();
  const auto& agent_sl_feature = lane_selection_feature.agent_sl_feature;
  *mutable_agent_feature->mutable_sl_pos() = {agent_sl_feature.sl_pos.begin(),
                                              agent_sl_feature.sl_pos.end()};
  *mutable_agent_feature->mutable_sl_speed() = {
      agent_sl_feature.sl_speed.begin(), agent_sl_feature.sl_speed.end()};
  *mutable_agent_feature->mutable_yaw_diff_lane_path() = {
      agent_sl_feature.yaw_diff_lane_path.begin(),
      agent_sl_feature.yaw_diff_lane_path.end()};
  *mutable_agent_feature->mutable_sl_shape() = {
      agent_sl_feature.sl_shape.begin(), agent_sl_feature.sl_shape.end()};
  *mutable_agent_feature->mutable_length_width() = {
      agent_sl_feature.length_width.begin(),
      agent_sl_feature.length_width.end()};
  mutable_agent_feature->set_type(agent_sl_feature.type);

  // Fill in agent specific features.
  mutable_agent_feature->set_l_leap(agent_sl_feature.l_leap);
  *mutable_agent_feature->mutable_sl_acc() = {agent_sl_feature.sl_acc.begin(),
                                              agent_sl_feature.sl_acc.end()};
  *mutable_agent_feature->mutable_yaw_diff_self_leap() = {
      agent_sl_feature.yaw_diff_self_leap.begin(),
      agent_sl_feature.yaw_diff_self_leap.end()};
  *mutable_agent_feature->mutable_yaw_diff_lane_path_leap() = {
      agent_sl_feature.yaw_diff_lane_path_leap.begin(),
      agent_sl_feature.yaw_diff_lane_path_leap.end()};
  *mutable_agent_feature->mutable_dist_to_left_lane_boundary() = {
      agent_sl_feature.dist_to_left_lane_boundary.begin(),
      agent_sl_feature.dist_to_left_lane_boundary.end()};
  mutable_agent_feature->set_dist_to_left_lane_boundary_leap(
      agent_sl_feature.dist_to_left_lane_boundary_leap);
  *mutable_agent_feature->mutable_dist_to_right_lane_boundary() = {
      agent_sl_feature.dist_to_right_lane_boundary.begin(),
      agent_sl_feature.dist_to_right_lane_boundary.end()};
  mutable_agent_feature->set_dist_to_right_lane_boundary_leap(
      agent_sl_feature.dist_to_right_lane_boundary_leap);
  *mutable_agent_feature->mutable_shape_dist_to_left_lane_boundary() = {
      agent_sl_feature.shape_dist_to_left_lane_boundary.begin(),
      agent_sl_feature.shape_dist_to_left_lane_boundary.end()};
  *mutable_agent_feature->mutable_shape_dist_to_right_lane_boundary() = {
      agent_sl_feature.shape_dist_to_right_lane_boundary.begin(),
      agent_sl_feature.shape_dist_to_right_lane_boundary.end()};
  *mutable_agent_feature->mutable_lane_width() = {
      agent_sl_feature.lane_width.begin(), agent_sl_feature.lane_width.end()};

  // Fill in objects feature.
  const auto& objs_sl_features = lane_selection_feature.objs_sl_features;
  auto* mutable_objects_sl_feature =
      lane_selection_dumped_feature_proto.mutable_objects_sl_feature();
  for (const auto& obj_sl_feature : objs_sl_features) {
    mutable_objects_sl_feature->mutable_sl_pos()->Add(
        obj_sl_feature.sl_pos.begin(), obj_sl_feature.sl_pos.end());
    mutable_objects_sl_feature->mutable_sl_speed()->Add(
        obj_sl_feature.sl_speed.begin(), obj_sl_feature.sl_speed.end());
    mutable_objects_sl_feature->mutable_yaw_diff_lane_path()->Add(
        obj_sl_feature.yaw_diff_lane_path.begin(),
        obj_sl_feature.yaw_diff_lane_path.end());
    mutable_objects_sl_feature->mutable_sl_shape()->Add(
        obj_sl_feature.sl_shape.begin(), obj_sl_feature.sl_shape.end());
    mutable_objects_sl_feature->mutable_length_width()->Add(
        obj_sl_feature.length_width.begin(), obj_sl_feature.length_width.end());

    mutable_objects_sl_feature->mutable_sl_pos_to_agent()->Add(
        obj_sl_feature.rel_sl_pos_agent.begin(),
        obj_sl_feature.rel_sl_pos_agent.end());
    mutable_objects_sl_feature->mutable_sl_speed_to_agent()->Add(
        obj_sl_feature.rel_sl_speed_agent.begin(),
        obj_sl_feature.rel_sl_speed_agent.end());
    mutable_objects_sl_feature->mutable_yaw_diff_to_agent()->Add(
        obj_sl_feature.yaw_diff_agent.begin(),
        obj_sl_feature.yaw_diff_agent.end());
    mutable_objects_sl_feature->mutable_dist_to_agent()->Add(
        obj_sl_feature.rel_dist_agent.begin(),
        obj_sl_feature.rel_dist_agent.end());
    mutable_objects_sl_feature->add_type(obj_sl_feature.type);
  }

  // Fill in map features
  std::vector<const std::vector<LaneChangeSLPolylineFeatures>*> poly_features =
      {&lane_selection_feature.lc_sl_features,
       &lane_selection_feature.lb_sl_features,
       &lane_selection_feature.cw_sl_features};

  auto* mutable_lc_sl_feature =
      lane_selection_dumped_feature_proto.mutable_lane_centers_sl_features();
  auto* mutable_lb_sl_feature =
      lane_selection_dumped_feature_proto.mutable_lane_boundaries_sl_features();
  auto* mutable_cw_sl_feature =
      lane_selection_dumped_feature_proto.mutable_crosswalks_sl_features();

  std::vector<LaneSelectionDumpedFeatureProto::MapDumpedFeature*>
      feature_protos = {mutable_lc_sl_feature, mutable_lb_sl_feature,
                        mutable_cw_sl_feature};

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
      mutable_feat.mutable_yaw_diff_to_agent()->Add(feat.yaw_diff.begin(),
                                                    feat.yaw_diff.end());
      mutable_feat.mutable_type()->Add(feat.type.begin(), feat.type.end());
      mutable_feat.mutable_light()->Add(feat.light.begin(), feat.light.end());
      mutable_feat.mutable_mask()->Add(feat.mask.begin(), feat.mask.end());
    }
  }
  *lane_selection_dumped_feature_proto.mutable_lcs_masks() = {
      lane_selection_feature.lc_masks.begin(),
      lane_selection_feature.lc_masks.end()};
  *lane_selection_dumped_feature_proto.mutable_lbs_masks() = {
      lane_selection_feature.lb_masks.begin(),
      lane_selection_feature.lb_masks.end()};
  *lane_selection_dumped_feature_proto.mutable_cws_masks() = {
      lane_selection_feature.cw_masks.begin(),
      lane_selection_feature.cw_masks.end()};

  // Dump lane_path_boundary_sl_pos
  *lane_selection_dumped_feature_proto
       .mutable_left_lane_path_boundary_sl_pos() = {
      lane_selection_feature.left_lane_path_boundary_sl_pos.begin(),
      lane_selection_feature.left_lane_path_boundary_sl_pos.end()};
  *lane_selection_dumped_feature_proto
       .mutable_right_lane_path_boundary_sl_pos() = {
      lane_selection_feature.right_lane_path_boundary_sl_pos.begin(),
      lane_selection_feature.right_lane_path_boundary_sl_pos.end()};

  // Dump gt:
  for (const auto& gt_point : agent_gt) {
    lane_selection_dumped_feature_proto.mutable_agent_sl_gt()->Add(gt_point.s);
    lane_selection_dumped_feature_proto.mutable_agent_sl_gt()->Add(gt_point.l);
  }

  // Dump dp:
  *lane_selection_dumped_feature_proto.mutable_lane_path_center_xy_seg() = {
      lane_selection_feature.lane_path_center_xy_seg.begin(),
      lane_selection_feature.lane_path_center_xy_seg.end()};

  return lane_selection_dumped_feature_proto;
}
}  // namespace prediction
}  // namespace qcraft
