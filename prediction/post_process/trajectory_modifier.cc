#include "onboard/prediction/post_process/trajectory_modifier.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type

#include <algorithm>
#include <cmath>
#include <limits>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/types/span.h"
#include "glog/logging.h"

#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/fast_math.h"
#include "onboard/math/geometry/segment2d.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/prediction/prediction_defs.h"
#include "onboard/prediction/prediction_flags.h"
#include "onboard/proto/perception/fusion/object.pb.h"

namespace qcraft {
namespace prediction {

namespace {

// Used for cutting off prediction traj by curb
constexpr int kFindCurbPointStep = 5;
constexpr double kCurbDirectionDiffDeg = 150.0;  // deg.

struct TrajCurbInvasionInfo {
  int index;
  bool out;
  mapping::ElementId curb_id;
  Segment2d curb_seg;
  std::string DebugString() const {
    return absl::StrFormat("Traj index %d, going out of curb: %d, curb_id: %d.",
                           index, out, curb_id);
  }
};

bool ComputeOutCurbIntegral(
    absl::Span<const PredictedTrajectoryPoint> traj_points,
    const Segment2d& curb_seg, int start_idx, int end_idx, double max_threshold,
    int* cutoff_idx) {
  double sum = 0.0;
  for (int i = start_idx; i <= end_idx - 1; ++i) {
    const auto& point = traj_points[i];
    const auto& next_point = traj_points[i + 1];
    const Vec2d dir = (next_point.pos() - point.pos()).normalized();
    const double inv_v = 1.0 / std::max<double>(1e-10, std::fabs(point.v()));
    sum += curb_seg.unit_direction().CrossProd(dir * inv_v);
    if (sum > max_threshold) {
      *cutoff_idx = i;
      return true;
    }
  }
  return false;
}

void ExtendTrajByTimeHorizon(
    absl::Span<const PredictedTrajectoryPoint> traj_points,
    double nominal_brake, double kept_horizon, int* cutoff_idx) {
  const double inv_nominal_brake = std::fabs(1.0 / nominal_brake);
  const double min_nominal_brake_dis =
      std::pow(traj_points.front().v(), 2) * 0.5 * inv_nominal_brake;
  const double cutoff_dis = traj_points[*cutoff_idx].s();
  // Cutoff means planner won't see this object after cutoff idx. Assume object
  // is the reactor, if it cannot stop before intended cutoff position, extend
  // to max deceleration position or the min kept horizon.
  if (min_nominal_brake_dis > cutoff_dis) {
    const double final_cutoff_horizon = std::max<double>(
        kept_horizon, traj_points.front().v() * inv_nominal_brake);
    *cutoff_idx = FloorToInt(final_cutoff_horizon / kPredictionTimeStep);
  }
}

void DecideTrajCutoffByOutThreshold(
    absl::Span<const PredictedTrajectoryPoint> traj_points,
    absl::Span<const TrajCurbInvasionInfo> infos, double max_threshold,
    int start, int end, bool* updated, int* cutoff_idx) {
  if (*updated || infos.empty() || traj_points.empty()) {
    return;
  }
  end = std::min<int>(infos.size() - 1, end);
  if (end - start < 0) return;  // No more range to consider.
  if (infos[start].out) {
    int cutoff_idx_temp = traj_points.size();
    int end_index =
        end == start ? traj_points.size() - 1 : infos[end].index - 1;
    if (ComputeOutCurbIntegral(traj_points, infos[start].curb_seg,
                               infos[start].index, end_index, max_threshold,
                               &cutoff_idx_temp)) {
      *updated = true;
      *cutoff_idx = std::min<int>(*cutoff_idx, cutoff_idx_temp);
      return;
    }
  }
  // start.out not true or did not update cutoff idx, continue to search.
  DecideTrajCutoffByOutThreshold(traj_points, infos, max_threshold, start + 1,
                                 end + 1, updated, cutoff_idx);
  return;
}

// Cutoff trajectory by (in, out) pair.
void DecideTrajCutoffBySemantics(
    absl::Span<const PredictedTrajectoryPoint> traj_points,
    absl::Span<const TrajCurbInvasionInfo> infos, double max_threshold,
    int start, int end, bool* cutoff, int* cutoff_idx) {
  if (*cutoff || infos.empty()) {
    return;
  }
  end = std::min<int>(end, infos.size() - 1);
  if (end - start <= 0) return;  // No more to consider.
  if (infos[start].out || !infos[end].out) {
    DecideTrajCutoffBySemantics(traj_points, infos, max_threshold, start + 1,
                                end + 1, cutoff, cutoff_idx);
    return;
  }
  const double cos_angle_diff =
      fast_math::Cos(kCurbDirectionDiffDeg / 180.0 * M_PI);
  const auto& in_curb_dir = infos[start].curb_seg.unit_direction();
  const auto& out_curb_dir = infos[end].curb_seg.unit_direction();
  if (in_curb_dir.Dot(out_curb_dir) < cos_angle_diff) {
    // In and out consecutively two nearly opposite curb segment, might be
    // double solid line or high impassable curbs. Cutoff at end index, set
    // status to can_stop.
    *cutoff_idx = infos[end].index;
    *cutoff = true;
    return;
  }
  // In and out some curb consecutively but the crub segment has small heading
  // diff. Leave them, consider next two pair.s
  DecideTrajCutoffBySemantics(traj_points, infos, max_threshold, start + 2,
                              end + 2, cutoff, cutoff_idx);
  return;
}

bool IsInsideCurb(const Segment2d& curb_seg, const Vec2d& pos) {
  const Vec2d v1 = curb_seg.end() - curb_seg.start();
  const Vec2d v2 = pos - curb_seg.start();
  return NormalizeAngle2D(v1, v2) < 0.0;
}

std::vector<TrajCurbInvasionInfo> FindTrajCurbInvasionInfo(
    absl::Span<const PredictedTrajectoryPoint> traj_points,
    const planner::PlannerSemanticMapManager& smm,
    const ObjectProto& object_proto,
    mapping::LaneBoundaryProto::Type lane_boundary_type) {
  std::vector<TrajCurbInvasionInfo> infos;
  const int point_num = static_cast<int>(traj_points.size());
  int point_idx = 0;
  for (int i = 0; i + kFindCurbPointStep < point_num; i += kFindCurbPointStep) {
    point_idx += kFindCurbPointStep;
    const auto& search_cur_pos = traj_points[i].pos();
    const auto& search_next_pos = traj_points[i + kFindCurbPointStep].pos();
    const Segment2d search_seg(search_cur_pos, search_next_pos);
    const auto curb_segments_info =
        smm.GetSpecifiedTypeBoundarySegmentInfosAtLevel(
            smm.GetLevel(), search_cur_pos, search_seg.length(),
            lane_boundary_type);
    if (curb_segments_info.empty()) {
      continue;
    }
    for (int j = 0; j < kFindCurbPointStep; ++j) {
      const auto& cur_pos = traj_points[i + j].pos();
      const auto& next_pos = traj_points[i + j + 1].pos();
      const Segment2d check_seg(cur_pos, next_pos);
      for (const auto& curb_segment_info : curb_segments_info) {
        const auto& curb_seg = curb_segment_info.segment;
        if (curb_seg.HasIntersect(check_seg)) {
          // Cur pos outside curb and cur->next has intersection with curb ->
          // direction is into curb.
          infos.push_back(TrajCurbInvasionInfo({
              .index = i + j,
              .out = IsInsideCurb(curb_seg, cur_pos),
              .curb_id = curb_segment_info.lane_boundary_id,
              .curb_seg = curb_seg,
          }));
          break;
        }
      }
    }
  }
  for (int i = point_idx; i + 1 < point_num; ++i) {
    const auto& cur_pos = traj_points[i].pos();
    const auto& next_pos = traj_points[i + 1].pos();
    const Segment2d check_seg(cur_pos, next_pos);
    const auto curb_segments_info =
        smm.GetSpecifiedTypeBoundarySegmentInfosAtLevel(
            smm.GetLevel(), cur_pos, check_seg.length(), lane_boundary_type);
    if (curb_segments_info.empty()) {
      continue;
    }
    for (const auto& curb_segment_info : curb_segments_info) {
      const auto& curb_seg = curb_segment_info.segment;
      if (curb_seg.HasIntersect(check_seg)) {
        // Cur pos outside curb and cur->next has intersection with curb ->
        // direction is into curb.
        infos.push_back(TrajCurbInvasionInfo({
            .index = i,
            .out = IsInsideCurb(curb_seg, cur_pos),
            .curb_id = curb_segment_info.lane_boundary_id,
            .curb_seg = curb_seg,
        }));
        break;
      }
    }
  }

  if (!infos.empty()) {
    VLOG(5) << "find traj curb invasion info for :" << object_proto.id();
    for (const auto& info : infos) {
      VLOG(5) << info.DebugString();
    }
  }
  return infos;
}

inline void TruncateTrajectoryBySize(
    int new_size, std::vector<PredictedTrajectoryPoint>* mutable_points) {
  mutable_points->resize(
      std::min(static_cast<int>(mutable_points->size()), new_size));
}

}  // namespace

void CutoffTrajByLaneBoundary(
    const planner::PlannerSemanticMapManager& smm,
    const ObjectProto& object_proto,
    const ConflictResolutionConfigProto::ObjectConflictResolverConfig&
        object_config,
    mapping::LaneBoundaryProto::Type lane_boundary_type,
    PredictedTrajectory* traj) {
  if (object_proto.type() == ObjectType::OT_PEDESTRIAN &&
      !FLAGS_prediction_enable_ped_traj_cutoff_at_curb) {
    // Ped traj curb cutoff controlled by flag.
    return;
  }
  const auto invasion_infos = FindTrajCurbInvasionInfo(
      traj->points(), smm, object_proto, lane_boundary_type);
  if (invasion_infos.empty()) return;
  if (object_proto.type() == ObjectType::OT_PEDESTRIAN) {
    constexpr int kMinCutOffIndex = 30;
    for (const auto& info : invasion_infos) {
      if (info.index < kMinCutOffIndex) {
        return;
      }
    }
  }
  int cut_off_idx = traj->points().size();
  if (lane_boundary_type == mapping::LaneBoundaryProto::CURB) {
    bool cutoff = false;
    DecideTrajCutoffBySemantics(
        traj->points(), invasion_infos,
        /*max_threshold=*/std::numeric_limits<double>::max(), 0, 1, &cutoff,
        &cut_off_idx);
    const auto hard_cut_off_idx = cut_off_idx;
    bool updated = false;
    if (object_config.check_out_curb()) {
      DecideTrajCutoffByOutThreshold(traj->points(), invasion_infos,
                                     object_config.out_curb_max_threshold(), 0,
                                     1, &updated, &cut_off_idx);
    }
    if (!cutoff && !updated) {
      return;
    }
    VLOG(5) << "Want to cutoff at " << cut_off_idx;
    ExtendTrajByTimeHorizon(traj->points(), object_config.curb_nominal_brake(),
                            object_config.curb_kept_horizon(), &cut_off_idx);
    cut_off_idx = std::min<int>(hard_cut_off_idx, cut_off_idx);
  } else {
    cut_off_idx = invasion_infos[0].index;
  }
  VLOG(5) << "Final cutoff at " << cut_off_idx;
  if (cut_off_idx < traj->points().size()) {
    auto* mutable_points = traj->mutable_points();
    TruncateTrajectoryBySize(cut_off_idx, mutable_points);
    const int final_len =
        FloorToInt(object_config.curb_kept_horizon() / kPredictionTimeStep);
    if (mutable_points->back().v() <
            object_config.curb_cutoff_to_static_max_speed() &&
        cut_off_idx < final_len) {
      // Repeat last point before collision when object has a low speed.
      // Patch up to kept_horizon seconds
      const auto& last_pt = mutable_points->back();
      for (int i = cut_off_idx; i < final_len; ++i) {
        auto new_pt = last_pt;
        new_pt.set_t(last_pt.t() + (i - cut_off_idx + 1) * kPredictionTimeStep);
        new_pt.set_v(0.0);
        new_pt.set_a(0.0);
        mutable_points->push_back(std::move(new_pt));
      }
      traj->set_annotation(
          absl::StrCat(traj->annotation(), ", curb stop at ", cut_off_idx));
    } else {
      traj->set_annotation(
          absl::StrCat(traj->annotation(), ", curb cutoff at ", cut_off_idx));
    }
  }
}

void CutoffTrajByCurb(
    const planner::PlannerSemanticMapManager& smm,
    const ObjectProto& object_proto,
    const ConflictResolutionConfigProto::ObjectConflictResolverConfig&
        object_config,
    PredictedTrajectory* traj) {
  CutoffTrajByLaneBoundary(smm, object_proto, object_config,
                           mapping::LaneBoundaryProto::CURB, traj);
}

}  // namespace prediction
}  // namespace qcraft
