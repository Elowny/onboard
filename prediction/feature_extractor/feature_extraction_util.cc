#include "onboard/prediction/feature_extractor/feature_extraction_util.h"

#include <algorithm>
#include <limits>
#include <ostream>
#include <utility>

#include "absl/types/span.h"

#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/prediction/prediction_message_compressor.h"
#include "onboard/proto/trajectory_point.pb.h"

namespace qcraft {
namespace prediction {
namespace {
std::vector<Vec2d> PickPointsByGreedy(absl::Span<const Vec2d> points,
                                      double radius) {
  std::vector<bool> to_removed(points.size(), false);
  const auto radius_sqr = radius * radius;
  for (int i = 0; i < points.size(); ++i) {
    if (to_removed[i]) {
      continue;
    }
    const auto& cur_pt = points[i];
    for (int j = i + 1; j < points.size(); ++j) {
      if (cur_pt.DistanceSquareTo(points[j]) < radius_sqr) {
        to_removed[j] = true;
      }
    }
  }
  std::vector<Vec2d> filtered_points;
  filtered_points.reserve(points.size());
  for (int i = 0; i < points.size(); ++i) {
    if (!to_removed[i]) {
      filtered_points.push_back(points[i]);
    }
  }
  return filtered_points;
}

}  // namespace
std::vector<PredictedTrajectoryPointProto> AlignPredictedTrajectoryPoints(
    const std::vector<PredictedTrajectoryPointProto>& prev_pts,
    double prev_time, double cur_time, double new_horizon, double dt) {
  if (prev_pts.size() <= 1) {
    return prev_pts;
  }
  std::vector<double> vec_t;
  vec_t.reserve(prev_pts.size());
  for (const auto& prev_pt : prev_pts) {
    vec_t.push_back(prev_pt.t() + prev_time);
  }
  PiecewiseLinearFunction<PredictedTrajectoryPointProto, double,
                          PredictedTrajectoryPointProtoLerper>
      plf_traj(vec_t, std::vector<PredictedTrajectoryPointProto>(
                          prev_pts.begin(), prev_pts.end()));
  std::vector<PredictedTrajectoryPointProto> res;
  double origin_horizon = prev_pts.back().t() - prev_pts.front().t();
  // ensure at least we have current point & next point.
  const int num = std::max(
      static_cast<int>(std::min(new_horizon, origin_horizon) / dt) + 1, 2);
  res.reserve(num);
  double s = 0.0;
  for (int i = 0; i < num; i++) {
    double cur_t = cur_time + i * dt;
    auto new_pt = plf_traj.EvaluateWithExtrapolation(cur_t);
    new_pt.set_t(i * dt);
    if (i != 0) {
      s += (Vec2d(new_pt.pos()) - Vec2d(res.back().pos())).norm();
    }
    new_pt.set_s(s);
    res.push_back(std::move(new_pt));
  }
  return res;
}

std::vector<PredictedTrajectoryPointProto>
ConvertTrajectoryProtoToPredictedTrajectoryPoints(
    const TrajectoryProto& trajectory_proto,
    const VehicleGeometryParamsProto& veh_geom_params) {
  std::vector<PredictedTrajectoryPointProto> res;
  res.reserve(trajectory_proto.trajectory_point_size());
  const double half_length = veh_geom_params.length() * 0.5;
  const double rac_to_center =
      half_length - veh_geom_params.back_edge_to_center();
  for (const auto& pt : trajectory_proto.trajectory_point()) {
    PredictedTrajectoryPointProto predict_pt;
    auto& pos = *predict_pt.mutable_pos();
    const Vec2d tangent = Vec2d::FastUnitFromAngle(pt.path_point().theta());
    const Vec2d center = Vec2d(pt.path_point().x(), pt.path_point().y()) +
                         tangent * rac_to_center;
    pos.set_x(center.x());
    pos.set_y(center.y());
    predict_pt.set_v(pt.v());
    predict_pt.set_a(pt.a());
    predict_pt.set_theta(pt.path_point().theta());
    predict_pt.set_kappa(pt.path_point().kappa());
    predict_pt.set_s(pt.path_point().s());
    predict_pt.set_t(pt.relative_time());
    res.push_back(std::move(predict_pt));
  }
  return res;
}

std::vector<Vec2d> GetExitsOfIntersection(
    const planner::PlannerSemanticMapManager& semantic_map_manager,
    const qcraft::mapping::IntersectionInfo& intersection,
    double reduction_radius) {
  std::vector<Vec2d> targets;
  for (const auto& [lane_id, fractions] : intersection.Lanes()) {
    // Find lanes that have intersection with the intersection polygon
    if (!(fractions.y() >= fractions.x() && fractions.y() < 1.0)) {
      continue;
    }
    const auto* lane_info_ptr =
        semantic_map_manager.FindLaneInfoOrNull(lane_id);
    if (lane_info_ptr == nullptr) continue;
    auto point = lane_info_ptr->LerpPointFromFraction(fractions.y());
    targets.push_back(std::move(point));
  }
  std::sort(targets.begin(), targets.end(),
            [](const auto& a, const auto& b) -> bool { return a.x() < b.x(); });
  return PickPointsByGreedy(targets, reduction_radius);
}

LeftRightLaneBoundaries GetLeftRightLaneBoundaries(
    const planner::DrivePassage& drive_passage) {
  // Extract left and right lane boundaries info
  std::vector<float> left_lane_boundary;
  std::vector<float> right_lane_boundary;
  constexpr double kDrivePassageForwardSRangeInFrenet = 200.0;   // m
  constexpr double kDrivePassageBackwardSRangeInFrenet = -80.0;  // m
  constexpr int kLaneBoundarySegNum =
      static_cast<int>(kDrivePassageForwardSRangeInFrenet +
                       std::abs(kDrivePassageBackwardSRangeInFrenet));

  const auto& drive_passage_segs = drive_passage.segments();
  for (const auto& seg : drive_passage_segs) {
    const auto& start_pt = seg.start();
    const auto& end_pt = seg.end();
    const auto start_pt_sl =
        drive_passage.QueryUnboundedFrenetCoordinateAt(start_pt);
    const auto end_pt_sl =
        drive_passage.QueryUnboundedFrenetCoordinateAt(end_pt);
    // filter the seg out of the range of frenet coord
    if (!start_pt_sl.ok() ||
        start_pt_sl->s < kDrivePassageBackwardSRangeInFrenet) {
      continue;
    }
    if (start_pt_sl.ok() &&
        start_pt_sl->s > kDrivePassageForwardSRangeInFrenet) {
      break;
    }

    const auto boundaries_at_start_s =
        drive_passage.QueryEnclosingLaneBoundariesAtS(start_pt_sl->s);
    const auto boundaries_at_end_s =
        drive_passage.QueryEnclosingLaneBoundariesAtS(end_pt_sl->s);
    double start_pt_dist_to_left_lane_boundary =
        std::numeric_limits<double>::max();
    double start_pt_dist_to_right_lane_boundary =
        std::numeric_limits<double>::max();
    double end_pt_dist_to_left_lane_boundary =
        std::numeric_limits<double>::max();
    double end_pt_dist_to_right_lane_boundary =
        std::numeric_limits<double>::max();
    // kMaxLaneBoundaryOffset is to avoid the offset too large in the junction.
    if (boundaries_at_start_s.left.has_value()) {
      start_pt_dist_to_left_lane_boundary = std::min(
          boundaries_at_start_s.left->lat_offset, kMaxLaneBoundaryOffset);
    } else {
      start_pt_dist_to_left_lane_boundary = kMaxLaneBoundaryOffset;
    }
    if (boundaries_at_end_s.left.has_value()) {
      end_pt_dist_to_left_lane_boundary = std::min(
          boundaries_at_end_s.left->lat_offset, kMaxLaneBoundaryOffset);
    } else {
      end_pt_dist_to_left_lane_boundary = kMaxLaneBoundaryOffset;
    }
    if (boundaries_at_start_s.right.has_value()) {
      start_pt_dist_to_right_lane_boundary = std::max(
          boundaries_at_start_s.right->lat_offset, -kMaxLaneBoundaryOffset);
    } else {
      start_pt_dist_to_right_lane_boundary = -kMaxLaneBoundaryOffset;
    }
    if (boundaries_at_end_s.right.has_value()) {
      end_pt_dist_to_right_lane_boundary = std::max(
          boundaries_at_end_s.right->lat_offset, -kMaxLaneBoundaryOffset);
    } else {
      end_pt_dist_to_right_lane_boundary = -kMaxLaneBoundaryOffset;
    }
    left_lane_boundary.push_back(start_pt_sl->s);
    left_lane_boundary.push_back(start_pt_sl->l +
                                 start_pt_dist_to_left_lane_boundary);
    left_lane_boundary.push_back(end_pt_sl->s);
    left_lane_boundary.push_back(end_pt_sl->l +
                                 end_pt_dist_to_left_lane_boundary);

    right_lane_boundary.push_back(start_pt_sl->s);
    right_lane_boundary.push_back(start_pt_sl->l +
                                  start_pt_dist_to_right_lane_boundary);
    right_lane_boundary.push_back(end_pt_sl->s);
    right_lane_boundary.push_back(end_pt_sl->l +
                                  end_pt_dist_to_right_lane_boundary);
  }

  // Keep the size of lane boundary vector fixed.
  // One segment consists of 4 numbers to denotes two pts.
  if (left_lane_boundary.size() < kLaneBoundarySegNum * 4) {
    std::vector<float> zero_4d(
        kLaneBoundarySegNum * 4 - left_lane_boundary.size(), 0.0);
    std::copy(zero_4d.begin(), zero_4d.end(),
              std::back_inserter(left_lane_boundary));
  } else if (left_lane_boundary.size() > kLaneBoundarySegNum * 4) {
    left_lane_boundary.resize(kLaneBoundarySegNum * 4);
  }
  if (right_lane_boundary.size() < kLaneBoundarySegNum * 4) {
    std::vector<float> zero_4d(
        kLaneBoundarySegNum * 4 - right_lane_boundary.size(), 0.0);
    std::copy(zero_4d.begin(), zero_4d.end(),
              std::back_inserter(right_lane_boundary));
  } else if (right_lane_boundary.size() > kLaneBoundarySegNum * 4) {
    right_lane_boundary.resize(kLaneBoundarySegNum * 4);
  }

  return LeftRightLaneBoundaries{
      .left_boundary = std::move(left_lane_boundary),
      .right_boundary = std::move(right_lane_boundary),
  };
}

}  // namespace prediction
}  // namespace qcraft
