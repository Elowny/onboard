#include "onboard/planner/speed/st_close_trajectory.h"

#include <algorithm>
#include <iterator>
#include <utility>

#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/math/util.h"

namespace qcraft::planner {

namespace {
using StNearestPoint = StCloseTrajectory::StNearestPoint;
}  // namespace

StCloseTrajectory::StCloseTrajectory(
    std::vector<StNearestPoint> st_nearest_points, std::string id,
    std::string traj_id, std::string object_id,
    StBoundaryProto::ObjectType object_type, double probability,
    bool is_stationary)
    : st_nearest_points_(std::move(st_nearest_points)),
      id_(std::move(id)),
      traj_id_(std::move(traj_id)),
      object_id_(std::move(object_id)),
      object_type_(object_type),
      is_stationary_(is_stationary),
      probability_(probability) {
  QCHECK(!st_nearest_points_.empty());
  Init();
}

std::optional<StNearestPoint> StCloseTrajectory::GetNearestPointByTime(
    double time) const {
  if (!InRange(time, min_t_, max_t_)) return std::nullopt;
  const auto it =
      std::lower_bound(st_nearest_points_.begin(), st_nearest_points_.end(),
                       time, [](const auto& pt, double t) { return pt.t < t; });
  if (it == st_nearest_points_.begin()) {
    return st_nearest_points_.front();
  } else if (it == st_nearest_points_.end()) {
    return st_nearest_points_.back();
  }
  const int index = std::distance(st_nearest_points_.begin(), it);
  QCHECK_GT(index, 0);
  const int left = index - 1;
  const int right = index;
  const double alpha =
      (time - st_nearest_points_[left].t) /
      (st_nearest_points_[right].t - st_nearest_points_[left].t);
  const auto& left_pt = st_nearest_points_[left];
  const auto& right_pt = st_nearest_points_[right];
  return StNearestPoint{
      .s = Lerp(left_pt.s, right_pt.s, alpha),
      .t = time,
      .v = Lerp(left_pt.v, right_pt.v, alpha),
      .lat_dist = Lerp(left_pt.lat_dist, right_pt.lat_dist, alpha),
      .obj_idx = left_pt.obj_idx};
}

void StCloseTrajectory::Init() {
  min_t_ = st_nearest_points_.front().t;
  max_t_ = st_nearest_points_.back().t;
  for (const auto& pt : st_nearest_points_) {
    min_s_ = std::min(min_s_, pt.s);
    max_s_ = std::max(max_s_, pt.s);
  }
}

}  // namespace qcraft::planner
