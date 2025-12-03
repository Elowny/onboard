#include "onboard/planner/router/preprocess/future_pos_estimation.h"

#include <optional>
#include <string>

#include "absl/status/status.h"
#include "absl/strings/str_cat.h"

#include "onboard/math/util.h"
#include "onboard/proto/trajectory_point.pb.h"
namespace qcraft::planner::route {

absl::StatusOr<Vec2d> InferPosFromTrajectory(
    const TrajectoryProto& trajectory_proto, double expect_future_time_secs,
    double max_infer_interval_secs) {
  const double trajectory_start_seconds =
      trajectory_proto.trajectory_start_timestamp();
  if (expect_future_time_secs - trajectory_start_seconds >
      max_infer_interval_secs) {
    return absl::InvalidArgumentError(absl::StrCat(
        "The trajectory_proto is stale or the expect future time is too long. "
        "ignore it. expected:",
        expect_future_time_secs,
        ", trajectory_start_timestamp:", trajectory_start_seconds,
        ", diff:", expect_future_time_secs - trajectory_start_seconds,
        ", max_infer_interval_secs:", max_infer_interval_secs));
  }
  std::optional<Vec2d> lower_bound_point;
  double t1;
  std::optional<Vec2d> upper_bound_point;
  double t2;
  for (const auto& trajectory_point : trajectory_proto.trajectory_point()) {
    const auto& path_point = trajectory_point.path_point();
    const auto ts_secs =
        trajectory_start_seconds + trajectory_point.relative_time();
    if (ts_secs <= expect_future_time_secs) {
      lower_bound_point = Vec2d{path_point.x(), path_point.y()};
      t1 = ts_secs;
    } else {
      upper_bound_point = Vec2d{path_point.x(), path_point.y()};
      t2 = ts_secs;
      break;
    }
  }
  if (lower_bound_point.has_value() && upper_bound_point.has_value()) {
    const double anchor_smooth_x =
        Lerp(lower_bound_point->x(), upper_bound_point->x(),
             (expect_future_time_secs - t1) / (t2 - t1));
    const double anchor_smooth_y =
        Lerp(lower_bound_point->y(), upper_bound_point->y(),
             (expect_future_time_secs - t1) / (t2 - t1));
    return Vec2d(anchor_smooth_x, anchor_smooth_y);
  } else if (lower_bound_point.has_value()) {
    return *lower_bound_point;
  }
  return absl::NotFoundError("Cannot infer the target future point.");
}
}  // namespace qcraft::planner::route
