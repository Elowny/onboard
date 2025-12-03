#ifndef ONBOARD_PLANNER_OBJECT_PATH_BOUNDARY_FILTER_H_
#define ONBOARD_PLANNER_OBJECT_PATH_BOUNDARY_FILTER_H_

#include <algorithm>
#include <memory>
#include <utility>

#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/math/frenet_frame.h"
#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/object/proto/planner_object.pb.h"
#include "onboard/planner/object/trajectory_filter.h"
#include "onboard/prediction/predicted_trajectory.h"

namespace qcraft {
namespace planner {

class PathBoundaryFilter final : public TrajectoryFilter {
 public:
  explicit PathBoundaryFilter(const PathSlBoundary* path_sl,
                              double lateral_distance_buffer)
      : path_sl_(QCHECK_NOTNULL(path_sl)),
        lateral_distance_buffer_(std::max(0.0, lateral_distance_buffer)) {
    FUNC_QTRACE();
    auto ff_or = BuildQtfmEnhancedKdTreeFrenetFrame(
        path_sl_->reference_center_xy_vector(),
        /*down_sample_raw_points=*/false);
    if (ff_or.ok()) {
      path_ff_ = std::make_unique<QtfmEnhancedKdTreeFrenetFrame>(
          std::move(ff_or).value());
    }
  }

  FilterReason::Type Filter(
      const PlannerObject& object,
      const prediction::PredictedTrajectory& pred_traj) const override;

 private:
  const PathSlBoundary* path_sl_;
  double lateral_distance_buffer_ = 0.0;
  std::unique_ptr<QtfmEnhancedKdTreeFrenetFrame> path_ff_;
};
}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_OBJECT_PATH_BOUNDARY_FILTER_H_
