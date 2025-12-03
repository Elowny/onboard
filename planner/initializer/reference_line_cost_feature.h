#ifndef ONBOARD_PLANNER_INITIALIZER_REFERENCE_LINE_COST_FEATURE_H_
#define ONBOARD_PLANNER_INITIALIZER_REFERENCE_LINE_COST_FEATURE_H_

#include <algorithm>
#include <string>
#include <vector>

#include "absl/types/span.h"

#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/planner/initializer/cost_feature.h"
#include "onboard/planner/object/spacetime_object_state.h"
#include "onboard/planner/object/spacetime_object_trajectory.h"
#include "onboard/planner/object/spacetime_planner_object_trajectories.h"
#include "onboard/planner/router/drive_passage.h"

namespace qcraft::planner {

// Major objective of reference line searcher is to search for a reference line
// which avoids stationary objects so during motion expansion, more time
// can be used to expand useful geometry edges.

class RefLineStationaryObjectFeatureCost : public FeatureCost {
 public:
  explicit RefLineStationaryObjectFeatureCost(
      const SpacetimePlannerObjectTrajectories* st_planner_object_traj,
      const DrivePassage* passage)
      : FeatureCost("ref_line_stationary_object"), passage_(passage) {
    const auto& st_planner_trajs = st_planner_object_traj->trajectories;
    for (const auto& traj : st_planner_trajs) {
      if (traj.is_stationary()) {
        stationary_obj_states_.push_back(traj.states().front());
      }
    }
  }

  void ComputeCost(const GeometryEdgeInfo& edge_info,
                   absl::Span<double> cost) const override;

 private:
  static constexpr int sample_step_ = 1;
  std::vector<SpacetimeObjectState> stationary_obj_states_;
  [[maybe_unused]] const DrivePassage* passage_;
};

class RefLineProgressFeatureCost : public FeatureCost {
 public:
  explicit RefLineProgressFeatureCost(double geom_graph_max_accum_s,
                                      const PathSlBoundary* path_sl_boundary)
      : FeatureCost("ref_line_progress"),
        geom_graph_max_accum_s_(geom_graph_max_accum_s),
        path_sl_boundary_(path_sl_boundary) {}

  void ComputeCost(const GeometryEdgeInfo& edge_info,
                   absl::Span<double> cost) const override;

 private:
  double geom_graph_max_accum_s_;
  const PathSlBoundary* path_sl_boundary_;
};

class RefLinePathBoundaryFeatureCost : public FeatureCost {
 public:
  explicit RefLinePathBoundaryFeatureCost(
      const PathSlBoundary* path_sl_boundary)
      : FeatureCost("ref_line_path_boundary"),
        path_sl_boundary_(path_sl_boundary) {}

  void ComputeCost(const GeometryEdgeInfo& edge_info,
                   absl::Span<double> cost) const override;

 private:
  const PathSlBoundary* path_sl_boundary_;
};

class RefLineCurvatureFeatureCost : public FeatureCost {
 public:
  explicit RefLineCurvatureFeatureCost(double relaxed_center_max_curvature)
      : FeatureCost("ref_line_curvature"),
        inv_relaxed_center_max_curvature_(1.0 / relaxed_center_max_curvature) {}

  void ComputeCost(const GeometryEdgeInfo& edge_info,
                   absl::Span<double> cost) const override;

 private:
  double inv_relaxed_center_max_curvature_;
};

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_INITIALIZER_REFERENCE_LINE_COST_FEATURE_H_
