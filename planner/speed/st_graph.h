#ifndef ONBOARD_PLANNER_SPEED_ST_GRAPH_H_
#define ONBOARD_PLANNER_SPEED_ST_GRAPH_H_

#include <limits>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/synchronization/mutex.h"
#include "absl/types/span.h"
#include "gflags/gflags.h"

#include "onboard/async/thread_pool.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/geometry/segment2d.h"
#include "onboard/math/segment_matcher/segment_matcher_kdtree.h"
#include "onboard/math/vec.h"
#include "onboard/planner/common/discretized_path.h"
#include "onboard/planner/common/path_approx.h"
#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/planner/common/vehicle_shape.h"
#include "onboard/planner/decision/constraint_manager.h"
#include "onboard/planner/decision/proto/constraint.pb.h"
#include "onboard/planner/object/spacetime_object_state.h"
#include "onboard/planner/object/spacetime_object_trajectory.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/speed/proto/speed_finder.pb.h"
#include "onboard/planner/speed/proto/speed_finder_params.pb.h"
#include "onboard/planner/speed/st_boundary.h"
#include "onboard/planner/speed/st_boundary_with_decision.h"
#include "onboard/planner/speed/st_close_trajectory.h"
#include "onboard/planner/speed/st_graph_defs.h"
#include "onboard/proto/vehicle.pb.h"

DECLARE_bool(planner_use_path_approx_based_st_mapping);

namespace qcraft::planner {

class StGraph {
 public:
  struct AgentNearestPoint {
    double ra_s = 0.0;
    double ra_heading = 0.0;
    double t = 0.0;
    double obj_v = 0.0;
    double obj_theta = 0.0;  // Speed direction.
    double lat_dist = 0.0;
    int obj_idx = 0;
  };
  StGraph(const DiscretizedPath* path_points, int traj_steps,
          double plan_start_v, double max_decel,
          const VehicleGeometryParamsProto* vehicle_geo_params,
          const SpeedFinderParamsProto::StGraphParamsProto* st_graph_params,
          const std::vector<VehicleShapeBasePtr>* av_shape_on_path_points,
          const SegmentMatcherKdtree* path_kd_tree,
          const PathApprox* path_approx,
          const PathApprox* path_approx_for_mirrors);

  std::vector<StBoundaryRef> GetStBoundaries(
      const SpacetimeTrajectoryManager& traj_mgr,
      const std::map<std::string, ConstraintProto::LeadingObjectProto>&
          leading_objs,
      const ConstraintManager& constraint_mgr,
      const PlannerSemanticMapManager* psman_mgr,
      const DrivePassage* drive_passage, const PathSlBoundary* path_sl_boundary,
      ThreadPool* thread_pool);

  // For moving object, this function will only check and return the first
  // point(which is the object current state) if it is within the
  // slow_down_radius.
  std::vector<CloseSpaceTimeObject> GetCloseSpaceTimeObjects(
      absl::Span<const StBoundaryWithDecision> st_boundaries_with_decision,
      absl::Span<const SpacetimeObjectTrajectory* const> spacetime_object_trajs,
      double slow_down_radius) const;

  std::vector<StBoundaryRef> MapMovingSpacetimeObject(
      const SpacetimeObjectTrajectory& spacetime_object,
      bool generate_lane_change_gap, bool calc_moving_close_traj) const;

  /**
   * @brief Return a vector which contains the distance info on each path
   * point before collision with impassable boundaries.
   */
  const std::vector<DistanceInfo>& distance_info_to_impassable_boundaries()
      const {
    return distance_info_to_impassable_boundaries_;
  }

  const std::vector<StCloseTrajectory>& moving_close_trajs() const {
    return moving_close_trajs_;
  }

  StBoundaryRef MapPathStopLine(
      const ConstraintProto::PathStopLineProto& stop_line) const;

 private:
  std::vector<StBoundaryRef> MapStationarySpacetimeObjects(
      absl::Span<const SpacetimeObjectTrajectory* const>
          stationary_spacetime_objs,
      const std::map<std::string, ConstraintProto::LeadingObjectProto>&
          leading_objs) const;

  StBoundaryRef MapStopLine(
      const ConstraintProto::StopLineProto& stop_line) const;

  StBoundaryRef MapNearestImpassableBoundary(
      const PlannerSemanticMapManager& psman_mgr,
      const DrivePassage* drive_passage);

  StBoundaryRef MapNearestPathBoundary(
      const DrivePassage& drive_passage,
      const PathSlBoundary& path_sl_boundary) const;

  // Generate possibly multiple st-boundaries for a moving object.
  std::vector<StBoundaryPoints> GetMovingObjStBoundaryPoints(
      const SpacetimeObjectTrajectory& spacetime_object,
      bool generate_lane_change_gap, bool calc_moving_close_traj) const;

  // Generate single st-boundary for a stationary object.
  std::optional<StBoundaryPoints> GetStationaryObjStBoundaryPoints(
      const SpacetimeObjectTrajectory& spacetime_object) const;

  bool FindOverlapRangeOrNearestPoint(
      const Vec2d& search_point, double search_radius,
      double search_radius_for_mirrors, const SpacetimeObjectState& obj_state,
      int obj_idx, bool consider_mirrors, double lat_buffer, double lon_buffer,
      int* low_idx, int* high_idx,
      std::vector<AgentNearestPoint>* agent_nearest_points) const;

  void GenerateLatInflatedStBoundary(
      const SpacetimeObjectTrajectory& spacetime_object,
      StBoundaryProto::ProtectionType protection_type, double lat_buffer,
      std::vector<StBoundaryPoints>* st_boundaries_points) const;
  void GenerateProtectiveStBoundaries(
      const SpacetimeObjectTrajectory& spacetime_object,
      absl::Span<const StGraph::AgentNearestPoint> agent_nearest_points,
      const Vec2d& obj_center, bool generate_lane_change_gap,
      std::vector<StBoundaryPoints>* st_boundaries_points) const;

  std::optional<std::pair<int, int>> FindSegment2dOverlapRange(
      const Segment2d& segment) const;

  std::optional<std::pair<int, int>> FindStopLineOverlapRange(
      const ConstraintProto::StopLineProto& stop_line) const;

  std::vector<StBoundaryRef> MapLeadingSpacetimeObjects(
      const SpacetimeTrajectoryManager& traj_mgr,
      const std::map<std::string, ConstraintProto::LeadingObjectProto>&
          leading_objs,
      const DrivePassage& drive_passage,
      const PathSlBoundary& path_sl_boundary) const;

  std::optional<int> FindLeadingObjectLowerIndex(
      const Polygon2d& obj_shape, const DrivePassage& drive_passage,
      const PathSlBoundary& path_sl_boundary) const;

  bool GetStDistancePointInfo(const SpacetimeObjectState& state,
                              double slow_down_radius,
                              StDistancePoint* st_distance_point) const;

 private:
  double total_plan_time_ = 0.0;
  double plan_start_v_ = 0.0;
  double max_decel_ = 0.0;
  const DiscretizedPath* path_points_;
  // The SDC shapes on path points.
  const std::vector<VehicleShapeBasePtr>* av_shape_on_path_points_;
  const SegmentMatcherKdtree* path_kd_tree_;
  double ego_radius_ = 0.0;
  double ego_radius_for_mirrors_ = 0.0;
  double min_mirror_height_avg_ = std::numeric_limits<double>::infinity();
  double max_mirror_height_avg_ = -std::numeric_limits<double>::infinity();

  const VehicleGeometryParamsProto* vehicle_geo_params_;
  const SpeedFinderParamsProto::StGraphParamsProto* st_graph_params_;
  std::vector<DistanceInfo> distance_info_to_impassable_boundaries_;
  const PathApprox* path_approx_;
  const PathApprox* path_approx_for_mirrors_;
  mutable std::vector<StCloseTrajectory> moving_close_trajs_;
  mutable absl::Mutex mutex_;
};

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_SPEED_ST_GRAPH_H_
