#ifndef ONBOARD_PLANNER_COMMON_LANE_PATH_INFO_H_
#define ONBOARD_PLANNER_COMMON_LANE_PATH_INFO_H_

#include <float.h>

#include <vector>

#include "onboard/container/strong_int.h"
#include "onboard/maps/lane_path.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/vec.h"
#include "onboard/planner/planner_semantic_map_manager.h"

namespace qcraft::planner {

class LanePathInfo {
 public:
  LanePathInfo() {}
  LanePathInfo(mapping::LanePath lane_path, double len_along_route,
               double path_cost, const PlannerSemanticMapManager& psmm);

  bool empty() const { return lane_path_.IsEmpty(); }
  const mapping::LanePath& lane_path() const { return lane_path_; }
  mapping::ElementId start_lane_id() const {
    return lane_path_.front().lane_id();
  }

  // If a routed lane change must be executed on this lane path, length
  // decreases by a default lane change distance.
  //
  // [— length_along_route —]
  // ----------------------------------------->
  //
  // [————— length_along_route —————]
  // ----------------------------------------->
  //
  // ------------------------------------------
  //                                          |
  //                                          |
  //                                      destination
  double length_along_route() const { return length_along_route_; }
  double max_reach_length() const { return max_reach_length_; }
  double path_cost() const { return path_cost_; }
  bool is_solid_lane_change() const { return is_solid_lane_change_; }
  mapping::ElementId first_fork_lk_lane_id() const {
    return first_fork_lk_lane_id_;
  }

  void set_length_along_route(double length_along_route) {
    length_along_route_ = length_along_route;
  }
  void set_max_reach_length(double reach_length) {
    max_reach_length_ = reach_length;
  }
  void set_is_solid_lane_change(bool is_solid_lane_change) {
    is_solid_lane_change_ = is_solid_lane_change;
  }
  void set_first_fork_lk_lane_id(mapping::ElementId first_fork_lk_lane_id) {
    first_fork_lk_lane_id_ = first_fork_lk_lane_id;
  }

  FrenetCoordinate ProjectionSL(const Vec2d& xy) const;
  FrenetCoordinate ProjectionSLInRange(const Vec2d& xy, double start_s,
                                       double end_s) const;
  Vec2d ProjectionXY(FrenetCoordinate sl) const;

 private:
  mapping::LanePath lane_path_;
  double length_along_route_ = 0.0;
  double max_reach_length_ = 0.0;
  double path_cost_ = DBL_MAX;
  mapping::ElementId first_fork_lk_lane_id_ = mapping::kInvalidElementId;

  // Projection system
  std::vector<Vec2d> anchor_points_;
  std::vector<double> anchor_s_;
  std::vector<Vec2d> tangents_;
  std::vector<double> segment_len_inv_;

  bool is_solid_lane_change_ = false;
  // TODO(weijun): potentially add lane path boundary here.
  // speed_limit
};

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_COMMON_LANE_PATH_INFO_H_
