#ifndef ONBOARD_PLANNER_ROUTER_THIRDPARTY_EXTERNAL_ROUTE_MANAGER_H_
#define ONBOARD_PLANNER_ROUTER_THIRDPARTY_EXTERNAL_ROUTE_MANAGER_H_

// IWYU pragma: no_include <ext/alloc_traits.h>
// IWYU pragma: no_include <google/protobuf/repeated_ptr_field.h>

#include <memory>
#include <utility>
#include <vector>

#include "absl/status/status.h"

#include "onboard/math/vec.h"
#include "onboard/planner/router/route_manager_state.h"
#include "onboard/proto/adasis.pb.h"
#include "onboard/proto/route.pb.h"

namespace qcraft::planner::route {

void UpdateSdRouteMatchState(const Vec2d& match_point, int cur_coord_index,
                             SdRouteMatchState* sd_route_match_state);
class ExternalRouteManager {
 public:
  ExternalRouteManager() = default;

 public:
  /**
   *@brief Initialize sd route first time. The `sd_route` cannot be null.
   */
  absl::Status InitSdRoute(const std::shared_ptr<const SDRouteProto>& sd_route,
                           const std::shared_ptr<const SdRouteMatchResultProto>&
                               sd_route_match_result_proto);

  void DeleteSdRoute(const std::shared_ptr<const SDRouteProto>& sd_route);

  /**
   *@brief UpdateRoute from last match position. Ignore the av speed.
   */
  bool UpdateRoute(const Vec2d& global, double max_project_distance);

  void Clear();

  bool has_route() const { return data_.routing_result_proto.success(); }
  RoutingResultProto route() const { return data_.routing_result_proto; }

  // Getters & Setters
  const RouteManagerState& route_manager_state() const { return data_; }
  RouteManagerState* mutable_route_manager_state() { return &data_; }
  const std::vector<Vec2d>& coords() const {
    return data_.sd_route_match_state.coords;
  }
  std::vector<Vec2d>* mutable_coords() {
    return &data_.sd_route_match_state.coords;
  }
  void set_cur_coord_idx(int coord_idx) {
    data_.sd_route_match_state.cur_coord_index = coord_idx;
  }
  int cur_coord_idx() const {
    return data_.sd_route_match_state.cur_coord_index;
  }

  void InjectState(RouteManagerState route_manager_state) {
    data_ = std::move(route_manager_state);
  }

 private:
  RouteManagerState data_;
  int consecutive_failure_times_ = 0;
  bool is_first_match_ = false;  // The first time to match.
};

}  // namespace qcraft::planner::route

#endif  // ONBOARD_PLANNER_ROUTER_THIRDPARTY_EXTERNAL_ROUTE_MANAGER_H_
