#ifndef ONBOARD_PLANNER_ROUTER_ROUTE_CORE_H_
#define ONBOARD_PLANNER_ROUTER_ROUTE_CORE_H_

#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/hash/hash.h"
#include "absl/status/statusor.h"

#include "onboard/lite/check.h"
#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/maps/spatial_search_util.h"
#include "onboard/maps/v2/semantic_map_manager.h"
#include "onboard/planner/composite_lane_path.h"
#include "onboard/planner/router/proto/route_params.pb.h"
#include "onboard/planner/router/road_conditions_process.h"
#include "onboard/planner/router/route_sections.h"
#include "onboard/planner/router/util/route_struct.h"

namespace qcraft::planner::route {
using LaneSearchState = SearchStateT<const mapping::LaneProto*>;
using SearchStateMap = absl::flat_hash_map<int64_t, LaneSearchState*>;

struct SearchStateComparator {
  bool operator()(const LaneSearchState* x, const LaneSearchState* y) const {
    if (use_time) {
      return x->time_from_start > y->time_from_start;
    } else {
      return x->dist_from_start > y->dist_from_start;
    }
  }
  bool use_time = true;
};

struct RouteCorePrecondition {
  bool is_bus;
  bool use_time;
  RouteRestrictDistrict route_restrict_district;
};

class RouteCore {
  using OpenQueue =
      std::priority_queue<LaneSearchState*, std::vector<LaneSearchState*>,
                          SearchStateComparator>;
  enum class SearchStateCode : int {
    kSearchSuccess = 0,
    kForwardError = 10,
    kBackwardError = 11,
    kPathNotFound = 20,
    kDefaultState = 99,
  };

 public:
  RouteCore() = default;
  explicit RouteCore(const mapping::v2::SemanticMapManager* smm,
                     const RouteParamProto* route_param)
      : smm_(smm), route_param_(route_param) {}

  void set_semantic_map_manager(const mapping::v2::SemanticMapManager* smm) {
    smm_ = QCHECK_NOTNULL(smm);
  }

  void set_route_param_proto(const RouteParamProto* route_param) {
    route_param_ = QCHECK_NOTNULL(route_param);
  }

  SearchStateCode state_code() const { return state_code_; }

  absl::StatusOr<std::pair<RouteSections, CompositeLanePath>>
  SearchForRoutePathAlongLanePoints(
      const RouteCorePrecondition& route_precondi,
      const std::vector<mapping::PointToLane>& origin,
      const std::vector<std::vector<mapping::PointToLane>>& dests,
      const CoordinateConverter* cc = nullptr);

  absl::StatusOr<std::pair<RouteSections, CompositeLanePath>>
  SearchForRoutePathFromLanePoint(
      const RouteCorePrecondition& route_precondi,
      const std::vector<mapping::PointToLane>& origins,
      const std::vector<mapping::PointToLane>& dests);

  std::string OpenQueueDebugString(bool forward) const;

 private:
  void ExpandState(const RouteCorePrecondition& route_pre,
                   LaneSearchState* cur_state, bool forward);

  absl::StatusOr<mapping::ElementId> LaneLevelBiDijkstra(
      const RouteCorePrecondition& route_pre);

  absl::StatusOr<std::pair<RouteSections, CompositeLanePath>> GenerateRoutePath(
      mapping::ElementId link_lane_id,
      const RouteCorePrecondition& route_pre) const;

 private:
  OpenQueue open_forward_;
  OpenQueue open_backward_;
  SearchStateMap forward_states_;
  SearchStateMap backward_states_;
  SearchStateCode state_code_ = SearchStateCode::kDefaultState;
  std::vector<std::unique_ptr<LaneSearchState>> state_ptrs_;

  // Not owned.
  const mapping::v2::SemanticMapManager* smm_;
  const RouteParamProto* route_param_;
};

}  // namespace qcraft::planner::route

#endif  // ONBOARD_PLANNER_ROUTER_ROUTE_CORE_H_
