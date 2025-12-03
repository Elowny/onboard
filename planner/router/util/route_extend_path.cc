#include "onboard/planner/router/util/route_extend_path.h"

#include <memory>
#include <string>
#include <vector>

#include "absl/status/status.h"
#include "absl/strings/str_cat.h"

#include "onboard/container/strong_int.h"
#include "onboard/global/logging.h"
#include "onboard/maps/lane_path_util.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/map_or_die_macros.h"
#include "onboard/maps/proto/lane_path.pb.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/maps/v2/semantic_map_manager.h"
#include "onboard/planner/router/util/route_struct.h"
#include "onboard/utils/source_location.h"

namespace qcraft::planner::route {

// The lane change is not allowed.
absl::StatusOr<mapping::LanePathProto> FindConnectedLanePathWithin(
    const mapping::v2::SemanticMapManager& semantic_map_manager,
    const mapping::LanePoint& origin, const mapping::LanePoint& dest,
    double allow_distance) {
  if (origin.lane_id() == dest.lane_id()) {
    if (origin.fraction() <= dest.fraction()) {
      return mapping::CreateLanePathProto(origin.fraction(), dest.fraction(),
                                          std::vector{origin.lane_id()});
    } else {
      return absl::InvalidArgumentError("origin is front of dest.");
    }
  }
  using SearchState = SearchStateT<const mapping::v2::Lane*>;
  std::vector<std::unique_ptr<SearchState>> search_states;
  search_states.reserve(10);
  std::vector<SearchState*> dfs_states;
  dfs_states.reserve(10);

  SMM_ASSIGN_LANE_OR_RETURN(origin_lane_info, semantic_map_manager,
                            origin.lane_id(),
                            absl::NotFoundError(absl::StrCat(
                                "Can not find origin id: ", origin.lane_id())));
  search_states.push_back(std::make_unique<SearchState>(SearchState{
      .id = &origin_lane_info,
      .dist_from_start = origin_lane_info.length() * (1.0 - origin.fraction()),
      .parent = nullptr,
  }));
  dfs_states.push_back(search_states.back().get());
  while (!dfs_states.empty()) {
    auto* cur_state = dfs_states.back();
    dfs_states.pop_back();
    if (cur_state->id->Proto().id() == dest.lane_id().value()) {
      std::vector<mapping::ElementId> res;
      auto* reverse_state = cur_state;
      while (reverse_state != nullptr) {
        res.push_back(mapping::ElementId(reverse_state->id->Proto().id()));
        reverse_state = reverse_state->parent;
      }
      std::reverse(res.begin(), res.end());
      return mapping::CreateLanePathProto(origin.fraction(), dest.fraction(),
                                          res);
    }
    if (cur_state->dist_from_start > allow_distance) {
      continue;
    }
    for (const auto& lane_id : cur_state->id->Proto().outgoing_lanes()) {
      SMM_ASSIGN_LANE_OR_CONTINUE(cur_lane_info, semantic_map_manager, lane_id);

      search_states.push_back(std::make_unique<SearchState>(SearchState{
          .id = &cur_lane_info,
          .dist_from_start =
              cur_state->dist_from_start + cur_lane_info.length(),
          .parent = cur_state,
      }));
      dfs_states.push_back(search_states.back().get());
    }
  }

  return absl::NotFoundError(absl::StrCat(
      "Cannot find a simple directed path. origin:", origin.DebugString(),
      ", destination:", dest.DebugString(), ", ", QCRAFT_LOC.ToString()));
}

}  // namespace qcraft::planner::route
