#ifndef ONBOARD_PLANNER_ROUTER_PLOT_UTIL_H_
#define ONBOARD_PLANNER_ROUTER_PLOT_UTIL_H_
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "absl/status/statusor.h"

#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/maps/v2/semantic_map_manager.h"
#include "onboard/math/coordinate_converter.h"
#include "onboard/planner/composite_lane_path.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/router/lane_graph/v2/lane_graph.h"
#include "onboard/planner/router/route_sections.h"
#include "onboard/planner/router/route_sections_info.h"
#include "onboard/planner/router/util/route_struct.h"
#include "onboard/proto/lite_msg.pb.h"
#include "onboard/vis/common/color.h"
namespace qcraft {
namespace planner {
absl::StatusOr<LiteMsgWrapper> FetchVantageLiteMsg(
    LiteMsgWrapper::LiteMsgCase msg_type,
    const std::optional<std::string>& channel);
absl::StatusOr<CoordinateConverter> FetchVantageCoordinateConverter();

// Render through smooth coordinate.
void SendDrivePassageToCanvas(const DrivePassage& drive_passages,
                              const std::string& channel);
void SendDrivePassageLineToCanvas(const DrivePassage& drive_passages,
                                  const std::string& channel);

void SendRouteLanePathToCanvas(const PlannerSemanticMapManager& psmm,
                               const CompositeLanePath& route_lane_path,
                               const std::string& topic,
                               const vis::Color& lp_color = vis::Color(0.6, 1.0,
                                                                       0.6));

void SendRouteSectionsAreaToCanvas(
    const PlannerSemanticMapManager& psmm, const RouteSections& route_sections,
    const std::string& channel,
    const vis::Color& fill_color = vis::Color(0.2, 0.81, 0.92, 0.2));

// Render through global coordinate.
void SendRouteCoreStateToCanvas(
    const CoordinateConverter& cc,
    const std::vector<
        std::unique_ptr<route::SearchStateT<const mapping::LaneProto*>>>&
        state_ptrs,
    const std::string& topic,
    const vis::Color& forward_color = vis::Color(0.9, 0.0, 0.5),
    const vis::Color& backward_color = vis::Color(0.0, 1.0, 0.0));

void SendRouteLaneGraphToCanvas(const v2::LaneGraph& lane_graph,
                                const mapping::v2::SemanticMapManager& smm,
                                const RouteSectionsInfo& sections_info,
                                const std::string& topic);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_ROUTER_PLOT_UTIL_H_
