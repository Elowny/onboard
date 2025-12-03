#ifndef ONBOARD_PLANNER_ROUTER_NOA_HD_HMI_ADAPTER_H_
#define ONBOARD_PLANNER_ROUTER_NOA_HD_HMI_ADAPTER_H_

#include <optional>

#include "onboard/maps/v2/semantic_map_manager.h"
#include "onboard/maps/v2/semantic_map_spatial_index.h"
#include "onboard/math/vec.h"
#include "onboard/proto/hmi_content.pb.h"
#include "onboard/proto/route.pb.h"
#include "onboard/proto/route_navigation.pb.h"

namespace qcraft::planner::route::noa {

/**
 * @brief Calculate and update the nearest navigation action based on the mpp
 * and hd map. Now only update the RAMP_EXIT action by lane proto.
 * @param nav_instruction [In&Out] NonNull.
 */
void UpdateNavInstruction(const mapping::v2::SemanticMapManager& smm,
                          const RouteSectionSequenceProto& sections,
                          int horizon, NaviInstructionProto* nav_instruction);

/**
 * @brief Calculate the distance to hd destination
 * @param smm_index smm with spatial index.
 * @param route_sections  The route path
 * @param sd_destination The destination point from the sd route.
 * @param dist_to_sd_end The disance to the sd route end not any
 * waypoints.
 * @return An optional object with the distance(m) iff the destination is on the
 * current hd map, otherwise std::nullopt
 */
std::optional<int> CaclDistanceToEnd(
    const mapping::v2::SemanticMapSpatialIndex& smm_index,
    const RouteSectionSequenceProto& route_sections,
    const Vec2d& sd_destination, int dist_to_sd_end, int dist_to_map_boundary);

/**
 * @brief Add the odd events to hmi_content_proto. Clear the odd events in the
 * `nca_odc_proto` before update.
 * @param nca_odc_proto [In&Out] NonNull.
 */
void UpdateHmiOddEvent(const mapping::v2::SemanticMapManager& smm,
                       const RouteSectionSequenceProto& route_sections,
                       int horizon, NcaOdcProto* nca_odc_proto);

}  // namespace qcraft::planner::route::noa

#endif  // ONBOARD_PLANNER_ROUTER_NOA_HD_HMI_ADAPTER_H_
