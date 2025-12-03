#ifndef ONBOARD_PLANNER_SCHEDULER_SCHEDULER_PLOT_UTIL_H_
#define ONBOARD_PLANNER_SCHEDULER_SCHEDULER_PLOT_UTIL_H_

#include <array>
#include <string>

#include "onboard/maps/lane_path.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/planner/common/lane_path_info.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/router/lane_graph/v2/lane_graph.h"
#include "onboard/planner/router/route_sections_info.h"

namespace qcraft::planner {

void SendLaneGraphToCanvas(const v2::LaneGraph& lane_graph,
                           const PlannerSemanticMapManager& psmm,
                           const RouteSectionsInfo& sections_info,
                           const std::string& topic);

void SendLanePathInfoToCanvas(const LanePathInfo& lp_info,
                              const PlannerSemanticMapManager& psmm,
                              const std::string& topic);

void SendLocalLaneMapToCanvas(
    const std::array<mapping::LanePath, 3>& lane_paths,
    const PlannerSemanticMapManager& psmm, const std::string& topic);

void SendAvSlTrajectoryToCanvas(
    const DrivePassage& drive_passage,
    const PiecewiseLinearFunction<double, double>& traj_l_s,
    const std::string& topic);
}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_SCHEDULER_SCHEDULER_PLOT_UTIL_H_
