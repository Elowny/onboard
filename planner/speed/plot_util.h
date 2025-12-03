#ifndef ONBOARD_PLANNER_SPEED_PLOT_UTIL_H_
#define ONBOARD_PLANNER_SPEED_PLOT_UTIL_H_

#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include "absl/types/span.h"

#include "onboard/planner/common/discretized_path.h"
#include "onboard/planner/object/spacetime_object_trajectory.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/speed/proto/speed_finder.pb.h"
#include "onboard/planner/speed/speed_vector.h"
#include "onboard/planner/speed/st_boundary_with_decision.h"
#include "onboard/proto/charts.pb.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/proto/vehicle.pb.h"
#include "onboard/vis/common/color.h"

namespace qcraft {
namespace planner {
namespace speed {

// Export st_boundary data to chart.
void ExportStBoundaryToChart(
    std::string_view base_name, int traj_steps,
    absl::Span<const StBoundaryWithDecision> st_boundaries_with_decision,
    const SpeedFinderDebugProto& speed_finder_proto, double path_length,
    vis::vantage::ChartDataProto* chart);

void ExportSpeedStDataToChart(
    const SpeedVector& speed, const std::string& name, const vis::Color& color,
    const vis::vantage::ChartSeriesDataProto::PenStyle& pen_style,
    vis::vantage::ChartDataProto* chart);

void ExportSpeedVtDataToChart(const SpeedVector& speed, const std::string& name,
                              const vis::Color& color, bool add_tips,
                              vis::vantage::ChartDataProto* chart);

void ExportSpeedPointsProtoStDataToChart(
    const SpeedPointsProto& speed, const std::string& name,
    const vis::Color& color,
    const vis::vantage::ChartSeriesDataProto::PenStyle& pen_style,
    vis::vantage::ChartDataProto* chart);

void ExportPredVtDataToChart(
    const std::unordered_map<std::string, const SpacetimeObjectTrajectory*>&
        pred_trajs,
    std::string_view base_name, vis::vantage::ChartDataProto* chart);

void ExportSamplingDpSpeedToChart(
    std::string_view base_name,
    const absl::Span<const StBoundaryWithDecision>& st_boundaries_with_decision,
    const SpeedVector& preliminary_speed,
    const InteractiveSpeedDebugProto& interactive_speed_debug,
    const SpeedFinderDebugProto& speed_finder_proto,
    vis::vantage::ChartDataProto* chart);

void ExportInteractiveSpeedToChart(
    std::string_view base_name,
    const absl::Span<const StBoundaryWithDecision>& st_boundaries_with_decision,
    const SpeedVector& preliminary_speed,
    const InteractiveSpeedDebugProto& interactive_speed_debug,
    const SpeedFinderDebugProto& speed_finder_proto,
    vis::vantage::ChartDataProto* interactive_speed_chart);

void ExportSpeedLimitToChart(std::string_view base_name,
                             const SpeedFinderDebugProto& speed_finder_proto,
                             vis::vantage::ChartDataProto* chart);
void ExportTrajToChart(
    std::string_view base_name,
    const std::vector<ApolloTrajectoryPointProto>& trajectory,
    vis::vantage::ChartDataProto* chart);

void ExportPathToChart(std::string_view base_name,
                       const std::vector<PathPoint>& path,
                       vis::vantage::ChartDataProto* chart);

void DrawPathPoint(const PathPoint& path_point, std::string_view traj_id,
                   const vis::Color& color);

void DrawStBoundaryOnCanvas(
    std::string_view base_name,
    absl::Span<const StBoundaryWithDecision> st_boundaries_with_decision,
    const VehicleGeometryParamsProto& vehicle_geom,
    const SpacetimeTrajectoryManager& traj_mgr, const DiscretizedPath& path);

}  // namespace speed
}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_SPEED_PLOT_UTIL_H_
