#ifndef ONBOARD_PLANNER_COMMON_PLOT_UTIL_H_
#define ONBOARD_PLANNER_COMMON_PLOT_UTIL_H_

#include <string>
#include <vector>

#include "absl/types/span.h"

#include "onboard/maps/lane_path.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/vec.h"
#include "onboard/planner/common/discretized_path.h"
#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/vis/canvas/proto/canvas_buffer.pb.h"
#include "onboard/vis/common/color.h"

namespace qcraft::planner {

void SendPointsToCanvas(absl::Span<const Vec2d> points,
                        const std::string& channel, const vis::Color& color);

void SendBox2dToCanvas(const Box2d& box, const std::string& channel,
                       const vis::Color& color);

void SendApolloTrajectoryPointsToCanvas(
    absl::Span<const ApolloTrajectoryPointProto> traj_pts,
    const std::string& channel, const vis::Color& color);

void SendDiscretizedPathToCanvas(const DiscretizedPath& path,
                                 const std::string& channel,
                                 const vis::Color& color);

void DrawPathSlBoundaryToCanvas(const PathSlBoundary& path_bound,
                                const std::string& channel);

void DrawLanePathToCanvas(const PlannerSemanticMapManager& psmm,
                          const mapping::LanePath& lp,
                          const std::string& channel, const vis::Color& color);

void DrawLanePathsToCanvas(const PlannerSemanticMapManager& psmm,
                           absl::Span<const mapping::LanePath> lps,
                           const std::string& channel, const vis::Color& color);

void DrawPointsLineStripToCanvas(
    const std::vector<Vec3d>& points, const std::string& channel,
    const vis::Color& color, int line_width = 3,
    vis::BorderStyleProto::LineStyle line_style = vis::BorderStyleProto::SOLID);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_COMMON_PLOT_UTIL_H_
