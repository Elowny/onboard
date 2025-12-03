#ifndef ONBOARD_PLANNER_UTIL_TRAJECTORY_PLOT_UTIL_H_
#define ONBOARD_PLANNER_UTIL_TRAJECTORY_PLOT_UTIL_H_

#include <functional>
#include <string>
#include <vector>

#include "onboard/math/vec.h"
#include "onboard/planner/trajectory_point.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/vis/canvas/canvas.h"
#include "onboard/vis/common/color.h"

namespace qcraft {
namespace planner {

struct TrajectoryPlotInfo {
  const std::vector<TrajectoryPoint>& traj;
  std::string name;
  const vis::Color& color;
};

// To create a Vec3d vector for use with CanvasDrawTrajectory() functions below
// from a planner trajectory (which are Vec2d points), use the VisTrajToVector()
// convenience functions below.
void CanvasDrawTrajectory(const std::vector<Vec3d>& traj,
                          const vis::Color& line_color,
                          const vis::Color& point_color, double line_width,
                          double point_size, bool render_indices,
                          vis::Canvas* canvas);
void CanvasDrawTrajectory(const std::vector<Vec3d>& traj,
                          const vis::Color& line_color,
                          const vis::Color& point_color, double line_width,
                          double point_size, vis::Canvas* canvas);
void CanvasDrawTrajectory(const std::vector<Vec3d>& traj,
                          const vis::Color& line_color,
                          const vis::Color& point_color, vis::Canvas* canvas);
void CanvasDrawTrajectory(const std::vector<Vec3d>& traj,
                          const vis::Color& color, vis::Canvas* canvas);
void CanvasDrawTrajectory(const std::vector<Vec3d>& traj,
                          const vis::Color& color,
                          const std::string& canvas_channel);
void CanvasDrawTrajectory(const std::vector<Vec3d>& traj,
                          const vis::Color& color, double line_width,
                          double point_size, bool render_indices,
                          vis::Canvas* canvas);
void CanvasDrawTrajectory(const std::vector<Vec3d>& traj,
                          const vis::Color& color, bool render_indices,
                          vis::Canvas* canvas);
void CanvasDrawTrajectory(const std::vector<Vec3d>& traj,
                          const vis::Color& color, bool render_indices,
                          const std::string& canvas_channel);

void CanvasDrawTrajectory(const std::vector<ApolloTrajectoryPointProto>& traj,
                          const std::string& channel);

// Horizon and traj parameter are indices.
std::vector<Vec3d> VisIndexTrajToVector(const std::function<Vec2d(int)>& traj,
                                        int horizon, double z_inc, double z0);
std::vector<Vec3d> VisIndexTrajToVector(const std::function<Vec2d(int)>& traj,
                                        int horizon);

void CanvasDrawTrajectories(const std::string& base_name,
                            const std::vector<TrajectoryPlotInfo>& trajs);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_UTIL_TRAJECTORY_PLOT_UTIL_H_
