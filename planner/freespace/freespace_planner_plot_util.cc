#include "onboard/planner/freespace/freespace_planner_plot_util.h"

#include <string>
#include <vector>

#include "onboard/math/geometry/box2d.h"
#include "onboard/math/vec.h"
#include "onboard/planner/common/discretized_path.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/vis/canvas/canvas.h"
#include "onboard/vis/common/color.h"

#include "offboard/vis/vantage/vantage_server/vantage_canvas_client.h"

namespace qcraft {
namespace planner {

void DrawDirectionalPath(std::string_view name,
                         absl::Span<const DirectionalPath> paths) {
  vis::Canvas& canvas =
      vis::vantage::GetCanvasClient()->GetCanvas(std::string(name));
  canvas.SetGroundZero(1);
  for (const auto& path : paths) {
    for (const auto& point : path.path) {
      if (path.forward) {
        canvas.DrawCircle(Vec3d(point.x(), point.y(), 0.0), /*size*/ 0.2,
                          vis::Color::kLightGreen);
      } else {
        canvas.DrawCircle(Vec3d(point.x(), point.y(), 0.0), /*size*/ 0.2,
                          vis::Color::kLightBlue);
      }
    }
  }
  vis::vantage::GetCanvasClient()->FlushAll();
}

void DrawPathSweptVolume(std::string_view name,
                         absl::Span<const Box2d> path_swept_volume) {
  vis::Canvas& canvas =
      vis::vantage::GetCanvasClient()->GetCanvas(std::string(name));
  canvas.SetGroundZero(1);
  for (const auto& box : path_swept_volume) {
    canvas.DrawBox(Vec3d(box.center()), box.heading(),
                   Vec2d(box.length(), box.width()), vis::Color::kLightYellow);
  }
  vis::vantage::GetCanvasClient()->FlushAll();
}

}  // namespace planner
}  // namespace qcraft
