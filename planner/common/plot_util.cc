#include "onboard/planner/common/plot_util.h"

#include <algorithm>
#include <ostream>
#include <vector>

#include "glog/logging.h"

#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/math/geometry/util.h"
#include "onboard/planner/util/planner_semantic_map_util.h"
#include "onboard/vis/canvas/canvas.h"
#include "onboard/vis/canvas/proto/canvas_buffer.pb.h"

#include "offboard/vis/vantage/vantage_server/vantage_canvas_client.h"

namespace qcraft::planner {

void SendPointsToCanvas(absl::Span<const Vec2d> points,
                        const std::string& channel, const vis::Color& color) {
  VLOG(3) << "draw vector of points...";
  QCHECK(channel.length() > 0) << "empty canvas channel name!";
  auto& canvas = vis::vantage::GetCanvasClient()->GetCanvas(channel);
  canvas.SetGroundZero(1);
  for (const auto& pt : points) {
    canvas.DrawPoint(Vec3d(pt, 0.1), color, 6);
  }

  vis::vantage::GetCanvasClient()->FlushAll();
}

void SendBox2dToCanvas(const Box2d& box, const std::string& channel,
                       const vis::Color& color) {
  QCHECK(channel.length() > 0) << "empty canvas channel name!";
  auto& canvas = vis::vantage::GetCanvasClient()->GetCanvas(channel);
  canvas.SetGroundZero(1);

  canvas.DrawBox(Vec3d(box.center()), box.heading(),
                 Vec2d(box.length(), box.width()), color, 3);
  const Vec2d front_center = box.center() + box.half_length() * box.tangent();
  canvas.DrawLine(Vec3d(front_center),
                  Vec3d(front_center + 0.15 * box.length() * box.tangent()),
                  color, 3);

  vis::vantage::GetCanvasClient()->FlushAll();
}

void SendApolloTrajectoryPointsToCanvas(
    absl::Span<const ApolloTrajectoryPointProto> traj_pts,
    const std::string& channel, const vis::Color& color) {
  std::vector<Vec2d> points;
  points.reserve(traj_pts.size());

  for (const auto& traj_pt : traj_pts) {
    points.push_back(Vec2dFromApolloTrajectoryPointProto(traj_pt));
  }

  SendPointsToCanvas(points, channel, color);
}

void SendDiscretizedPathToCanvas(const DiscretizedPath& path,
                                 const std::string& channel,
                                 const vis::Color& color) {
  std::vector<Vec2d> points;
  points.reserve(path.size());

  for (const auto& pt : path) {
    points.push_back(Vec2d(pt.x(), pt.y()));
  }

  SendPointsToCanvas(points, channel, color);
}

void DrawPathSlBoundaryToCanvas(const PathSlBoundary& path_bound,
                                const std::string& channel) {
  auto& canvas = vis::vantage::GetCanvasClient()->GetCanvas(channel);
  canvas.SetGroundZero(1);
  for (int i = 0; i + 1 < path_bound.size(); ++i) {
    canvas.DrawLine(Vec3d(path_bound.reference_center_xy_vector()[i], 0.1),
                    Vec3d(path_bound.reference_center_xy_vector()[i + 1], 0.1),
                    vis::Color::kOrange, 5);

    canvas.DrawLine(Vec3d(path_bound.right_xy_vector()[i], 0.1),
                    Vec3d(path_bound.right_xy_vector()[i + 1], 0.1),
                    vis::Color::kLightCyan, 4);

    canvas.DrawLine(Vec3d(path_bound.target_right_xy_vector()[i], 0.1),
                    Vec3d(path_bound.target_right_xy_vector()[i + 1], 0.1),
                    vis::Color::kLightCyan, 3, vis::BorderStyleProto::DASHED);

    canvas.DrawLine(Vec3d(path_bound.left_xy_vector()[i], 0.1),
                    Vec3d(path_bound.left_xy_vector()[i + 1], 0.1),
                    vis::Color::kLightCyan, 4);

    canvas.DrawLine(Vec3d(path_bound.target_left_xy_vector()[i], 0.1),
                    Vec3d(path_bound.target_left_xy_vector()[i + 1], 0.1),
                    vis::Color::kLightCyan, 3, vis::BorderStyleProto::DASHED);
  }

  vis::vantage::GetCanvasClient()->FlushAll();
}

void DrawLanePathToCanvas(const PlannerSemanticMapManager& psmm,
                          const mapping::LanePath& lp,
                          const std::string& channel, const vis::Color& color) {
  const auto points = SampleLanePathPoints(psmm, lp);
  std::vector<Vec3d> points_3d(points.size());
  std::transform(points.begin(), points.end(), points_3d.begin(),
                 [](const Vec2d& p) { return Vec3d(p, 0.1); });
  DrawPointsLineStripToCanvas(points_3d, channel, color);
}

void DrawLanePathsToCanvas(const PlannerSemanticMapManager& psmm,
                           absl::Span<const mapping::LanePath> lps,
                           const std::string& channel,
                           const vis::Color& color) {
  auto& canvas = vis::vantage::GetCanvasClient()->GetCanvas(channel);
  canvas.SetGroundZero(1);
  for (int i = 0; i < lps.size(); ++i) {
    const auto& lp = lps[i];
    const auto points = SampleLanePathPoints(psmm, lp);
    std::vector<Vec3d> points_3d(points.size());
    std::transform(points.begin(), points.end(), points_3d.begin(),
                   [](const Vec2d& p) { return Vec3d(p, 0.1); });
    canvas.DrawLineStrip(points_3d, color, 3);
  }
  vis::vantage::GetCanvasClient()->Flush(channel, /*append=*/false);
}

void DrawPointsLineStripToCanvas(const std::vector<Vec3d>& points,
                                 const std::string& channel,
                                 const vis::Color& color, int line_width,
                                 vis::BorderStyleProto::LineStyle line_style) {
  auto& canvas = vis::vantage::GetCanvasClient()->GetCanvas(channel);
  canvas.SetGroundZero(1);
  canvas.DrawLineStrip(points, color, line_width, line_style);
  vis::vantage::GetCanvasClient()->FlushAll();
}

}  // namespace qcraft::planner
