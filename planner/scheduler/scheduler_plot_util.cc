#include "onboard/planner/scheduler/scheduler_plot_util.h"

#include <algorithm>
#include <ostream>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/hash/hash.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/types/span.h"
#include "boost/container/vector.hpp"

#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/maps/lane_point.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/router/lane_graph/v2/lane_graph.h"
#include "onboard/planner/util/lane_point_util.h"
#include "onboard/planner/util/planner_semantic_map_util.h"
#include "onboard/vis/canvas/canvas.h"
#include "onboard/vis/common/color.h"

#include "offboard/vis/vantage/vantage_server/vantage_canvas_client.h"

namespace qcraft::planner {

void SendLaneGraphToCanvas(const v2::LaneGraph& lane_graph,
                           const PlannerSemanticMapManager& psmm,
                           const RouteSectionsInfo& sections_info,
                           const std::string& topic) {
  QCHECK(!topic.empty()) << "empty canvas channel name!";
  auto& canvas = vis::vantage::GetCanvasClient()->GetCanvas(topic);
  canvas.SetGroundZero(1);

  const auto& layers = lane_graph.layers;
  const auto& sections = sections_info.section_segments();
  absl::flat_hash_map<v2::LaneGraph::VertexId, Vec2d> vertex_pos;

  for (int i = 0; i < layers.size(); ++i) {
    const auto [sec_idx, frac] = layers[i];
    for (const auto lane_id : sections[sec_idx].lane_ids) {
      const Vec2d pos =
          ComputeLanePointPos(psmm, mapping::LanePoint(lane_id, frac));
      canvas.DrawPoint(Vec3d(pos, 0.1), vis::Color::kLightBlue, 8);
      vertex_pos[v2::ToVertId(i, lane_id)] = pos;
    }
    const auto rightmost_id = sections[sec_idx].lane_ids.back();
    const double theta =
        (-ComputeLanePointTangent(psmm, mapping::LanePoint(rightmost_id, frac))
              .Perp())
            .FastAngle();
    canvas.DrawText(absl::StrFormat("Layer %d", i),
                    Vec3d(vertex_pos[v2::ToVertId(i, rightmost_id)], 0.1),
                    theta, 0.3, vis::Color::kLightRed);
  }

  const auto graph = lane_graph.graph;
  for (const auto& vertex : graph.vertices()) {
    if (!vertex_pos.contains(vertex)) continue;
    for (const auto& [to_id, cost] : graph.edges_from(vertex)) {
      if (!vertex_pos.contains(to_id)) continue;

      const auto &from_pos = vertex_pos[vertex], to_pos = vertex_pos[to_id];
      canvas.DrawLine(Vec3d(from_pos, 0.1), Vec3d(to_pos, 0.1),
                      vis::Color::kLightCyan, 3);
      canvas.DrawText(absl::StrFormat("%.3f", cost),
                      Vec3d(Lerp(from_pos, to_pos, 0.15), 0.1),
                      (to_pos - from_pos).normalized().FastAngle(), 0.2,
                      vis::Color::kLightRed);
    }
  }

  vis::vantage::GetCanvasClient()->FlushAll();
}

void SendLanePathInfoToCanvas(const LanePathInfo& lp_info,
                              const PlannerSemanticMapManager& psmm,
                              const std::string& topic) {
  QCHECK(!topic.empty()) << "empty canvas channel name!";
  auto& canvas = vis::vantage::GetCanvasClient()->GetCanvas(topic);
  canvas.SetGroundZero(1);

  const auto& lp = lp_info.lane_path();
  const auto points = SampleLanePathPoints(psmm, lp);
  std::vector<Vec3d> points_3d(points.size());
  std::transform(points.begin(), points.end(), points_3d.begin(),
                 [](const Vec2d& p) { return Vec3d(p, 1.0); });
  canvas.DrawLineStrip(points_3d, vis::Color::kMint, 3);

  const Vec2d len_along_route_pt = ComputeLanePointPos(
      psmm, lp.ArclengthToLanePoint(lp_info.length_along_route()));
  canvas.DrawCircle(Vec3d(len_along_route_pt, 1.0), 0.3,
                    vis::Color::kLightMagenta, vis::Color::kLightMagenta);
  const Vec2d max_reach_len_pt = ComputeLanePointPos(
      psmm, lp.ArclengthToLanePoint(lp_info.max_reach_length()));
  canvas.DrawCircle(Vec3d(max_reach_len_pt, 1.0), 0.4, vis::Color::kLightRed,
                    vis::Color::kLightRed);

  vis::vantage::GetCanvasClient()->FlushAll();
}

void SendLocalLaneMapToCanvas(
    const std::array<mapping::LanePath, 3>& lane_paths,
    const PlannerSemanticMapManager& psmm, const std::string& topic) {
  QCHECK(!topic.empty()) << "empty canvas channel name!";
  auto& canvas = vis::vantage::GetCanvasClient()->GetCanvas(topic);
  canvas.SetGroundZero(1);

  for (const auto& lane_path : lane_paths) {
    const auto points = SampleLanePathPoints(psmm, lane_path);
    std::vector<Vec3d> points_3d(points.size());
    std::transform(points.begin(), points.end(), points_3d.begin(),
                   [](const Vec2d& p) { return Vec3d(p, 1.0); });
    canvas.DrawLineStrip(points_3d, vis::Color::kSkyBlue, 3);

    double accumulate_length = 0.0;
    for (const auto& seg : lane_path) {
      accumulate_length += seg.length();
      const Vec2d seg_end_pt = ComputeLanePointPos(
          psmm, lane_path.ArclengthToLanePoint(accumulate_length));

      canvas.DrawCircle(Vec3d(seg_end_pt, 1.0), 0.3, vis::Color::kMint,
                        vis::Color::kMint);
    }
  }
  vis::vantage::GetCanvasClient()->FlushAll();
}

void SendAvSlTrajectoryToCanvas(
    const DrivePassage& drive_passage,
    const PiecewiseLinearFunction<double, double>& traj_l_s,
    const std::string& topic) {
  QCHECK(!topic.empty()) << "empty canvas channel name!";
  auto& canvas = vis::vantage::GetCanvasClient()->GetCanvas(topic);
  canvas.SetGroundZero(1);
  for (int i = 0; i + 1 < traj_l_s.x().size(); ++i) {
    const auto xy =
        drive_passage.QueryPointXYAtSL(traj_l_s.x()[i], traj_l_s.y()[i])
            .value();
    const auto xy_next =
        drive_passage.QueryPointXYAtSL(traj_l_s.x()[i + 1], traj_l_s.y()[i + 1])
            .value();
    canvas.DrawLine(Vec3d(xy, 0.1), Vec3d(xy_next, 0.1), vis::Color::kRed, 3);
    canvas.DrawPoint(Vec3d(xy, 0.1), vis::Color::kRed, 5);
  }

  vis::vantage::GetCanvasClient()->FlushAll();
}

}  // namespace qcraft::planner
