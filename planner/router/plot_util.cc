#include "onboard/planner/router/plot_util.h"

#include <algorithm>
#include <cstdint>
#include <iostream>
#include <map>
#include <string>
#include <utility>
#include <vector>
// IWYU pragma: no_include "Eigen/Core"
// IWYU pragma: no_include <google/protobuf/repeated_ptr_field.h>
// IWYU pragma: no_include <boost/container/vector.hpp>
#include "absl/container/flat_hash_map.h"
#include "absl/hash/hash.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/types/span.h"
#include "glog/logging.h"

#include "common/proto/map_geometry.pb.h"

#include "onboard/container/strong_int.h"
#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"
#include "onboard/maps/lane_path.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/maps_common.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/router/route_util.h"
#include "onboard/planner/util/lane_point_util.h"
#include "onboard/proto/lite_msg.pb.h"
#include "onboard/proto/localization.pb.h"
#include "onboard/utils/status_macros.h"
#include "onboard/vis/canvas/canvas.h"

#include "offboard/vis/vantage/vantage_server/proto/vantage_server.pb.h"
#include "offboard/vis/vantage/vantage_server/vantage_canvas_client.h"
#include "offboard/vis/vantage/vantage_server/vantage_client_man.h"

namespace qcraft {
namespace planner {

void SendDrivePassageToCanvas(const DrivePassage& drive_passage,
                              const std::string& channel) {
  VLOG(3) << "draw drive passage...";
  QCHECK(channel.length() > 0) << "empty canvas channel name!";
  auto& canvas = vis::vantage::GetCanvasClient()->GetCanvas(channel);
  canvas.SetGroundZero(1);
  for (const auto& station : drive_passage.stations()) {
    canvas.DrawPoint(Vec3d(station.xy()), vis::Color::kLightBlue, 10);
    canvas.DrawText(absl::StrCat(station.accumulated_s()), Vec3d(station.xy()),
                    station.tangent().FastAngle(), 0.1, vis::Color::kLightRed);
    double left_max_dis = 0;
    double right_max_dis = 0;
    for (const auto& boundary : station.boundaries()) {
      if (boundary.lat_offset > left_max_dis)
        left_max_dis = boundary.lat_offset;
      if (boundary.lat_offset < right_max_dis)
        right_max_dis = boundary.lat_offset;
      switch (boundary.type) {
        case StationBoundaryType::BROKEN_WHITE:
          canvas.DrawPoint(Vec3d(station.lat_point(boundary.lat_offset)),
                           vis::Color(0.7, 0.7, 0.7, 0.1), 8);
          break;
        case StationBoundaryType::SOLID_WHITE:
        case StationBoundaryType::BROKEN_LEFT_DOUBLE_WHITE:
        case StationBoundaryType::BROKEN_RIGHT_DOUBLE_WHITE:
          canvas.DrawPoint(Vec3d(station.lat_point(boundary.lat_offset)),
                           vis::Color(1.0, 1.0, 1.0, 1), 10);
          break;
        case StationBoundaryType::BROKEN_YELLOW:
        case StationBoundaryType::SOLID_YELLOW:
        case StationBoundaryType::SOLID_DOUBLE_YELLOW:
          canvas.DrawPoint(Vec3d(station.lat_point(boundary.lat_offset)),
                           vis::Color(1.0, 1.0, 0.0, 1), 10);
          break;
        case StationBoundaryType::CURB:
          canvas.DrawPoint(Vec3d(station.lat_point(boundary.lat_offset)),
                           vis::Color(1.0, 0.1, 0.0, 1), 10);
          break;
        case StationBoundaryType::VIRTUAL_CURB:
        case StationBoundaryType::PREDICTION_VIRTUAL_CURB:
          canvas.DrawPoint(Vec3d(station.lat_point(boundary.lat_offset)),
                           vis::Color(0.55, 0.0, 0.0, 1), 10);
          break;
        case StationBoundaryType::UNKNOWN_TYPE:
          break;
      }
    }
    canvas.DrawLine(Vec3d(station.lat_point(right_max_dis)),
                    Vec3d(station.lat_point(left_max_dis)),
                    vis::Color(0.8, 0.8, 0.8, 0.2), 1);
  }

  vis::vantage::GetCanvasClient()->FlushAll();
}

void SendDrivePassageLineToCanvas(const DrivePassage& drive_passage,
                                  const std::string& channel) {
  VLOG(3) << "draw drive passage...";
  QCHECK(channel.length() > 0) << "empty canvas channel name!";
  auto& canvas = vis::vantage::GetCanvasClient()->GetCanvas(channel);
  canvas.SetGroundZero(1);
  const auto& stations = drive_passage.stations();
  for (int i = 1; i < stations.size(); ++i) {
    canvas.DrawLine(Vec3d(stations[StationIndex(i - 1)].xy()),
                    Vec3d(stations[StationIndex(i)].xy()), vis::Color::kRed, 5);
  }

  vis::vantage::GetCanvasClient()->FlushAll();
}

void SendRouteLanePathToCanvas(const PlannerSemanticMapManager& psmm,
                               const CompositeLanePath& route_lane_path,
                               const std::string& topic,
                               const vis::Color& lp_color) {
  QCHECK(!route_lane_path.IsEmpty());
  vis::Canvas& canvas = vis::vantage::GetCanvasClient()->GetCanvas(topic);
  canvas.SetGroundZero(1);
  const auto render_lane_points = [&canvas](const std::vector<Vec2d>& points,
                                            const vis::Color& color) {
    std::vector<Vec3d> points_3d(points.size());
    std::transform(points.begin(), points.end(), points_3d.begin(),
                   [](const Vec2d& p) { return Vec3d(p, 1.0); });
    canvas.DrawLineStrip(points_3d, color, 3);
    canvas.DrawCircle(points_3d.front(), 0.1, color, color);
  };
  const auto render_lane_portion =
      [&psmm, &render_lane_points](const mapping::ElementId lane_id,
                                   double start_fraction, double end_fraction,
                                   const vis::Color& color) {
        const auto* lane_info_ptr = psmm.FindLaneInfoOrNull(lane_id);
        if (lane_info_ptr == nullptr) {
          return;
        }
        const auto& lane_info = *lane_info_ptr;
        const auto start_seg_frac = lane_info.SegmentFraction(start_fraction);
        const auto end_seg_frac = lane_info.SegmentFraction(end_fraction);
        std::vector<Vec2d> points;
        points.push_back(Lerp(lane_info.points_smooth[start_seg_frac.first],
                              lane_info.points_smooth[start_seg_frac.first + 1],
                              start_seg_frac.second));
        for (int i = start_seg_frac.first; i < end_seg_frac.first; ++i) {
          points.push_back(lane_info.points_smooth[i + 1]);
        }
        points.push_back(Lerp(lane_info.points_smooth[end_seg_frac.first],
                              lane_info.points_smooth[end_seg_frac.first + 1],
                              end_seg_frac.second));
        render_lane_points(points, color);
      };
  const auto render_lane_path_portion =
      [&render_lane_portion](const mapping::LanePath& lane_path, int lane_index,
                             double start_fraction, double end_fraction,
                             const vis::Color& color) {
        const double clamped_start_fraction = std::max(
            lane_index == 0 ? lane_path.start_fraction() : 0.0, start_fraction);
        const double clamped_end_fraction = std::min(
            lane_index + 1 == lane_path.size() ? lane_path.end_fraction() : 1.0,
            end_fraction);
        render_lane_portion(lane_path.lane_id(lane_index),
                            clamped_start_fraction, clamped_end_fraction,
                            color);
      };

  const vis::Color kColorBeforeEntry(0.4, 0.6, 0.8);  // NOLINT
  const vis::Color& kColorActive = lp_color;          // NOLINT
  const vis::Color kColorAfterExit(0.8, 0.6, 0.4);    // NOLINT

  for (int i = 0; i < route_lane_path.num_lane_paths(); ++i) {
    const mapping::LanePath& lane_path_i = route_lane_path.lane_path(i);
    const auto& entry = route_lane_path.LanePathEntryIndexPoint(i);
    const auto& exit = route_lane_path.LanePathExitIndexPoint(i);
    VLOG(3) << "lane path index " << i << ": " << lane_path_i.DebugString();
    VLOG(3) << "  entry: " << entry.first << " " << entry.second;
    VLOG(3) << "  entry: " << exit.first << " " << exit.second;

    // Draw the portion before entry point.
    for (int j = 0; j < entry.first; ++j) {
      render_lane_path_portion(lane_path_i, j, 0.0, 1.0, kColorBeforeEntry);
    }
    render_lane_path_portion(lane_path_i, entry.first, 0.0, entry.second,
                             kColorBeforeEntry);
    if (entry.first == exit.first) {
      render_lane_path_portion(lane_path_i, entry.first, entry.second,
                               exit.second, kColorActive);
    } else {
      render_lane_path_portion(lane_path_i, entry.first, entry.second, 1.0,
                               kColorActive);
      for (int j = entry.first + 1; j < exit.first; ++j) {
        render_lane_path_portion(lane_path_i, j, 0.0, 1.0, kColorActive);
      }
      render_lane_path_portion(lane_path_i, exit.first, 0.0, exit.second,
                               kColorActive);
    }
    render_lane_path_portion(lane_path_i, exit.first, exit.second, 1.0,
                             kColorAfterExit);
    for (int j = exit.first + 1; j < lane_path_i.size(); ++j) {
      render_lane_path_portion(lane_path_i, j, 0.0, 1.0, kColorAfterExit);
    }
  }

  vis::vantage::GetCanvasClient()->FlushAll();
}

void SendRouteSectionsAreaToCanvas(const PlannerSemanticMapManager& psmm,
                                   const RouteSections& route_sections,
                                   const std::string& channel,
                                   const vis::Color& fill_color) {
  vis::Canvas& canvas = vis::vantage::GetCanvasClient()->GetCanvas(channel);
  canvas.SetGroundZero(1);

  for (const auto& polygon :
       SampleRouteSectionsPolygons(psmm, route_sections)) {
    const auto& points = polygon.points();
    std::vector<Vec3d> points_3d(points.size());
    std::transform(points.begin(), points.end(), points_3d.begin(),
                   [](const Vec2d& p) { return Vec3d(p, 0.1); });
    canvas.DrawPolygon(points_3d, vis::Color::kTransparent, fill_color);
  }
  vis::vantage::GetCanvasClient()->FlushAll();
}

void SendRouteCoreStateToCanvas(
    const CoordinateConverter& cc,
    const std::vector<
        std::unique_ptr<route::SearchStateT<const mapping::LaneProto*>>>&
        state_ptrs,
    const std::string& topic, const vis::Color& forward_color,
    const vis::Color& backward_color) {
  QCHECK(!state_ptrs.empty()) << "Input states are empty!";
  vis::Canvas& canvas = vis::vantage::GetCanvasClient()->GetCanvas(topic);
  canvas.SetGroundZero(1);

  std::map<std::pair<int64_t, bool>, double> visited;
  for (const auto& state_ptr : state_ptrs) {
    const auto cur_lane_id = state_ptr->id->id();
    const bool is_forward = state_ptr->IsForward();
    const double cost = state_ptr->time_from_start;
    if (visited.find({cur_lane_id, is_forward}) != visited.end() &&
        visited[{cur_lane_id, is_forward}] <= cost) {
      continue;
    }
    visited[{cur_lane_id, is_forward}] = cost;
    const auto& cur_lane_proto = *state_ptr->id;
    const std::string state = state_ptr->IsOpen() ? "O" : "C";

    const auto parent_id = (state_ptr->parent == nullptr)
                               ? mapping::kInvalidElementId.value()
                               : state_ptr->parent->id->id();

    const Vec2d smooth_pos =
        cc.GlobalToSmooth(ComputePointOnLane(cur_lane_proto.polyline().points(),
                                             state_ptr->fraction)
                              .xy());

    const Vec2d half_smooth_pos = cc.GlobalToSmooth(
        ComputePointOnLane(cur_lane_proto.polyline().points(), /*fraction=*/0.5)
            .xy());

    const double heading = (half_smooth_pos - smooth_pos).FastAngle();
    canvas.DrawText(absl::StrFormat("Cur: %d, Cost: %.4f, Parent: %d, %s",
                                    cur_lane_id, cost, parent_id, state),
                    Vec3d(smooth_pos, 0.1), heading,
                    /*size=*/0.5, is_forward ? forward_color : backward_color);
  }
  vis::vantage::GetCanvasClient()->FlushAll();
}

absl::StatusOr<LiteMsgWrapper> FetchVantageLiteMsg(
    LiteMsgWrapper::LiteMsgCase msg_type,
    const std::optional<std::string>& channel) {
  vis::vantage::FetchLiteMsgsRequest request;
  vis::vantage::FetchLiteMsgsResponse response;
  if (channel.has_value()) {
    request.set_use_msg_channel(true);
    request.mutable_channels()->Add(std::string(*channel));
  }
  vantage_client_man::FetchLiteMsgs(request, &response);
  const auto& msgs = response.lite_msgs();
  QLOG_EVERY_N_SEC(INFO, 1) << "msg :" << response.DebugString();
  auto it = std::find_if(msgs.begin(), msgs.end(), [msg_type](const auto& msg) {
    return msg.lite_msg_case() == msg_type;
  });
  if (it != msgs.end()) {
    return *it;
  }
  return absl::NotFoundError(
      absl::StrCat("Cannot find remote message: ", msg_type));
}

absl::StatusOr<CoordinateConverter> FetchVantageCoordinateConverter() {
  auto msg_or = FetchVantageLiteMsg(
      LiteMsgWrapper::LiteMsgCase::kLocalizationTransformProto,
      "localization_transform_proto");
  if (msg_or.ok()) {
    CoordinateConverter remote_coordinate =
        CoordinateConverter::FromLocalizationTransform(
            msg_or->localization_transform_proto());
    QLOG_EVERY_N_SEC(INFO, 1)
        << "Fetch localization_transform_proto success,Global proto:"
        << msg_or->localization_transform_proto().DebugString();
    return remote_coordinate;
  }
  return msg_or.status();
}

void SendRouteLaneGraphToCanvas(const v2::LaneGraph& lane_graph,
                                const mapping::v2::SemanticMapManager& smm,
                                const RouteSectionsInfo& sections_info,
                                const std::string& topic) {
  QCHECK(!topic.empty()) << "empty canvas channel name!";

  auto coordinate_or = qcraft::planner::FetchVantageCoordinateConverter();
  qcraft::CoordinateConverter cc =
      coordinate_or.ok() ? std::move(coordinate_or).value()
                         : qcraft::CoordinateConverter::FromLocale();
  auto& canvas = vis::vantage::GetCanvasClient()->GetCanvas(topic);
  canvas.SetGroundZero(1);

  const auto& layers = lane_graph.layers;
  const auto& sections = sections_info.section_segments();
  absl::flat_hash_map<v2::LaneGraph::VertexId, Vec2d> vertex_pos;

  for (int i = 0; i < layers.size(); ++i) {
    const auto [sec_idx, frac] = layers[i];
    for (const auto lane_id : sections[sec_idx].lane_ids) {
      ASSIGN_OR_CONTINUE(
          const auto global_pos,
          ComputeLanePointGlobalPose(smm, mapping::LanePoint(lane_id, frac)));
      const Vec2d pos = cc.GlobalToSmooth(global_pos);
      canvas.DrawPoint(Vec3d(pos, 0.1), vis::Color::kLightBlue, 8);
      vertex_pos[v2::ToVertId(i, lane_id)] = pos;
    }
    const auto rightmost_id = sections[sec_idx].lane_ids.back();
    ASSIGN_OR_CONTINUE(const auto heading,
                       ComputeLanePointGlobalHeading(
                           smm, mapping::LanePoint(rightmost_id, frac)));
    const double theta = (-heading.Perp()).FastAngle();
    canvas.DrawText(absl::StrFormat("Layer %d", i),
                    Vec3d(vertex_pos[v2::ToVertId(i, rightmost_id)], 0.1),
                    theta, 0.3, vis::Color::kLightRed);
  }

  const auto graph = lane_graph.graph;
  for (const auto& vertex : graph.vertices()) {
    if (!vertex_pos.contains(vertex)) continue;
    for (const auto& [to_id, cost] : graph.edges_from(vertex)) {
      if (!vertex_pos.contains(to_id)) continue;

      const auto& from_pos = vertex_pos[vertex];
      const auto& to_pos = vertex_pos[to_id];
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

}  // namespace planner
}  // namespace qcraft
