#include "onboard/planner/decision/stop_polyline_decider.h"

#include <stdint.h>

#include <algorithm>
#include <cmath>
#include <utility>

#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "google/protobuf/repeated_ptr_field.h"

#include "common/proto/map_geometry.pb.h"

#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/math/coordinate_converter.h"
#include "onboard/math/geometry/halfplane.h"
#include "onboard/math/geometry/polyline2d.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/utils/status_macros.h"

namespace qcraft::planner {
namespace {

struct StopPointInfo {
  Vec2d smooth_coordinate;
  double s_on_drive_passage;
};

bool IsSameStopPoint(const DrivePassage& passage, const Vec2d& last_stop_point,
                     double cur_stop_point_s) {
  const auto last_stop_point_sl_or =
      passage.QueryFrenetCoordinateAt(last_stop_point);
  if (!last_stop_point_sl_or.ok()) return false;

  constexpr double kSameStopPointDistThreshold = 1.0;  // m
  return std::fabs(last_stop_point_sl_or->s - cur_stop_point_s) <
         kSameStopPointDistThreshold;
}

absl::StatusOr<StopPointInfo> CalculateStopPointInfo(
    const DrivePassage& passage,
    const CoordinateConverter& coordinate_converter,
    const FrenetBox& ego_frenet_box, const mapping::LineProto& line) {
  if (line.type() != mapping::LineProto::STOP_LINE) {
    return absl::InvalidArgumentError(
        absl::StrCat("Lane type: ", mapping::LineProto::Type_Name(line.type()),
                     " is not STOP_LINE type."));
  }

  // Convert geo points to smooth points.
  const auto& geo_points = line.polyline().points();
  std::vector<Vec2d> smooth_points;
  smooth_points.reserve(geo_points.size());
  for (const auto& geo_pt : geo_points) {
    smooth_points.push_back(coordinate_converter.GlobalToSmooth(
        Vec2d(geo_pt.longitude(), geo_pt.latitude())));
  }

  // Resample smooth points.
  if (smooth_points.size() < 2) {
    return absl::InvalidArgumentError(
        absl::StrCat("The points num of stop line is ", smooth_points.size(),
                     " less than 2, can not resample."));
  }
  const Polyline2d polyline(std::move(smooth_points));
  constexpr double kStopLineSampleStep = 0.5;  // m
  std::vector<Vec2d> resample_smooth_points;
  const int resample_size =
      CeilToInt(polyline.length() / kStopLineSampleStep) + 1;
  resample_smooth_points.reserve(resample_size);
  for (double s = 0.0; s < polyline.length() + kStopLineSampleStep;
       s += kStopLineSampleStep) {
    resample_smooth_points.push_back(
        polyline.Sample(std::min<double>(s, polyline.length())));
  }

  // TODO(jiayu): This is a lazy method to calc stop line s, optimize later.
  for (int i = 0; i < resample_smooth_points.size() - 1; ++i) {
    const auto& cur_pt_smooth = resample_smooth_points[i];
    const auto& next_pt_smooth = resample_smooth_points[i + 1];
    ASSIGN_OR_CONTINUE(const auto cur_pt_sl,
                       passage.QueryFrenetCoordinateAt(cur_pt_smooth));
    ASSIGN_OR_CONTINUE(const auto next_pt_sl,
                       passage.QueryFrenetCoordinateAt(next_pt_smooth));

    // Ignore stop line which behind av.
    const auto stop_point_s = (cur_pt_sl.s + next_pt_sl.s) / 2.0;
    if (stop_point_s < ego_frenet_box.s_min) continue;

    if (cur_pt_sl.l * next_pt_sl.l < 0.0) {
      return StopPointInfo{
          .smooth_coordinate = (cur_pt_smooth + next_pt_smooth) / 2.0,
          .s_on_drive_passage = stop_point_s};
    }
  }

  return absl::NotFoundError(absl::StrCat("Can not find stop point for line ",
                                          line.id(), " on drive passage."));
}

absl::StatusOr<ConstraintProto::StopLineProto> CalculateStopLineAtS(
    const DrivePassage& passage, double s, int64_t id) {
  ASSIGN_OR_RETURN(const auto curbs, passage.QueryCurbPointAtS(s));

  constexpr double kStandoff = 1.0;  // m.
  ConstraintProto::StopLineProto stop_line;
  stop_line.set_s(s);
  stop_line.set_standoff(kStandoff);
  stop_line.set_time(0.0);
  HalfPlane halfplane(curbs.first, curbs.second);
  halfplane.ToProto(stop_line.mutable_half_plane());
  stop_line.set_id(absl::StrCat("stop_polyline", id));
  stop_line.mutable_source()->mutable_stop_polyline()->set_id(id);
  return stop_line;
}
}  // namespace

absl::StatusOr<StopPolylineDeciderOutput> BuildStopPolylineConstraints(
    const PlannerSemanticMapManager& psmm, const DrivePassage& passage,
    const FrenetBox& ego_frenet_box,
    const StopPolylineDeciderStateProto& decider_state,
    bool is_pass_nearest_stop_line) {
  std::vector<ConstraintProto::StopLineProto> stop_lines;
  StopPolylineDeciderStateProto new_decider_state;

  const auto push_stop_line_by_s =
      [&stop_lines](ConstraintProto::StopLineProto stop_line) {
        const auto it =
            std::lower_bound(stop_lines.begin(), stop_lines.end(), stop_line,
                             [](const ConstraintProto::StopLineProto& elem,
                                const ConstraintProto::StopLineProto& val) {
                               return elem.s() < val.s();
                             });
        stop_lines.insert(it, std::move(stop_line));
      };

  Vec2d nearest_stop_point;
  const auto& lines = psmm.semantic_map_proto().lines();
  for (const auto& line : lines) {
    // Calc stop point s and smooth coordinate.
    ASSIGN_OR_CONTINUE(
        auto stop_point_info,
        CalculateStopPointInfo(passage, psmm.coordinate_converter(),
                               ego_frenet_box, line));

    // Ignore stop point near passable stop point.
    if (decider_state.has_passable_stop_point() &&
        IsSameStopPoint(passage, Vec2d(decider_state.passable_stop_point()),
                        stop_point_info.s_on_drive_passage)) {
      stop_point_info.smooth_coordinate.ToProto(
          new_decider_state.mutable_passable_stop_point());
      continue;
    }

    // Calc stop line constraint at s on drive passage.
    ASSIGN_OR_CONTINUE(
        auto stop_line,
        CalculateStopLineAtS(passage, stop_point_info.s_on_drive_passage,
                             line.id()));

    // Record nearest stop point.
    if (stop_lines.empty() ||
        stop_point_info.s_on_drive_passage < stop_lines.front().s()) {
      nearest_stop_point = stop_point_info.smooth_coordinate;
    }

    // Record all stop line.
    push_stop_line_by_s(std::move(stop_line));
  }

  // Record nearest stop point which to be passed.
  if (is_pass_nearest_stop_line &&
      !new_decider_state.has_passable_stop_point() && !stop_lines.empty()) {
    nearest_stop_point.ToProto(new_decider_state.mutable_passable_stop_point());
    stop_lines.erase(stop_lines.begin());
  }

  return StopPolylineDeciderOutput{
      .stop_lines = std::move(stop_lines),
      .stop_polyline_state = std::move(new_decider_state)};
}
}  // namespace qcraft::planner
