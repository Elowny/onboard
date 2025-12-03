#include "onboard/planner/assist/alcc_target_selector.h"

#include <algorithm>
#include <cmath>
#include <complex>
#include <map>
#include <optional>
#include <ostream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/types/span.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/frenet_frame.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/geometry/util.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/common/plot_util.h"
#include "onboard/planner/object/proto/planner_object.pb.h"
#include "onboard/planner/object/spacetime_object_state.h"
#include "onboard/planner/object/spacetime_object_trajectory.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/util/vehicle_geometry_util.h"

namespace qcraft {
namespace planner {

namespace {
DEFINE_bool(planner_alcc_draw_motion_path_sl_and_objects, false,
            "Whether to visualize motion path sl.");

const PiecewiseLinearFunction<double, double> kMotionPathSlShrinkBufferTimePLF(
    std::vector<double>{0.0, 2.0, 5.0, 6.0},
    std::vector<double>{0.4, 0.3, 0.2, 0.1});

constexpr double kTargetBoundaryEpsilon = 0.1;  // m.

absl::StatusOr<PathSlBoundary> BuildMotionDependentPathSlBoundary(
    const PathSlBoundary& map_path_sl, const DrivePassage& drive_passage,
    const ApolloTrajectoryPointProto& plan_start_point,
    const VehicleGeometryParamsProto& vehicle_geom) {
  FUNC_QTRACE();
  if (drive_passage.frenet_frame() == nullptr) {
    return absl::InternalError(
        "AlccTargetSelector: empty ptr from dp frenet frame.");
  }
  const int n = map_path_sl.size();
  std::vector<double> s_vec, ref_center_l_vec, right_l_vec, left_l_vec,
      target_right_l_vec, target_left_l_vec;
  const auto& map_s_vec = map_path_sl.s_vector();
  const auto& map_ref_center_l_vec = map_path_sl.reference_center_l_vector();
  const auto& map_target_right_l_vec = map_path_sl.target_right_l_vector();
  const auto& map_target_left_l_vec = map_path_sl.target_left_l_vector();
  s_vec.reserve(n);
  ref_center_l_vec.reserve(n);
  right_l_vec.reserve(n);
  left_l_vec.reserve(n);
  target_right_l_vec.reserve(n);
  target_left_l_vec.reserve(n);

  std::vector<Vec2d> target_right_xys, target_left_xys, right_xys, left_xys,
      ref_center_xy;
  const auto& map_ref_center_xy = map_path_sl.reference_center_xy_vector();
  target_right_xys.reserve(n);
  target_left_xys.reserve(n);
  right_xys.reserve(n);
  left_xys.reserve(n);
  ref_center_xy.reserve(n);

  const auto plan_start_pos = Vec2dFromPathPoint(plan_start_point.path_point());
  const auto av_box = ComputeAvBox(
      plan_start_pos, plan_start_point.path_point().theta(), vehicle_geom);
  const auto av_fbox_or = drive_passage.QueryFrenetBoxAt(av_box);
  if (!av_fbox_or.ok()) {
    return absl::InternalError(
        "AlccTargetSelector/BuildMotionDependentPathSlBoundary: av fbox "
        "compute error.");
  }

  // Same reference center. Only modify target boundary by reducing the width.
  for (int i = 0; i < n; ++i) {
    constexpr double kMinNominalSpeed = 5.0;
    const double nominal_t =
        map_s_vec[i] / std::max(kMinNominalSpeed, plan_start_point.v());
    const double target_path_shrink_buffer =
        kMotionPathSlShrinkBufferTimePLF(nominal_t);

    // DrivePassage and PathSlBoundary sl coordinate match.
    const auto& cur_center = map_ref_center_xy[i];
    const auto cur_sl_or = drive_passage.QueryFrenetCoordinateAt(cur_center);
    if (!cur_sl_or.ok()) {
      return absl::InternalError(absl::StrFormat(
          "AlccTargetSelector/BuildMotionDependentPathSlBoundary: failed to "
          "compute sl coord at %s.",
          cur_center.DebugString()));
    }
    const auto& cur_s = cur_sl_or->s;
    const double target_right_l =
        std::min<double>(map_target_right_l_vec[i] + target_path_shrink_buffer,
                         av_fbox_or->l_min - kTargetBoundaryEpsilon);
    const double target_left_l =
        std::max<double>(map_target_left_l_vec[i] - target_path_shrink_buffer,
                         av_fbox_or->l_max + kTargetBoundaryEpsilon);
    const auto target_right_xy_or =
        drive_passage.QueryPointXYAtSL(cur_s, target_right_l);
    const auto target_left_xy_or =
        drive_passage.QueryPointXYAtSL(cur_s, target_left_l);
    if (!target_right_xy_or.ok() || !target_left_xy_or.ok()) {
      return absl::InternalError(
          "AlccTargetSelector/BuildMotionDependentPathSlBoundary: fail to "
          "compute target left/right xy.");
    }

    // Do not change reference center.
    ref_center_xy.push_back(map_ref_center_xy[i]);
    ref_center_l_vec.push_back(map_ref_center_l_vec[i]);
    s_vec.push_back(cur_s);

    // Same inner boundary and outer boundary.
    right_l_vec.push_back(target_right_l);
    target_right_l_vec.push_back(target_right_l);
    left_l_vec.push_back(target_left_l);
    target_left_l_vec.push_back(target_left_l);
    right_xys.push_back(*target_right_xy_or);
    left_xys.push_back(*target_left_xy_or);
    target_right_xys.push_back(*target_right_xy_or);
    target_left_xys.push_back(*target_left_xy_or);
  }

  return PathSlBoundary(
      std::move(s_vec), std::move(ref_center_l_vec), std::move(right_l_vec),
      std::move(left_l_vec), std::move(target_right_l_vec),
      std::move(target_left_l_vec), std::move(ref_center_xy),
      std::move(right_xys), std::move(left_xys), std::move(target_right_xys),
      std::move(target_left_xys));
}

inline bool IsBox2dOverlapWithTargetBoundary(const PathSlBoundary& path_sl,
                                             const FrenetFrame& path_ff,
                                             const Box2d& box) {
  const double s_offset = path_sl.start_s() - path_ff.start_s();
  const auto fbox_or =
      path_ff.QueryLongitudinallyBoundedFrenetBoxAtContour(Polygon2d(box));
  if (!fbox_or.ok()) return false;
  const auto center_sl = fbox_or->center();
  const auto target_bound_l =
      path_sl.QueryTargetBoundaryL(center_sl.s + s_offset);
  return (fbox_or->l_min <= target_bound_l.second &&
          fbox_or->l_max >= target_bound_l.first);
}

struct OverlapState {
  int index;
  double relative_time;
  const SpacetimeObjectState* state = nullptr;
};

std::optional<OverlapState>
EstimateSpacetimeObjectTrajectoryEnterTargetBoundaryState(
    const PathSlBoundary& path_sl, const FrenetFrame& path_ff,
    const SpacetimeObjectTrajectory& st_traj) {
  const auto& states = st_traj.states();
  // Find the earliest state.
  for (int i = 0; i < states.size(); ++i) {
    const auto& box = states[i].box;
    if (IsBox2dOverlapWithTargetBoundary(path_sl, path_ff, box)) {
      return OverlapState{
          .index = i,
          .relative_time = i * kTrajectoryTimeStep,
          .state = &states[i],
      };
    }
  }
  return std::nullopt;
}
}  // namespace

absl::Status SelectAlccTarget(
    const PathSlBoundary& map_path_sl, const DrivePassage& drive_passage,
    const ApolloTrajectoryPointProto& plan_start_point,
    const VehicleGeometryParamsProto& vehicle_geom,
    SpacetimeTrajectoryManager* st_traj_mgr) {
  FUNC_QTRACE();
  QCHECK_NOTNULL(st_traj_mgr);
  // 1.0 Get frenet frame from DrivePassage.
  const auto* dp_ff_ptr = drive_passage.frenet_frame();
  if (dp_ff_ptr == nullptr) {
    return absl::InternalError(
        "AlccTargetSelector: empty ptr from dp frenet frame.");
  }
  const auto last_real_station_idx = drive_passage.last_real_station_index();
  const auto length_on_map =
      drive_passage.station(last_real_station_idx).accumulated_s() -
      drive_passage.front_s();

  // 2.0 Build new path sl boundary based on Av shape and motion.
  const auto motion_path_sl_or = BuildMotionDependentPathSlBoundary(
      map_path_sl, drive_passage, plan_start_point, vehicle_geom);
  if (!motion_path_sl_or.ok()) {
    return absl::InternalError(
        "AlccTargetSelector: build motion path sl failed.");
  }
  const auto& motion_path_sl = *motion_path_sl_or;
  if (FLAGS_planner_alcc_draw_motion_path_sl_and_objects) {
    // ClearCanvasServerBuffers();
    DrawPathSlBoundaryToCanvas(motion_path_sl, "alcc/motion_path_sl");
  }

  // 3.0 Iterate spacetime object trajectory, check relative time for each
  // object to enter path boundaries (generated by map path boundaries and
  // buffer dependent of s coordinate and the speed of AV).
  std::map<std::string, FilterReason::Type> id_reason_map;
  {
    SCOPED_QTRACE_ARG1("AlccTargetSelector::FilterStTrajs", "num",
                       st_traj_mgr->trajectories().size());
    for (const auto& st_traj : st_traj_mgr->trajectories()) {
      if (st_traj.is_stationary()) {
        // Do not filter out stationary objects.
        continue;
      }
      // Get first enter target bound object state.
      const auto enter_target_bound_state_opt =
          EstimateSpacetimeObjectTrajectoryEnterTargetBoundaryState(
              motion_path_sl, *dp_ff_ptr, st_traj);
      if (!enter_target_bound_state_opt.has_value()) {
        continue;
      }
      const auto& object_box = enter_target_bound_state_opt->state->box;
      const auto& obj_box_center = object_box.center();
      const auto obj_box_center_sl = dp_ff_ptr->XYToSL(obj_box_center);
      const auto path_tangent =
          dp_ff_ptr->InterpolateTangentByS(obj_box_center_sl.s);
      const double heading_diff =
          NormalizeAngle(object_box.heading() - path_tangent.FastAngle());
      constexpr double kEnterTargetBoundHeadingDiffThreshold =
          M_PI / 2;  // rad.
      if (std::fabs<double>(heading_diff) >
          kEnterTargetBoundHeadingDiffThreshold) {
        // Do not filter any objects if heading diff with path exceeds 90 deg
        // for safety. Maybe oncoming objects.
        continue;
      }

      // 3.1 Filter trajectories entering target bound too late.
      constexpr double kEnterTargetBoundTimeThreshold = 5.0;  // s.
      VLOG(1) << absl::StrFormat(
          "Traj id %s enter target boundary in %.3f second.", st_traj.traj_id(),
          enter_target_bound_state_opt->relative_time);
      if (enter_target_bound_state_opt->relative_time >
          kEnterTargetBoundTimeThreshold) {
        id_reason_map.insert(
            std::make_pair(std::string(st_traj.traj_id()),
                           FilterReason::ST_TRAJ_ENTER_TARGET_BOUND_TOO_LATE));
        continue;
      }
      // 3.2 Filter first overlap state beyond online map (in extended drive
      // passage part). Both prediction traj and planning path may not be
      // accurate.
      constexpr double kBeyondMapLookAheadTime = 1.0;  // s.
      const double protective_length_on_map =
          length_on_map + kBeyondMapLookAheadTime * plan_start_point.v();
      if (obj_box_center_sl.s - dp_ff_ptr->start_s() >
          protective_length_on_map) {
        id_reason_map.insert(std::make_pair(
            std::string(st_traj.traj_id()),
            FilterReason::ST_TRAJ_ENTER_TARGET_BOUND_BEYOND_MAP));
        VLOG(1) << absl::StrFormat(
            "Traj id %s enter target boundary at s: %.5f. Length on map with "
            "protective length: %.3f.",
            st_traj.traj_id(), obj_box_center_sl.s - dp_ff_ptr->start_s(),
            protective_length_on_map);
        continue;
      }
    }
  }
  // Modify spacetime trajectory manager by ids to filter and corresponding
  // reasons.
  if (!id_reason_map.empty()) {
    st_traj_mgr->FilterTrajectoriesWithReason(id_reason_map);
  }
  return absl::OkStatus();
}
}  // namespace planner
}  // namespace qcraft
