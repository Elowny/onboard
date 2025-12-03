#include "onboard/planner/common/map_path_generator.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <utility>

#include "absl/hash/hash.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/types/span.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

#include "onboard/container/strong_int.h"
#include "onboard/global/buffered_logger.h"
#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/maps_common.h"
#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/cubic_spline.h"
#include "onboard/math/fast_math.h"
#include "onboard/math/frenet_cartesian_converter.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/geometry/segment2d.h"
#include "onboard/math/gradient_points_smoother.h"
#include "onboard/math/polynomial_connector.h"
#include "onboard/math/polynomialxd.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/common/path_generator_util.h"
#include "onboard/planner/math/cubic_spiral_boundary_value_problem.h"
#include "onboard/planner/math/spiral.h"
#include "onboard/planner/util/lane_point_util.h"
#include "onboard/planner/util/path_util.h"
#include "onboard/planner/util/spatial_search_util.h"
#include "onboard/utils/map_util.h"
#include "onboard/utils/status_macros.h"
#include "onboard/vis/canvas/canvas.h"
#include "onboard/vis/common/color.h"

#include "offboard/vis/vantage/vantage_server/vantage_canvas_client.h"

DEFINE_bool(draw_coarse_path_canvas, false,
            "Whether draw coarse and smooth path on canvas.");

namespace qcraft {
namespace planner {

namespace {

using mapping::LaneInfo;

constexpr double kMotionPreviewTime = 1.0;      // s.
constexpr double kMinMotionPreviewSpeed = 2.0;  // m/s.
constexpr double kCoarseSampleStep = 1.0;       // m.
constexpr int kPolyDegree = 5;
constexpr int kMinPointsSize = 2;
constexpr double kPathMaxLateralAccLimit = 3.0;  // m/s^2.

// It's not possible to constrain theta & kappa of the start point
// simultaneously for cubic splines. In order to get a better approximation of
// the start point, We extend coarse path reversely for a certain distance. By
// extending the coarse path reversely by three points, the errors in theta and
// kappa can be reduced to about 1e-3.
constexpr int kReverseExtensionPointsNum = 3;

constexpr double kEps = 1e-6;

inline double ComputeCurvature(double dx, double dy, double ddx, double ddy) {
  const double base = dx * dx + dy * dy;
  const double denominator = base * std::sqrt(base);
  const double nominator = dx * ddy - dy * ddx;
  return std::abs(denominator) < kEps ? 0.0 : nominator / denominator;
}

inline PathPoint EvaluatePathPointOnCubicSplines(const CubicSpline& x_s,
                                                 const CubicSpline& y_s,
                                                 double s) {
  const double dx = x_s.EvaluateDerivative<1>(s);
  const double dy = y_s.EvaluateDerivative<1>(s);
  const double ddx = x_s.EvaluateDerivative<2>(s);
  const double ddy = y_s.EvaluateDerivative<2>(s);
  PathPoint p;
  p.set_x(x_s.Evaluate(s));
  p.set_y(y_s.Evaluate(s));
  p.set_theta(fast_math::Atan2(dy, dx));
  p.set_s(s);
  p.set_kappa(ComputeCurvature(dx, dy, ddx, ddy));
  p.set_lambda(0.0);
  return p;
}

absl::StatusOr<PathPoint> EvaluatePathPointOnDrivePassage(
    const DrivePassage& drive_passage, double target_s) {
  FUNC_QTRACE();
  constexpr double kExtendDistance = 20.0;  // m.
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> s;
  const int extend_size = CeilToInt(kExtendDistance / kCoarseSampleStep) + 1;
  x.reserve(extend_size);
  y.reserve(extend_size);
  s.reserve(extend_size);
  const double start_s =
      std::max(target_s - 0.5 * kExtendDistance, drive_passage.front_s());
  const double end_s =
      std::min(target_s + 0.5 * kExtendDistance, drive_passage.end_s());
  if (start_s > target_s || target_s > end_s) {
    return absl::NotFoundError(
        "[Path Generator]: Cannot find target path point.");
  }
  double current_s = target_s;
  while (current_s >= start_s) {
    ASSIGN_OR_BREAK(const auto point, drive_passage.QueryPointXYAtS(current_s));
    s.push_back(current_s);
    x.push_back(point.x());
    y.push_back(point.y());
    current_s -= kCoarseSampleStep;
  }
  std::reverse(x.begin(), x.end());
  std::reverse(y.begin(), y.end());
  std::reverse(s.begin(), s.end());
  current_s = target_s + kCoarseSampleStep;
  while (current_s <= end_s) {
    ASSIGN_OR_BREAK(const auto point, drive_passage.QueryPointXYAtS(current_s));
    s.push_back(current_s);
    x.push_back(point.x());
    y.push_back(point.y());
    current_s += kCoarseSampleStep;
  }
  if (s.size() < kMinPointsSize || s.back() < target_s) {
    return absl::NotFoundError(
        "[Path Generator]: Cannot find target path point.");
  }
  const CubicSpline x_s(s, x);
  const CubicSpline y_s(s, y);
  auto target_path_point = EvaluatePathPointOnCubicSplines(x_s, y_s, target_s);
  // Use min abs kappa in kKappaSearchRange for target path point.
  double target_kappa = target_path_point.kappa();
  constexpr double kKappaSearchRange = 3.0;  // m.
  current_s = std::max(s.front(), target_s - kKappaSearchRange);
  const double kappa_search_end_s =
      std::min(s.back(), target_s + kKappaSearchRange);
  while (current_s <= kappa_search_end_s) {
    current_s += kCoarseSampleStep;
    const double dx = x_s.EvaluateDerivative<1>(current_s);
    const double dy = y_s.EvaluateDerivative<1>(current_s);
    const double ddx = x_s.EvaluateDerivative<2>(current_s);
    const double ddy = y_s.EvaluateDerivative<2>(current_s);
    const double kappa = ComputeCurvature(dx, dy, ddx, ddy);
    if (std::abs(kappa) < std::abs(target_kappa)) {
      target_kappa = kappa;
    }
  }
  target_path_point.set_kappa(target_kappa);
  return target_path_point;
}

void DrawPathPointsOnCanvas(const std::vector<double>& x,
                            const std::vector<double>& y,
                            const vis::Color& color, const std::string& name) {
  std::vector<Vec3d> points;
  points.reserve(x.size());
  for (int i = 0; i < x.size(); ++i) {
    points.emplace_back(x[i], y[i], 0.0);
  }
  auto& canvas = vis::vantage::GetCanvasClient()->GetCanvas(
      absl::StrCat("emergency_brake/", name));
  canvas.SetGroundZero(1);
  canvas.DrawLineStrip(points, color, /*line_width=*/4);
  vis::vantage::GetCanvasClient()->FlushAll();
}

void ExtendPathByLastTheta(std::vector<double>* x, std::vector<double>* y,
                           std::vector<double>* s, double path_length) {
  QCHECK_EQ(x->size(), y->size());
  QCHECK_EQ(y->size(), s->size());
  if (x->size() < 3 || s->back() > path_length) {
    return;
  }

  const auto& step = Segment2d(Vec2d((*x)[x->size() - 2], (*y)[y->size() - 2]),
                               Vec2d(x->back(), y->back()))
                         .unit_direction() *
                     kCoarseSampleStep;
  const int extend_size =
      FloorToInt((path_length - s->back()) / kCoarseSampleStep) + 1;
  for (int i = 0; i < extend_size; ++i) {
    x->push_back(x->back() + step.x());
    y->push_back(y->back() + step.y());
    s->push_back(s->back() + kCoarseSampleStep);
  }
}

absl::Status GenerateLaneCenteringCoarsePath(const DrivePassage& drive_passage,
                                             const PathPoint& start_path_point,
                                             double speed, double path_length,
                                             std::vector<double>* x,
                                             std::vector<double>* y,
                                             std::vector<double>* s) {
  FUNC_QTRACE();
  QCHECK_NOTNULL(x);
  QCHECK_NOTNULL(y);
  QCHECK_NOTNULL(s);
  x->clear();
  y->clear();
  s->clear();
  const int coarse_path_size = CeilToInt(path_length / kCoarseSampleStep) + 1;
  x->reserve(coarse_path_size);
  y->reserve(coarse_path_size);
  s->reserve(coarse_path_size);
  x->push_back(start_path_point.x());
  y->push_back(start_path_point.y());
  s->push_back(0.0);
  // Lane centering trajectory by quintic polynomial in Frenet coordinates.
  ASSIGN_OR_RETURN(
      const auto start_frenet_coord,
      drive_passage.QueryFrenetCoordinateAt(ToVec2d(start_path_point)));
  ASSIGN_OR_RETURN(
      const auto refline_start_point,
      EvaluatePathPointOnDrivePassage(drive_passage, start_frenet_coord.s));
  const SecondOrderCartesianCoordinate start_point_cartesian = {
      .x = start_path_point.x(),
      .y = start_path_point.y(),
      .theta = start_path_point.theta(),
      .kappa = start_path_point.kappa()};
  ASSIGN_OR_RETURN(
      const auto start_point_frenet,
      CartesianToFrenet(refline_start_point.s(), refline_start_point.x(),
                        refline_start_point.y(), refline_start_point.theta(),
                        refline_start_point.kappa(),
                        refline_start_point.lambda(), start_point_cartesian));
  constexpr double kBackToCenterDlThres = 0.025;
  constexpr double kMaxBackTime = 5.0;
  constexpr double kMinBackTime = 2.0;
  double poly_end_l, poly_end_s;
  if (std::abs(start_point_frenet.dl) > kBackToCenterDlThres &&
      start_point_frenet.dl * start_point_frenet.l < 0.0) {
    poly_end_l = 0.0;
    const double back_to_center_time =
        std::clamp(std::abs(start_point_frenet.l) /
                       std::max(speed * std::abs(start_point_frenet.dl), kEps),
                   kMinBackTime, kMaxBackTime);
    poly_end_s = back_to_center_time * speed + start_frenet_coord.s;
  } else {
    poly_end_l = start_point_frenet.l;
    constexpr double kComfortLateralAcc = 0.5;  // m/s2.
    const double back_to_straight_time =
        std::clamp(speed * std::abs(start_point_frenet.dl) / kComfortLateralAcc,
                   kMinBackTime, kMaxBackTime);
    poly_end_s = back_to_straight_time * speed + start_frenet_coord.s;
  }
  ASSIGN_OR_RETURN(const auto poly_coef,
                   ConnectByQuinticPolynomial(
                       start_point_frenet.s, start_point_frenet.l,
                       start_point_frenet.dl, start_point_frenet.d2l,
                       poly_end_s, poly_end_l, /*dy1=*/0.0, /*ddy1=*/0.0));
  const auto quintic_poly = PolynomialXd<kPolyDegree>(poly_coef);

  double next_s = start_frenet_coord.s + s->back();
  while (s->back() < path_length) {
    next_s += kCoarseSampleStep;
    const double next_l =
        next_s < poly_end_s ? quintic_poly.Evaluate(next_s) : poly_end_l;
    ASSIGN_OR_BREAK(const Vec2d point,
                    drive_passage.QueryPointXYAtSL(next_s, next_l));
    s->push_back(s->back() +
                 std::hypot(point.x() - x->back(), point.y() - y->back()));
    x->push_back(point.x());
    y->push_back(point.y());
  }
  return absl::OkStatus();
}

inline bool IsOppositeLaneBoundary(
    const mapping::LaneBoundaryInfo* lane_boundary) {
  using mapping::LaneBoundaryProto;
  if (nullptr == lane_boundary) return false;
  return lane_boundary->type == LaneBoundaryProto::SOLID_DOUBLE_YELLOW ||
         lane_boundary->type == LaneBoundaryProto::SOLID_YELLOW ||
         lane_boundary->type == LaneBoundaryProto::CURB;
}

const LaneInfo* GetNearestValidLaneInfo(
    const PlannerSemanticMapManager& psmm, const PathPoint& preview_path_point,
    double max_search_radius, double max_heading_diff,
    absl::flat_hash_map<mapping::ElementId, bool>* lane_attr_map) {
  const auto preview_pos = ToVec2d(preview_path_point);
  auto close_points =
      FindCloseLanePointsAndDistanceToSmoothPointWithHeadingBoundAmongLanesAtLevel(  // NOLINT
          psmm.GetLevel(), psmm, preview_pos, preview_path_point.theta(),
          /*heading_penalty_weight=*/0.0, max_search_radius, max_heading_diff);
  // close_points have been sorted by distance.
  for (const auto& close_point : close_points) {
    const auto* lane_info =
        psmm.FindLaneInfoOrNull(close_point.second.lane_id());
    if (nullptr == lane_info ||
        (lane_info->proto->has_is_virtual() &&
         lane_info->proto->is_virtual()) ||
        IsOppositeLane(psmm, lane_info, lane_attr_map)) {
      continue;
    }
    return lane_info;
  }
  return nullptr;
}

struct LanePreviewResult {
  double preview_s;
  PathPoint preview_path_point;
  const LaneInfo* preview_lane_info = nullptr;
};

std::optional<LanePreviewResult> GetBestLaneInfoWithOneMoreStep(
    const PlannerSemanticMapManager& psmm, const PathPoint& preview_path_point,
    double preview_s, double preview_step, double max_length,
    double max_search_radius, double max_heading_diff,
    absl::flat_hash_map<mapping::ElementId, bool>* lane_attr_map,
    std::optional<LanePreviewResult>* candidate_preview_result) {
  QCHECK_NOTNULL(candidate_preview_result);
  *candidate_preview_result = std::nullopt;
  const LaneInfo* lane_info =
      GetNearestValidLaneInfo(psmm, preview_path_point, max_search_radius,
                              max_heading_diff, lane_attr_map);
  if (nullptr == lane_info) return std::nullopt;

  LanePreviewResult res{.preview_s = preview_s,
                        .preview_path_point = preview_path_point,
                        .preview_lane_info = lane_info};
  const double next_preview_s = preview_s + preview_step;
  if (next_preview_s > max_length) return res;
  const PathPoint next_preview_path_point =
      GetPathPointAlongCircle(preview_path_point, preview_step);
  const LaneInfo* next_lane_info =
      GetNearestValidLaneInfo(psmm, next_preview_path_point, max_search_radius,
                              max_heading_diff, lane_attr_map);

  if (nullptr == next_lane_info || next_lane_info == lane_info) return res;

  *candidate_preview_result =
      LanePreviewResult{.preview_s = next_preview_s,
                        .preview_path_point = next_preview_path_point,
                        .preview_lane_info = next_lane_info};
  // Compare current lane info and next step lane info.
  struct LaneProjectionResult {
    double dist;
    double rel_heading;
  };

  const auto get_lane_projection_result =
      [&psmm](
          const PathPoint& path_point,
          mapping::ElementId lane_id) -> std::optional<LaneProjectionResult> {
    Vec2d closest_point;
    const Vec2d pos = ToVec2d(path_point);
    const auto lane_point =
        FindClosestLanePointToSmoothPointWithHeadingBoundOnLaneAtLevel(
            psmm.GetLevel(), psmm, pos, lane_id, path_point.theta(),
            /*heading_penalty_weight=*/0.0, &closest_point,
            /*start_fraction=*/0.0, /*end_fraction=*/1.0);
    if (!lane_point.has_value()) return std::nullopt;

    const double lane_heading =
        ComputeLanePointTangent(psmm, *lane_point).FastAngle();
    const double rel_heading =
        NormalizeAngle(path_point.theta() - lane_heading);
    const double dist = pos.DistanceTo(closest_point);
    return LaneProjectionResult{.dist = dist, .rel_heading = rel_heading};
  };

  const auto proj_res =
      get_lane_projection_result(preview_path_point, lane_info->id);
  if (!proj_res.has_value()) return res;
  const auto next_proj_res =
      get_lane_projection_result(next_preview_path_point, next_lane_info->id);
  if (!next_proj_res.has_value()) return res;
  const double dist_diff = next_proj_res->dist - proj_res->dist;
  const double heading_diff =
      std::abs(next_proj_res->rel_heading) - std::abs(proj_res->rel_heading);
  constexpr double kDistDiffMaxThres = -1.0;            // m.
  constexpr double kHeadingDiffMaxThres = M_PI / 18.0;  // 10 deg.
  // If the heading diff between these two lanes is similar and the distance to
  // the next lane is much smaller than the current lane, then choose the next
  // lane as the final preview result.
  if (dist_diff < kDistDiffMaxThres && heading_diff < kHeadingDiffMaxThres) {
    std::swap(res, **candidate_preview_result);
  }
  return res;
}

absl::StatusOr<DiscretizedPath>
GeneratePreviewLaneKeepPathFromLanePreviewResult(
    const PlannerSemanticMapManager& psmm,
    const ApolloTrajectoryPointProto& plan_start_point,
    const LanePreviewResult& lane_preview_result, double path_length,
    double path_step, double path_min_length, double path_min_extension_time,
    double drive_passage_length_factor) {
  FUNC_QTRACE();
  const auto& start_path_point = plan_start_point.path_point();
  const int path_size = FloorToInt(path_length / path_step) + 1;
  DiscretizedPath path;
  path.reserve(path_size);

  const auto* lane_info = lane_preview_result.preview_lane_info;
  double preview_s = lane_preview_result.preview_s;
  auto preview_path_point = lane_preview_result.preview_path_point;

  if (nullptr == lane_info || lane_info->id == mapping::kInvalidElementId) {
    return absl::NotFoundError(
        "[Path Generator]: Cannot find preview nearest lane.");
  }

  const double speed = std::max(kMinMotionPreviewSpeed, plan_start_point.v());
  const double lane_center_path_s = path_length - preview_s;
  const auto drive_passage_opt = BuildDrivePassageFromLaneId(
      psmm, lane_info->id, preview_path_point,
      drive_passage_length_factor * lane_center_path_s);
  if (!drive_passage_opt.has_value()) {
    return absl::NotFoundError(
        "[Path Generator]: Preview drive passage building failed.");
  }

  // Give an extra distance to preview lane center point to prevent large kappa
  // in sprial curve.
  constexpr double kMaxPreviewExtraDistance = 10.0;  // m.
  double preview_end_s =
      drive_passage_opt->front_s() +
      std::min(speed * kMotionPreviewTime, kMaxPreviewExtraDistance);
  const auto preview_end_frenet_coord =
      drive_passage_opt->QueryUnboundedFrenetCoordinateAt(
          ToVec2d(preview_path_point));
  if (preview_end_frenet_coord.ok()) {
    constexpr double kMaxLatVel = 0.5;  // m/s.
    const double back_to_center_dis =
        std::abs(preview_end_frenet_coord->l) / kMaxLatVel * speed;
    preview_end_s = std::max(preview_end_frenet_coord->s + back_to_center_dis,
                             preview_end_s);
  }
  preview_end_s = std::min(preview_end_s, drive_passage_opt->end_s());

  ASSIGN_OR_RETURN(preview_path_point, EvaluatePathPointOnDrivePassage(
                                           *drive_passage_opt, preview_end_s));
  // Generate spiral path from start point to preview lane center.
  const SpiralPoint start_spiral_point = {.x = start_path_point.x(),
                                          .y = start_path_point.y(),
                                          .theta = start_path_point.theta(),
                                          .k = start_path_point.kappa(),
                                          .s = 0.0};
  const SpiralPoint end_spiral_point = {.x = preview_path_point.x(),
                                        .y = preview_path_point.y(),
                                        .theta = preview_path_point.theta(),
                                        .k = preview_path_point.kappa(),
                                        .s = 0.0};
  const auto spirals = CubicSpiralBoundaryValueSolver::solve(start_spiral_point,
                                                             end_spiral_point);
  if (spirals.empty()) {
    return absl::NotFoundError("[Path Generator]: No feasible spiral curve.");
  }
  const auto& spiral = *std::min_element(
      spirals.begin(), spirals.end(),
      [](const auto& s1, const auto& s2) { return s1.length() < s2.length(); });
  const int spiral_steps = CeilToInt(spiral.length() / kCoarseSampleStep);
  const auto spiral_points = spiral.BatchEval(spiral.length(), spiral_steps);
  preview_s = spiral.length();

  const int coarse_path_size = CeilToInt(path_length / kCoarseSampleStep) + 1 +
                               kReverseExtensionPointsNum;
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> s;
  x.reserve(coarse_path_size);
  y.reserve(coarse_path_size);
  s.reserve(coarse_path_size);
  for (int i = kReverseExtensionPointsNum; i > 0; --i) {
    const double current_s = -static_cast<double>(i) * kCoarseSampleStep;
    const auto path_point =
        GetPathPointAlongCircle(start_path_point, current_s);
    x.push_back(path_point.x());
    y.push_back(path_point.y());
    s.push_back(current_s);
  }
  for (int i = 0; i + 1 < spiral_points.size(); ++i) {
    x.push_back(spiral_points[i].x);
    y.push_back(spiral_points[i].y);
    s.push_back(spiral_points[i].s);
  }

  if (const double lc_path_s = path_length - preview_s;
      lc_path_s > kCoarseSampleStep) {
    std::vector<double> lc_x;
    std::vector<double> lc_y;
    std::vector<double> lc_s;
    // Lane keeping trajectory by quintic polynomial in Frenet coordinates.
    RETURN_IF_ERROR(
        GenerateLaneCenteringCoarsePath(*drive_passage_opt, preview_path_point,
                                        speed, lc_path_s, &lc_x, &lc_y, &lc_s));

    std::transform(lc_s.begin(), lc_s.end(), lc_s.begin(),
                   [&preview_s](double s) { return s + preview_s; });
    x.insert(x.end(), lc_x.begin(), lc_x.end());
    y.insert(y.end(), lc_y.begin(), lc_y.end());
    s.insert(s.end(), lc_s.begin(), lc_s.end());
  }

  if (s.size() < kMinPointsSize || s.back() < path_min_length) {
    return absl::NotFoundError(
        "[Path Generator]: Preview drive passage too short.");
  }

  const double max_check_distance =
      std::max(path_min_length, speed * path_min_extension_time);
  return GenerateSmoothPath(speed, path_step, max_check_distance,
                            kPathMaxLateralAccLimit, &x, &y, &s);
}

}  // namespace

bool IsOppositeLane(
    const PlannerSemanticMapManager& psmm, const LaneInfo* lane_info,
    absl::flat_hash_map<mapping::ElementId, bool>* lane_attr_map) {
  if (nullptr == lane_info) return false;
  if (const auto* is_opposite_lane =
          FindOrNull(*lane_attr_map, lane_info->id)) {
    return *is_opposite_lane;
  }
  (*lane_attr_map)[lane_info->id] = false;
  constexpr mapping::ElementId kInvalidLaneId(0);
  if (lane_info->lane_neighbors_on_right.empty() ||
      lane_info->lane_boundaries_on_right.empty() ||
      lane_info->lane_neighbors_on_right.front().other_id ==
          mapping::kInvalidElementId ||
      lane_info->lane_neighbors_on_right.front().other_id == kInvalidLaneId) {
    return false;
  }
  const mapping::LaneBoundaryInfo* right_lane_boundary =
      psmm.FindLaneBoundaryByIdOrNull(
          lane_info->lane_boundaries_on_right.front().lane_boundary_id);
  const LaneInfo* right_lane_info = psmm.FindLaneInfoOrNull(
      lane_info->lane_neighbors_on_right.front().other_id);
  if (IsOppositeLaneBoundary(right_lane_boundary) ||
      IsOppositeLane(psmm, right_lane_info, lane_attr_map)) {
    (*lane_attr_map)[lane_info->id] = true;
    return true;
  }
  return false;
}

absl::StatusOr<DiscretizedPath> GenerateSmoothPath(
    double speed, double path_step, double max_check_distance,
    double max_lateral_acc_limit, std::vector<double>* x,
    std::vector<double>* y, std::vector<double>* s) {
  FUNC_QTRACE();
  QCHECK_NOTNULL(x);
  QCHECK_NOTNULL(y);
  QCHECK_NOTNULL(s);
  QCHECK_GT(x->size(), 1);
  QCHECK_EQ(x->size(), y->size());
  QCHECK_EQ(x->size(), s->size());
  const int path_size = FloorToInt(s->back() / path_step) + 1;
  DiscretizedPath path;
  path.reserve(path_size);

  constexpr double kPathKappaLimit = 0.2;   // 1/m.
  constexpr double kPreviewDistance = 5.0;  // m.

  // Smooth coarse path by gradient descent.
  if (FLAGS_draw_coarse_path_canvas) {
    DrawPathPointsOnCanvas(*x, *y, vis::Color::kYellow, "coarse_points");
  }
  constexpr int kMaxIter = 50;
  constexpr double kEps = 0.015;
  constexpr double kAlpha = 0.1;
  const int smooth_num = s->end() - std::lower_bound(s->begin(), s->end(), 0.0);
  const auto smooth_result = SmoothPointsByGradientDescent(
      absl::MakeSpan(*x).last(smooth_num), absl::MakeSpan(*y).last(smooth_num),
      kMaxIter, kEps, kAlpha);
  VLOG(2) << smooth_result.DebugString();
  if (smooth_result.success) {
    for (int i = 1; i < static_cast<int>(x->size()); ++i) {
      (*s)[i] =
          (*s)[i - 1] + Hypot((*x)[i] - (*x)[i - 1], (*y)[i] - (*y)[i - 1]);
    }
    if (FLAGS_draw_coarse_path_canvas) {
      DrawPathPointsOnCanvas(*x, *y, vis::Color::kGreen, "smooth_points");
    }
  }

  const CubicSpline x_s(*s, *x);
  const CubicSpline y_s(*s, *y);
  double current_s = 0.0;
  for (int i = 0; i < path_size; ++i) {
    path.push_back(EvaluatePathPointOnCubicSplines(x_s, y_s, current_s));
    current_s += path_step;
  }

  const double preview_step = CeilToInt(kPreviewDistance / path_step);
  const double preview_distance = preview_step * path_step;

  double max_kappa = 0.0;
  for (int i = 0;
       i + preview_step < path_size && path[i].s() < max_check_distance; ++i) {
    const double theta_now = path[i].theta();
    const double theta_preview = path[i + preview_step].theta();
    const double kappa =
        std::abs(NormalizeAngle(theta_preview - theta_now)) / preview_distance;
    if (kappa > max_kappa) {
      max_kappa = kappa;
    }
  }
  const double max_lateral_accel = speed * speed * max_kappa;

  if (max_lateral_accel > max_lateral_acc_limit) {
    return absl::FailedPreconditionError(absl::StrCat(
        "[GenerateSmoothPath]: Lateral acc too large, max_lateral_accel: ",
        max_lateral_accel, ", speed: ", speed, ", max kappa: ", max_kappa));
  } else if (max_kappa > kPathKappaLimit) {
    return absl::FailedPreconditionError(
        "[GenerateSmoothPath]: Kappa too large.");
  }

  return path;
}

absl::StatusOr<DiscretizedPath> GenerateLaneKeepPathFromDrivePassage(
    const DrivePassage& drive_passage,
    const ApolloTrajectoryPointProto& plan_start_point, double path_length,
    double path_step, double path_min_length, double path_min_extension_time) {
  FUNC_QTRACE();
  const auto& start_path_point = plan_start_point.path_point();
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> s;
  const int coarse_path_size = CeilToInt(path_length / kCoarseSampleStep) + 1 +
                               kReverseExtensionPointsNum;
  x.reserve(coarse_path_size);
  y.reserve(coarse_path_size);
  s.reserve(coarse_path_size);
  for (int i = kReverseExtensionPointsNum; i > 0; --i) {
    const double current_s = -static_cast<double>(i) * kCoarseSampleStep;
    const auto path_point =
        GetPathPointAlongCircle(start_path_point, current_s);
    x.push_back(path_point.x());
    y.push_back(path_point.y());
    s.push_back(current_s);
  }
  x.push_back(start_path_point.x());
  y.push_back(start_path_point.y());
  s.push_back(0.0);
  const double speed = std::max(kMinMotionPreviewSpeed, plan_start_point.v());
  // Lane keeping trajectory by quintic polynomial in Frenet coordinates.

  // To prevent accuracy issues from Frenet coordinates conversion, we
  // keep two path points in Cartesian frame and start path planning in
  // the Frenet frame from the second path point.
  std::vector<double> lc_x;
  std::vector<double> lc_y;
  std::vector<double> lc_s;
  const auto lc_start_point =
      GetPathPointAlongCircle(start_path_point, kCoarseSampleStep);
  RETURN_IF_ERROR(GenerateLaneCenteringCoarsePath(
      drive_passage, lc_start_point, speed, path_length - kCoarseSampleStep,
      &lc_x, &lc_y, &lc_s));
  std::transform(lc_s.begin(), lc_s.end(), lc_s.begin(),
                 [](double s) { return s + kCoarseSampleStep; });
  x.insert(x.end(), lc_x.begin(), lc_x.end());
  y.insert(y.end(), lc_y.begin(), lc_y.end());
  s.insert(s.end(), lc_s.begin(), lc_s.end());
  if (s.size() < kMinPointsSize || s.back() < path_min_length) {
    return absl::NotFoundError("[Path Generator]: Drive passage too short.");
  } else if (s.back() < speed * path_min_extension_time) {
    ExtendPathByLastTheta(&x, &y, &s, speed * path_min_extension_time);
  }
  const double max_check_distance =
      std::max(path_min_length, speed * path_min_extension_time);
  return GenerateSmoothPath(speed, path_step, max_check_distance,
                            kPathMaxLateralAccLimit, &x, &y, &s);
}

absl::StatusOr<DiscretizedPath> GenerateLaneChangePathFromDrivePassage(
    const DrivePassage& target_drive_passage,
    const ApolloTrajectoryPointProto& plan_start_point, double path_length,
    double path_step, double path_min_length, double path_min_extension_time) {
  FUNC_QTRACE();
  const auto& start_path_point = plan_start_point.path_point();
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> s;
  const int coarse_path_size = CeilToInt(path_length / kCoarseSampleStep) + 1 +
                               kReverseExtensionPointsNum;
  x.reserve(coarse_path_size);
  y.reserve(coarse_path_size);
  s.reserve(coarse_path_size);
  for (int i = kReverseExtensionPointsNum; i > 0; --i) {
    const double current_s = -static_cast<double>(i) * kCoarseSampleStep;
    const auto path_point =
        GetPathPointAlongCircle(start_path_point, current_s);
    x.push_back(path_point.x());
    y.push_back(path_point.y());
    s.push_back(current_s);
  }
  x.push_back(start_path_point.x());
  y.push_back(start_path_point.y());
  s.push_back(0.0);
  const double speed = std::max(kMinMotionPreviewSpeed, plan_start_point.v());
  // Motion prediction.
  const double motion_preview_dis = kMotionPreviewTime * speed;
  constexpr double kLaneChangeLatDistanceThres = 2.5;  // m.
  double lc_start_s = kCoarseSampleStep;
  while (lc_start_s < motion_preview_dis) {
    const auto path_point =
        GetPathPointAlongCircle(start_path_point, lc_start_s);
    const auto frenet_coord =
        target_drive_passage.QueryFrenetCoordinateAt(ToVec2d(path_point));
    if (frenet_coord.ok() &&
        std::abs(frenet_coord->l) < kLaneChangeLatDistanceThres) {
      break;
    }
    x.push_back(path_point.x());
    y.push_back(path_point.y());
    s.push_back(lc_start_s);
    lc_start_s += kCoarseSampleStep;
  }

  std::vector<double> lc_x;
  std::vector<double> lc_y;
  std::vector<double> lc_s;
  // Lane change trajectory by quintic polynomial in Frenet coordinates.
  const auto lc_start_point =
      GetPathPointAlongCircle(start_path_point, lc_start_s);
  RETURN_IF_ERROR(GenerateLaneCenteringCoarsePath(
      target_drive_passage, lc_start_point, speed, path_length - lc_start_s,
      &lc_x, &lc_y, &lc_s));

  std::transform(lc_s.begin(), lc_s.end(), lc_s.begin(),
                 [&lc_start_s](double s) { return s + lc_start_s; });
  x.insert(x.end(), lc_x.begin(), lc_x.end());
  y.insert(y.end(), lc_y.begin(), lc_y.end());
  s.insert(s.end(), lc_s.begin(), lc_s.end());

  if (s.size() < kMinPointsSize || s.back() < path_min_length) {
    return absl::NotFoundError("[Path Generator]: Drive passage too short.");
  } else if (s.back() < speed * path_min_extension_time) {
    ExtendPathByLastTheta(&x, &y, &s, speed * path_min_extension_time);
  }
  const double max_check_distance =
      std::max(path_min_length, speed * path_min_extension_time);
  return GenerateSmoothPath(speed, path_step, max_check_distance,
                            kPathMaxLateralAccLimit, &x, &y, &s);
}

absl::StatusOr<DiscretizedPath> GeneratePreviewLaneKeepPathFromStartPoint(
    const PlannerSemanticMapManager& psmm,
    const ApolloTrajectoryPointProto& plan_start_point, double path_length,
    double path_preview_time, double path_step, double path_min_length,
    double path_min_extension_time, double max_search_radius,
    double max_heading_diff, double drive_passage_length_factor,
    double min_start_preview_distance,
    std::optional<DiscretizedPath>* candidate_path) {
  FUNC_QTRACE();
  if (nullptr != candidate_path) {
    *candidate_path = std::nullopt;
  }
  const auto& start_path_point = plan_start_point.path_point();
  const double preview_step = path_length / path_preview_time;
  double preview_s = min_start_preview_distance + preview_step;
  PathPoint preview_path_point;
  // Whether the lane corresponding to the lane id is the opposite lane.
  absl::flat_hash_map<mapping::ElementId, bool> lane_attr_map;
  std::optional<LanePreviewResult> lane_preview_result;
  std::optional<LanePreviewResult> candidate_preview_result;
  while (preview_s < path_length) {
    preview_path_point = GetPathPointAlongCircle(start_path_point, preview_s);
    lane_preview_result = GetBestLaneInfoWithOneMoreStep(
        psmm, preview_path_point, preview_s, preview_step, path_length,
        max_search_radius, max_heading_diff, &lane_attr_map,
        &candidate_preview_result);
    if (lane_preview_result.has_value()) break;
    preview_s += preview_step;
  }

  if (!lane_preview_result.has_value()) {
    return absl::NotFoundError(
        "[Path Generator]: Cannot find preview nearest lane.");
  }

  auto path = GeneratePreviewLaneKeepPathFromLanePreviewResult(
      psmm, plan_start_point, *lane_preview_result, path_length, path_step,
      path_min_length, path_min_extension_time, drive_passage_length_factor);

  if (candidate_preview_result.has_value()) {
    if (!path.ok()) {
      return GeneratePreviewLaneKeepPathFromLanePreviewResult(
          psmm, plan_start_point, *candidate_preview_result, path_length,
          path_step, path_min_length, path_min_extension_time,
          drive_passage_length_factor);
    } else if (nullptr != candidate_path) {
      auto candidate_preview_path =
          GeneratePreviewLaneKeepPathFromLanePreviewResult(
              psmm, plan_start_point, *candidate_preview_result, path_length,
              path_step, path_min_length, path_min_extension_time,
              drive_passage_length_factor);
      if (candidate_preview_path.ok()) {
        *candidate_path = std::move(*candidate_preview_path);
      }
    }
  }

  return path;
}

}  // namespace planner
}  // namespace qcraft
