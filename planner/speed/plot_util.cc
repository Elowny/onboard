#include "onboard/planner/speed/plot_util.h"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <optional>
#include <utility>
#include <vector>

#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "gflags/gflags.h"

#include "onboard/global/buffered_logger.h"
#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/planner_util.h"
#include "onboard/planner/speed/proto/speed_finder.pb.h"
#include "onboard/planner/speed/speed_point.h"
#include "onboard/planner/speed/st_boundary.h"
#include "onboard/planner/speed/st_point.h"
#include "onboard/planner/trajectory_point.h"
#include "onboard/prediction/predicted_trajectory.h"
#include "onboard/utils/map_util.h"
#include "onboard/vis/canvas/canvas.h"
#include "onboard/vis/canvas/proto/canvas_buffer.pb.h"
#include "onboard/vis/common/color.h"

#include "offboard/vis/vantage/charts/chart_util.h"
#include "offboard/vis/vantage/vantage_server/vantage_canvas_client.h"

// IWYU pragma: no_include "Eigen/Core"
// IWYU pragma: no_include <google/protobuf/repeated_ptr_field.h>

DEFINE_bool(planner_plot_raw_st_boundary, false,
            "Whether to plot raw_st_boundary");

namespace qcraft {
namespace planner {
namespace speed {
namespace {

using vis::vantage::ChartDataProto;
using vis::vantage::ChartSeriesDataProto;
using vis::vantage::ChartSubchartDataProto;
using DecisionType = StBoundaryProto::DecisionType;

int GetSoftBoundIndex(
    const ::google::protobuf::RepeatedField<double>& repeated_time, double t) {
  return std::distance(
      repeated_time.begin(),
      std::lower_bound(repeated_time.begin(), repeated_time.end(), t));
}

void AddChartData(double t, double s, const std::string& tips,
                  ChartSubchartDataProto* subchart,
                  ChartSeriesDataProto* series_data) {
  QCHECK_NOTNULL(subchart);
  QCHECK_NOTNULL(series_data);
  subchart->add_x_values(t);
  series_data->add_values(s);
  series_data->add_tips(tips);
}

void DrawSoftBound(const StBoundaryWithDecision& st_boundary_with_decision,
                   const std::string& series_data_name,
                   const SpeedFinderDebugProto& speed_finder_proto,
                   ChartDataProto* chart) {
  QCHECK_NOTNULL(chart);

  const StBoundary* st_boundary = st_boundary_with_decision.st_boundary();
  const auto decision = st_boundary_with_decision.decision_type();
  if (decision == StBoundaryProto::UNKNOWN ||
      decision == StBoundaryProto::IGNORE) {
    return;
  }

  const auto id =
      GetStBoundaryIntegrationId(*st_boundary_with_decision.st_boundary());
  const auto& speed_opt_debug = speed_finder_proto.speed_optimizer();

  QCHECK(decision == StBoundaryProto::FOLLOW ||
         decision == StBoundaryProto::LEAD);
  const auto* soft_bound_data =
      decision == StBoundaryProto::FOLLOW
          ? FindOrNull(speed_opt_debug.soft_s_upper_bound(), id)
          : FindOrNull(speed_opt_debug.soft_s_lower_bound(), id);
  if (!soft_bound_data || soft_bound_data->time_size() < 2) return;

  const double min_t = st_boundary->min_t();
  const double max_t = st_boundary->max_t();
  const int start_idx = GetSoftBoundIndex(soft_bound_data->time(), min_t);

  // Plot preparation.
  auto* soft_bound_subchart = chart->add_subcharts();
  soft_bound_subchart->set_x_name("time");
  auto* soft_series_data = soft_bound_subchart->add_y();
  soft_series_data->set_name(series_data_name);
  vis::Color::kGray.ToProto(soft_series_data->mutable_color());

  // Plot first point.
  const auto stb_start_pt =
      st_boundary->GetBoundarySRange(soft_bound_data->time(start_idx));
  if (!stb_start_pt.has_value()) return;
  const double stb_start_point_s = decision == StBoundaryProto::FOLLOW
                                       ? stb_start_pt->second
                                       : stb_start_pt->first;

  AddChartData(soft_bound_data->time(start_idx), stb_start_point_s, "",
               soft_bound_subchart, soft_series_data);

  // Plot soft bound.
  int end_idx = 0.0;
  const int soft_bound_size = soft_bound_data->time_size();
  for (int i = start_idx; i < soft_bound_size; ++i) {
    const double curr_time = soft_bound_data->time(i);
    if (!InRange(curr_time, min_t, max_t)) break;
    end_idx = i;
    AddChartData(curr_time, soft_bound_data->value(i), soft_bound_data->info(i),
                 soft_bound_subchart, soft_series_data);
  }

  // Plot end point.
  const auto stb_end_pt =
      st_boundary->GetBoundarySRange(soft_bound_data->time(end_idx));
  if (!stb_end_pt.has_value()) return;
  const double stb_end_point_s = decision == StBoundaryProto::FOLLOW
                                     ? stb_end_pt->second
                                     : stb_end_pt->first;
  AddChartData(soft_bound_data->time(end_idx), stb_end_point_s, "",
               soft_bound_subchart, soft_series_data);
}

void DrawPolygon(absl::Span<const StPoint> lower_points,
                 absl::Span<const StPoint> upper_points, absl::string_view tips,
                 ChartSubchartDataProto* subchart,
                 ChartSeriesDataProto* series_data) {
  QCHECK_NOTNULL(subchart);
  QCHECK_NOTNULL(series_data);

  const auto make_tips = [](const auto& point, absl::string_view tips) {
    return absl::StrFormat("s : %.1fm\nt : %.1fs%s", point.s(), point.t(),
                           tips);
  };

  const auto& first_point = upper_points.front();
  AddChartData(first_point.t(), first_point.s(), make_tips(first_point, ""),
               subchart, series_data);

  const auto& bottom_left_point = lower_points.front();
  AddChartData(bottom_left_point.t(), bottom_left_point.s(),
               make_tips(bottom_left_point, tips), subchart, series_data);
  for (auto it = lower_points.begin() + 1; it != lower_points.end() - 1; ++it) {
    AddChartData(it->t(), it->s(), make_tips(*it, ""), subchart, series_data);
  }
  const auto& bottom_right_point = lower_points.back();
  AddChartData(bottom_right_point.t(), bottom_right_point.s(),
               make_tips(bottom_right_point, tips), subchart, series_data);
  for (auto it = upper_points.rbegin(); it != upper_points.rend(); ++it) {
    AddChartData(it->t(), it->s(), make_tips(*it, ""), subchart, series_data);
  }
}

void DrawStBoundary(const StBoundaryWithDecision& st_boundary_with_decision,
                    ChartDataProto* chart, std::string* series_data_name) {
  QCHECK_NOTNULL(chart);
  QCHECK_NOTNULL(series_data_name);

  auto* subchart = chart->add_subcharts();
  subchart->set_x_name("time");
  auto* series_data = subchart->add_y();
  const auto decision_type = st_boundary_with_decision.decision_type();
  if (decision_type == StBoundaryProto::IGNORE ||
      decision_type == StBoundaryProto::UNKNOWN) {
    vis::Color::kLightGray.ToProto(series_data->mutable_color());
  }

  if (st_boundary_with_decision.id().find("raw") != std::string::npos) {
    series_data->set_pen_style(ChartSeriesDataProto::DOTLINE);
  } else if (decision_type == StBoundaryProto::LEAD) {
    series_data->set_pen_style(ChartSeriesDataProto::DASHLINE);
  } else if (decision_type == StBoundaryProto::IGNORE) {
    series_data->set_pen_style(ChartSeriesDataProto::DASHDOTLINE);
  }

  const StBoundary* st_boundary = st_boundary_with_decision.st_boundary();
  QCHECK_EQ(st_boundary->lower_points().size(),
            st_boundary->upper_points().size());
  *series_data_name = st_boundary->id();
  series_data->set_name(*series_data_name);

  std::string tips = absl::StrFormat(
      "id : %s\nprob : %.2f\nis_stationary : %s\ndecision : "
      "%s\ndecision_reason : %s\nignore_reason : %s\nobject_type: "
      "%s\nprotection_type : %s\ndecision_info : %s\n\npass_time: "
      "%.2f\nyield_time: %.2f",
      st_boundary->id(), st_boundary->probability(),
      st_boundary->is_stationary() ? "true" : "false",
      StBoundaryProto::DecisionType_Name(decision_type),
      StBoundaryProto::DecisionReason_Name(
          st_boundary_with_decision.decision_reason()),
      StBoundaryProto::IgnoreReason_Name(
          st_boundary_with_decision.ignore_reason()),
      StBoundaryProto::ObjectType_Name(st_boundary->object_type()),
      StBoundaryProto::ProtectionType_Name(st_boundary->protection_type()),
      st_boundary_with_decision.decision_info(),
      st_boundary_with_decision.pass_time(),
      st_boundary_with_decision.yield_time());

  if (st_boundary->overlap_meta().has_value()) {
    tips += absl::StrFormat(
        "\npattern: %s\nsource: %s\npriority: "
        "%s\npriority_reason:%s\nmodification_type:%s",
        StOverlapMetaProto::OverlapPattern_Name(
            st_boundary->overlap_meta()->pattern()),
        StOverlapMetaProto::OverlapSource_Name(
            st_boundary->overlap_meta()->source()),
        StOverlapMetaProto::OverlapPriority_Name(
            st_boundary->overlap_meta()->priority()),
        st_boundary->overlap_meta()->priority_reason(),
        StOverlapMetaProto::ModificationType_Name(
            st_boundary->overlap_meta()->modification_type()));
  }

  DrawPolygon(st_boundary->lower_points(), st_boundary->upper_points(), tips,
              subchart, series_data);
}

void PlotStBoundaries(
    absl::Span<const StBoundaryWithDecision> st_boundaries_with_decision,
    const SpeedFinderDebugProto& speed_finder_proto, ChartDataProto* chart) {
  QCHECK_NOTNULL(chart);

  for (const StBoundaryWithDecision& boundary : st_boundaries_with_decision) {
    const StBoundary* st_boundary = boundary.st_boundary();
    QCHECK_EQ(st_boundary->lower_points().size(),
              st_boundary->upper_points().size());

    // Draw st_boundary.
    std::string series_data_name;
    DrawStBoundary(boundary, chart, &series_data_name);

    // Draw soft_bound
    DrawSoftBound(boundary, series_data_name, speed_finder_proto, chart);

    // Draw raw st_boundary
    if (FLAGS_planner_plot_raw_st_boundary) {
      const StBoundary* raw_st_boundary = boundary.raw_st_boundary();
      auto* raw_st_boundary_subchart = chart->add_subcharts();
      raw_st_boundary_subchart->set_x_name("time");
      auto* raw_series_data = raw_st_boundary_subchart->add_y();
      raw_series_data->set_name(series_data_name);
      raw_series_data->set_pen_style(ChartSeriesDataProto::DASHLINE);
      vis::Color::kRed.ToProto(raw_series_data->mutable_color());
      DrawPolygon(raw_st_boundary->lower_points(),
                  raw_st_boundary->upper_points(), "", raw_st_boundary_subchart,
                  raw_series_data);
    }
  }
}

void PlotSpeedLimitData(const SpeedFinderDebugProto& speed_finder_proto,
                        const SpeedLimitTypeProto::Type& type,
                        const vis::Color& color, ChartDataProto* chart) {
  QCHECK_NOTNULL(chart);
  const auto type_name = SpeedLimitTypeProto::Type_Name(type);

  const auto* data =
      FindOrNull(speed_finder_proto.speed_optimizer().speed_limit(), type_name);
  if (data == nullptr) {
    return;
  }
  QCHECK_EQ(data->time_size(), data->value_size());

  auto* subchart = chart->add_subcharts();
  subchart->set_x_name("time");
  auto* x_value_config = subchart->mutable_x_value_config();
  x_value_config->set_begin(*data->time().begin());
  x_value_config->set_size(data->time_size());
  x_value_config->set_interval(data->time(1) - data->time(0));

  auto* series_data = subchart->add_y();
  series_data->set_name(type_name);
  color.ToProto(series_data->mutable_color());

  constexpr double kMaxPlotSpeedLimit = 50.0;  // m/s
  for (int i = 0; i < data->time_size(); ++i) {
    series_data->add_values(std::min(data->value(i), kMaxPlotSpeedLimit));
    series_data->add_tips(data->info(i));
  }
}

std::string SvtGraphPointCostToTipString(const SvtGraphPointDebugProto& point) {
  return absl::StrFormat(
      "speed_limit_cost : %.2f\n"
      "reference_speed_cost : %.2f\n"
      "accel_cost : %.2f\n"
      "object_cost : %.2f\n"
      "spatial_potential_cost : %.2f\n"
      "total_cost : %.2f",
      point.speed_limit_cost(), point.reference_speed_cost(),
      point.accel_cost(), point.object_cost(), point.spatial_potential_cost(),
      point.total_cost());
}

void PlotAllSpeedProfilesDebugInfo(
    const SpeedProfileDebugProto& speed_profile, const std::string& name,
    const vis::Color& color, const ChartSeriesDataProto::PenStyle& pen_style,
    ChartDataProto* chart) {
  QCHECK_NOTNULL(chart);
  auto* subchart = chart->add_subcharts();

  subchart->set_x_name("time");
  auto* y = subchart->add_y();
  y->set_name(name);
  color.ToProto(y->mutable_color());
  y->set_pen_style(pen_style);

  for (const auto& point : speed_profile.svt_graph_points()) {
    subchart->add_x_values(point.speed_point().t());
    y->add_values(point.speed_point().s());
    y->add_tips(absl::StrFormat(
        "t : %.1f\ns : %.1f\nv : %.1f\na : %.1f\nj : %.1f\n%s",
        point.speed_point().t(), point.speed_point().s(),
        point.speed_point().v(), point.speed_point().a(),
        point.speed_point().j(), SvtGraphPointCostToTipString(point)));
  }
}
}  // namespace

void ExportStBoundaryToChart(
    std::string_view base_name, int traj_steps,
    absl::Span<const StBoundaryWithDecision> st_boundaries_with_decision,
    const SpeedFinderDebugProto& speed_finder_proto, double path_length,
    ChartDataProto* chart) {
  SCOPED_QTRACE_ARG1("ExportStBoundaryToChart", "n",
                     st_boundaries_with_decision.size());
  QCHECK_NOTNULL(chart);

  chart->set_title(absl::StrCat(base_name, "/speed/st_graph"));

  SpeedVector end_of_path({SpeedPoint(0.0, path_length, 0.0, 0.0, 0.0),
                           SpeedPoint((traj_steps - 1) * kTrajectoryTimeStep,
                                      path_length, 0.0, 0.0, 0.0)});
  ExportSpeedStDataToChart(end_of_path, "end_of_path", vis::Color::kGray,
                           ChartSeriesDataProto::SOLIDLINE, chart);

  PlotStBoundaries(st_boundaries_with_decision, speed_finder_proto, chart);
}

void ExportSamplingDpSpeedToChart(
    std::string_view base_name,
    const absl::Span<const StBoundaryWithDecision>& st_boundaries_with_decision,
    const SpeedVector& preliminary_speed,
    const InteractiveSpeedDebugProto& interactive_speed_debug,
    const SpeedFinderDebugProto& speed_finder_proto, ChartDataProto* chart) {
  SCOPED_QTRACE_ARG1("ExportSamplingDpSpeedToChart", "n",
                     st_boundaries_with_decision.size());
  QCHECK_NOTNULL(chart);
  chart->set_title(absl::StrCat(base_name, "/speed/sampling_dp"));
  PlotStBoundaries(st_boundaries_with_decision, speed_finder_proto, chart);
  for (const auto& speed_profile :
       interactive_speed_debug.non_interactive_sampling_dp().speed_profiles()) {
    PlotAllSpeedProfilesDebugInfo(speed_profile, "all_speed_profiles",
                                  vis::Color::kLightGray,
                                  ChartSeriesDataProto::DASHLINE, chart);
  }

  ExportSpeedPointsProtoStDataToChart(
      interactive_speed_debug.non_interactive_speed_profile(),
      "non_interactive_speed", vis::Color::kDarkGreen,
      ChartSeriesDataProto::DASHLINE, chart);
  ExportSpeedStDataToChart(preliminary_speed, "preliminary_speed",
                           vis::Color::kOrange, ChartSeriesDataProto::DASHLINE,
                           chart);
}

void ExportInteractiveSpeedToChart(
    std::string_view base_name,
    const absl::Span<const StBoundaryWithDecision>& st_boundaries_with_decision,
    const SpeedVector& preliminary_speed,
    const InteractiveSpeedDebugProto& interactive_speed_debug,
    const SpeedFinderDebugProto& speed_finder_proto,
    ChartDataProto* interactive_speed_chart) {
  SCOPED_QTRACE_ARG1("ExportInteractiveSpeedToChart", "n",
                     st_boundaries_with_decision.size());
  QCHECK_NOTNULL(interactive_speed_chart);

  // Interactive speed data.
  interactive_speed_chart->set_title(
      absl::StrCat(base_name, "/speed/interactive"));
  PlotStBoundaries(st_boundaries_with_decision, speed_finder_proto,
                   interactive_speed_chart);
  for (const auto& speed_profile :
       interactive_speed_debug.interactive_sampling_dp().speed_profiles()) {
    PlotAllSpeedProfilesDebugInfo(
        speed_profile, "all_speed_profiles", vis::Color::kLightGray,
        ChartSeriesDataProto::DASHLINE, interactive_speed_chart);
  }
  for (const auto& speed_profile :
       interactive_speed_debug.candidate_set().speed_profile()) {
    ExportSpeedPointsProtoStDataToChart(
        speed_profile, "candidate_set", vis::Color::kAzure,
        ChartSeriesDataProto::DASHLINE, interactive_speed_chart);
  }

  ExportSpeedPointsProtoStDataToChart(
      interactive_speed_debug.non_interactive_speed_profile(),
      "non_interactive_speed", vis::Color::kDarkGreen,
      ChartSeriesDataProto::DASHLINE, interactive_speed_chart);
  ExportSpeedStDataToChart(preliminary_speed, "preliminary_speed",
                           vis::Color::kOrange, ChartSeriesDataProto::DASHLINE,
                           interactive_speed_chart);
}

void ExportSpeedLimitToChart(std::string_view base_name,
                             const SpeedFinderDebugProto& speed_finder_proto,
                             ChartDataProto* chart) {
  FUNC_QTRACE();
  QCHECK_NOTNULL(chart);
  chart->set_title(absl::StrCat(base_name, "/speed/vt_graph"));

  PlotSpeedLimitData(speed_finder_proto, SpeedLimitTypeProto::LANE,
                     vis::Color::kBlack, chart);
  PlotSpeedLimitData(speed_finder_proto, SpeedLimitTypeProto::CURVATURE,
                     vis::Color::kRed, chart);
  PlotSpeedLimitData(speed_finder_proto, SpeedLimitTypeProto::EXTERNAL,
                     vis::Color::kDarkBlue, chart);
  PlotSpeedLimitData(speed_finder_proto, SpeedLimitTypeProto::MOVING_CLOSE_TRAJ,
                     vis::Color::kDarkYellow, chart);
  PlotSpeedLimitData(speed_finder_proto,
                     SpeedLimitTypeProto::UNCERTAIN_PEDESTRAIN,
                     vis::Color::kLightBlue, chart);
  PlotSpeedLimitData(speed_finder_proto, SpeedLimitTypeProto::UNCERTAIN_VEHICLE,
                     vis::Color::kViolet, chart);
  PlotSpeedLimitData(speed_finder_proto, SpeedLimitTypeProto::STEER_RATE,
                     vis::Color::kCyan, chart);
  PlotSpeedLimitData(speed_finder_proto,
                     SpeedLimitTypeProto::NEAR_PARALLEL_VEHICLE,
                     vis::Color::kLightRed, chart);
  PlotSpeedLimitData(speed_finder_proto, SpeedLimitTypeProto::IGNORE_OBJECT,
                     vis::Color::kMiddleBlueGreen, chart);
  PlotSpeedLimitData(speed_finder_proto, SpeedLimitTypeProto::CLOSE_CURB,
                     vis::Color::kChocolate, chart);
  PlotSpeedLimitData(speed_finder_proto, SpeedLimitTypeProto::DEFAULT,
                     vis::Color::kTiffanyBlue, chart);
}

void DrawStBoundaryOnCanvas(
    std::string_view base_name,
    absl::Span<const StBoundaryWithDecision> st_boundaries_with_decision,
    const VehicleGeometryParamsProto& vehicle_geom,
    const SpacetimeTrajectoryManager& traj_mgr, const DiscretizedPath& path) {
  const double rac_to_fb = vehicle_geom.front_edge_to_center();
  for (const auto& st : st_boundaries_with_decision) {
    if (st.st_boundary()->source_type() !=
        StBoundarySourceTypeProto::ST_OBJECT) {
      continue;
    }
    QCHECK(st.traj_id().has_value());
    const auto* traj =
        QCHECK_NOTNULL(traj_mgr.FindTrajectoryById(*st.traj_id()));

    const auto& obj = traj->planner_object();
    const auto pt = path.Evaluate(st.st_boundary()->min_s());
    const Vec2d front_pt = Vec2d(pt.x(), pt.y()) +
                           Vec2d::FastUnitFromAngle(pt.theta()) * rac_to_fb;
    auto& canvas = vis::vantage::GetCanvasClient()->GetCanvas(
        absl::StrCat(base_name, "/st_boundary/", st.id()));
    canvas.SetGroundZero(1);
    const auto& contour = obj.contour();
    canvas.DrawLine(Vec3d(front_pt, 0.1), Vec3d(contour.CircleCenter(), 0.1),
                    vis::Color::kOrange, /*line_width=*/1,
                    vis::BorderStyleProto::DASHED);
    canvas.DrawCircle(Vec3d(contour.CircleCenter(), 0.1),
                      contour.CircleRadius(), vis::Color::kOrange,
                      /*border_line_width=*/1.0, vis::BorderStyleProto::DASHED);
  }
}

void ExportTrajToChart(
    std::string_view base_name,
    const std::vector<ApolloTrajectoryPointProto>& trajectory,
    ChartDataProto* chart) {
  FUNC_QTRACE();
  QCHECK_NOTNULL(chart);
  const int size = trajectory.size();
  std::vector<std::string> names = {"s",         "v",       "a",     "j",
                                    "lon_jerk",  "theta",   "kappa", "psi",
                                    "lat_accel", "lat_jerk"};
  std::vector<std::vector<double>> values(names.size());
  for (auto& v : values) v.reserve(size);

  TrajectoryPoint traj_point0(trajectory[0]);
  for (int i = 0; i < size; ++i) {
    TrajectoryPoint traj_point(trajectory[i]);
    int y_index = 0;
    values[y_index++].push_back(traj_point.s() - traj_point0.s());
    values[y_index++].push_back(traj_point.v());
    values[y_index++].push_back(traj_point.a());
    values[y_index++].push_back(traj_point.j());
    values[y_index++].push_back(ComputeLongitudinalJerk(traj_point));
    values[y_index++].push_back(traj_point.theta());
    values[y_index++].push_back(traj_point.kappa());
    values[y_index++].push_back(traj_point.psi());
    values[y_index++].push_back(ComputeLateralAcceleration(traj_point));
    values[y_index++].push_back(ComputeLateralJerk(traj_point));
  }

  chart->set_title(absl::StrCat(base_name, "/speed/traj"));
  vis::vantage::GenerateSubchartFromData(
      traj_point0.t(), size, trajectory[1].relative_time() - traj_point0.t(),
      values, "t", names, {},
      std::vector<std::vector<std::string>>(names.size()),
      chart->add_subcharts());
}

void ExportPathToChart(std::string_view base_name,
                       const std::vector<PathPoint>& path,
                       ChartDataProto* chart) {
  FUNC_QTRACE();
  QCHECK_NOTNULL(chart);
  const int size = path.size();
  std::vector<double> s;
  std::vector<std::string> names =
      std::vector<std::string>{"x", "y", "theta", "kappa"};
  std::vector<std::vector<double>> values(names.size());
  s.reserve(size);
  for (auto& v : values) v.reserve(size);

  for (int i = 0; i < size; ++i) {
    const auto& path_point = path[i];
    s.push_back(path_point.s());
    int y_index = 0;
    values[y_index++].push_back(path_point.x());
    values[y_index++].push_back(path_point.y());
    values[y_index++].push_back(path_point.theta());
    values[y_index++].push_back(path_point.kappa());
  }

  chart->set_title(absl::StrCat(base_name, "/speed/path"));
  vis::vantage::GenerateSubchartFromData(
      s, values, "s", names, {},
      std::vector<std::vector<std::string>>(names.size()),
      chart->add_subcharts());
}

void ExportSpeedStDataToChart(const SpeedVector& speed, const std::string& name,
                              const vis::Color& color,
                              const ChartSeriesDataProto::PenStyle& pen_style,
                              ChartDataProto* chart) {
  SCOPED_QTRACE_ARG1("ExportSpeedStDataToChart", "name", name);

  QCHECK_NOTNULL(chart);
  auto* subchart = chart->add_subcharts();

  subchart->set_x_name("time");
  auto* y = subchart->add_y();
  y->set_name(name);
  color.ToProto(y->mutable_color());
  y->set_pen_style(pen_style);

  for (const auto& p : speed) {
    subchart->add_x_values(p.t());
    y->add_values(p.s());
    y->add_tips(
        absl::StrFormat("s : %.1f\nt : %.1f\nv : %.1f\na : %.1f\nj : %.1f",
                        p.s(), p.t(), p.v(), p.a(), p.j()));
  }
}

void ExportSpeedVtDataToChart(const SpeedVector& speed, const std::string& name,
                              const vis::Color& color, bool add_tips,
                              ChartDataProto* chart) {
  SCOPED_QTRACE_ARG1("ExportSpeedVtDataToChart", "name", name);
  QCHECK_NOTNULL(chart);
  auto* subchart = chart->add_subcharts();

  subchart->set_x_name("time");
  auto* y = subchart->add_y();
  y->set_name(name);
  color.ToProto(y->mutable_color());

  for (const auto& p : speed) {
    subchart->add_x_values(p.t());
    y->add_values(p.v());
    y->add_tips(add_tips
                    ? absl::StrFormat(
                          "s : %.1f\nt : %.1f\nv : %.1f\na : %.1f\nj : %.1f",
                          p.s(), p.t(), p.v(), p.a(), p.j())
                    : "");
  }
}

void ExportSpeedPointsProtoStDataToChart(
    const SpeedPointsProto& speed, const std::string& name,
    const vis::Color& color,
    const vis::vantage::ChartSeriesDataProto::PenStyle& pen_style,
    vis::vantage::ChartDataProto* chart) {
  QCHECK_NOTNULL(chart);
  auto* subchart = chart->add_subcharts();

  subchart->set_x_name("time");
  auto* y = subchart->add_y();
  y->set_name(name);
  color.ToProto(y->mutable_color());
  y->set_pen_style(pen_style);

  for (const auto& p : speed.speed_points()) {
    subchart->add_x_values(p.t());
    y->add_values(p.s());
    y->add_tips(
        absl::StrFormat("s : %.1f\nt : %.1f\nv : %.1f\na : %.1f\nj : %.1f",
                        p.s(), p.t(), p.v(), p.a(), p.j()));
  }
}

void ExportPredVtDataToChart(
    const std::unordered_map<std::string, const SpacetimeObjectTrajectory*>&
        pred_trajs,
    std::string_view base_name, ChartDataProto* chart) {
  FUNC_QTRACE();
  QCHECK_NOTNULL(chart);
  chart->set_title(absl::StrCat(base_name, "/speed/prediction_vt_graph"));

  // Plot prediction traj's speed on pred-vt-chart.
  for (const auto& [traj_id, st_traj] : pred_trajs) {
    auto* subchart = chart->add_subcharts();
    subchart->set_x_name("time");
    auto* y = subchart->add_y();
    y->set_pen_style(st_traj->is_stationary()
                         ? ChartSeriesDataProto::DASHLINE
                         : ChartSeriesDataProto::SOLIDLINE);
    if (traj_id.find("|raw") != std::string::npos) {
      vis::Color::kGray.ToProto(y->mutable_color());
    }
    y->set_name(traj_id);
    const auto& traj_points = st_traj->trajectory().points();
    double last_v = 0.0;
    constexpr double kSmallSpeedDiff = 1e-3;  // m/s.
    for (int i = 0, size = traj_points.size(); i < size; ++i) {
      const auto& p = traj_points[i];
      if (i != 0 && i != size - 1 &&
          std::fabs(p.v() - last_v) < kSmallSpeedDiff) {
        continue;
      }
      subchart->add_x_values(p.t());
      y->add_values(p.v());
      y->add_tips(
          absl::StrFormat("s : %.1f\nv : %.1f\na : %.1f", p.s(), p.v(), p.a()));
      last_v = p.v();
    }
  }
}

void DrawPathPoint(const PathPoint& path_point, std::string_view traj_id,
                   const vis::Color& color) {
  auto& canvas = vis::vantage::GetCanvasClient()->GetCanvas(
      absl::StrCat("path_point/", traj_id));
  canvas.SetGroundZero(1);
  const Vec2d pos(path_point.x(), path_point.y());
  canvas.DrawPoint(Vec3d(pos, 0.1), color, /*point_size=*/20);
  const auto heading = Vec2d::FastUnitFromAngle(path_point.theta());
  canvas.DrawLine(Vec3d(pos, 0.1), Vec3d(pos + heading * 10.0), color,
                  /*line_width=*/5);
}

}  // namespace speed
}  // namespace planner
}  // namespace qcraft
