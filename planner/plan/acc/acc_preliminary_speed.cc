#include "onboard/planner/plan/acc/acc_preliminary_speed.h"

#include <algorithm>
#include <cmath>
#include <optional>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "glog/logging.h"

#include "onboard/lite/check.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/planner/plan/acc/acc_defs.h"
#include "onboard/planner/plan/acc/timing_test_util.h"
#include "onboard/planner/speed/proto/speed_finder.pb.h"
#include "onboard/proto/trajectory_point.pb.h"

namespace qcraft {
namespace planner {
namespace {
const PiecewiseLinearFunction<double, double> kMaxExtCommandSpeedAccPLF(
    std::vector<double>{3.0, 5.0, 10.0, 15.0, 30.0},
    std::vector<double>{2.2, 1.8, 1.6, 1.4, 1.2});
const PiecewiseLinearFunction<double, double> kMaxExtCommandSpeedBrakePLF(
    std::vector<double>{3.0, 5.0, 10.0, 15.0},
    std::vector<double>{-1.5, -1.5, -1.3, -1.0});

inline AccSpeedLimitProto ToSpeedLimitProto(
    const std::optional<double>& v_limit_opt,
    const std::optional<double>& a_limit_opt, const absl::string_view& traj_id,
    AccSpeedLimitSource source) {
  AccSpeedLimitProto speed_limit_proto;
  speed_limit_proto.set_source(source);
  if (v_limit_opt.has_value()) {
    speed_limit_proto.set_v(*v_limit_opt);
  }
  if (a_limit_opt.has_value()) {
    speed_limit_proto.set_a(*a_limit_opt);
  }
  speed_limit_proto.set_info(std::string(traj_id));
  return speed_limit_proto;
}

}  // namespace

namespace internal {
constexpr double kKappaEpsilon = 1e-8;

const PiecewiseLinearFunction<double, double> kCurvatureSpeedLimitMaxBrakePLF(
    std::vector<double>{3.0, 5.0, 10.0},
    std::vector<double>{
        -1.75, -1.5,
        -1.25});  // Maximal brake allowed for curvature induced brake.

const PiecewiseLinearFunction<double, double> kMaxLatAccPLF(
    std::vector<double>{3.0, 5.0, 10.0, 15.0, 30.0},
    std::vector<double>{2.0, 2.0, 1.8, 1.4, 1.2});

SpeedLimit BuildDefaultVSLimit(double speed_limit, double length) {
  PRINT_FUNCTION
  std::vector<SpeedLimit::SpeedLimitRange> speed_limit_ranges;
  speed_limit_ranges.push_back(
      {.start_s = 0.0,
       .end_s = length,
       .speed_limit = speed_limit,
       .info = SpeedLimitTypeProto_Type_Name(SpeedLimitTypeProto::DEFAULT)});
  return SpeedLimit(speed_limit_ranges);
}

SpeedLimit BuildVSLimit(const PiecewiseLinearFunction<double>& kappa_s,
                        double cur_v,
                        AccPreliminarySpeedDebugProto* debug_proto) {
  constexpr double kSpeedLimitSampleResolution = 2.0;
  constexpr int kSpeedLimitMinSampleNum = 2;
  const double plf_start_s = kappa_s.x().front();
  const double plf_end_s = kappa_s.x().back();
  const int num_pts = std::max(static_cast<int>((plf_end_s - plf_start_s) /
                                                kSpeedLimitSampleResolution) +
                                   1,
                               kSpeedLimitMinSampleNum);
  std::vector<SpeedLimit::SpeedLimitRange> speed_limit_ranges;
  speed_limit_ranges.reserve(num_pts);
  const double max_lat_acc = kMaxLatAccPLF(cur_v);
  const auto compute_speed_limit = [&max_lat_acc, &cur_v,
                                    &kappa_s](double cur_s) {
    double v_limit_by_kappa = std::min(
        kDefaultAccSpeedLimitMps,
        sqrt(max_lat_acc / (std::fabs(kappa_s(cur_s)) + kKappaEpsilon)));
    // A safe guard to avoid too large speed limit drop
    constexpr double kTEpsilon = 1e-1;
    const double approximate_t = cur_s / (cur_v + kTEpsilon);
    v_limit_by_kappa = std::max(
        v_limit_by_kappa,
        cur_v + approximate_t * kCurvatureSpeedLimitMaxBrakePLF(cur_v));
    return v_limit_by_kappa;
  };

  // first: s second: v
  std::pair<double, double> prev_speed_limit_point =
      std::make_pair(plf_start_s, compute_speed_limit(plf_start_s));
  *debug_proto->add_speed_limits() =
      ToSpeedLimitProto(prev_speed_limit_point.second, std::nullopt,
                        absl::StrFormat("s(%.3f)", plf_start_s), CURVATURE);
  const double sample_solution = (plf_end_s - plf_start_s) / (num_pts - 1);

  double min_limit = prev_speed_limit_point.second;
  double min_limit_s = prev_speed_limit_point.first;
  for (int i = 1; i < num_pts; ++i) {
    const double cur_s = i * sample_solution + plf_start_s;
    const double v_limit_by_kappa =
        std::min(compute_speed_limit(cur_s), prev_speed_limit_point.second);
    // Ignore speed limit diff less than 1.0 m/s.
    constexpr double kApproxSpeedLimitEpsilon = 1.0;
    if (std::fabs(v_limit_by_kappa - prev_speed_limit_point.second) >
            kApproxSpeedLimitEpsilon ||
        i == num_pts - 1) {
      speed_limit_ranges.push_back(
          {.start_s = prev_speed_limit_point.first,
           .end_s = cur_s,
           .speed_limit = prev_speed_limit_point.second,
           .info =
               SpeedLimitTypeProto_Type_Name(SpeedLimitTypeProto::CURVATURE)});
      VLOG(2) << absl::StrFormat(
          "Speed limit by kappa:  start_s: %.3f, end_s: %.3f, speed_limit: "
          "%.3f.",
          prev_speed_limit_point.first, cur_s, prev_speed_limit_point.second);
      prev_speed_limit_point = std::make_pair(cur_s, v_limit_by_kappa);
      if (prev_speed_limit_point.second < min_limit) {
        min_limit = prev_speed_limit_point.second;
        min_limit_s = prev_speed_limit_point.first;
      }
    }
  }
  *debug_proto->add_speed_limits() =
      ToSpeedLimitProto(min_limit, std::nullopt,
                        absl::StrFormat("min_s:%.3f", min_limit_s), CURVATURE);
  return SpeedLimit(speed_limit_ranges);
}
}  // namespace internal

AccPreliminarySpeedOutput RunAccPreliminarySpeed(
    const AccPreliminarySpeedInput& input) {
  PRINT_FUNCTION
  QCHECK_NOTNULL(input.plan_start_point);
  QCHECK_NOTNULL(input.corridor);
  QCHECK_NOTNULL(input.targets);

  AccPreliminarySpeedDebugProto debug_proto;
  // 1. Generate stationary speed reference and limit when acc in standwait
  // mode.
  if (input.is_acc_standwait) {
    return AccPreliminarySpeedOutput{
        .speed_limit = internal::BuildDefaultVSLimit(/*speed_limit=*/0.0,
                                                     /*length=*/0.0),
    };
  }

  // 2. Compute curvature speed limit.
  auto curvature_speed_limit = internal::BuildVSLimit(
      input.corridor->kappa_s, input.plan_start_point->v(), &debug_proto);
  return AccPreliminarySpeedOutput{
      .speed_limit = std::move(curvature_speed_limit),
      .preliminary_speed_debug = std::move(debug_proto),
  };
}
}  // namespace planner
}  // namespace qcraft
