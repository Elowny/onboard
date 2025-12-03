#include "onboard/planner/speed/st_boundary_with_decision.h"

#include <cmath>
#include <ostream>

#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"

namespace qcraft::planner {

StBoundaryWithDecision::StBoundaryWithDecision(StBoundaryRef raw_st_boundary)
    : raw_st_boundary_(std::move(raw_st_boundary)),
      st_boundary_(StBoundary::CopyInstance(*raw_st_boundary_)) {}

StBoundaryWithDecision::StBoundaryWithDecision(
    StBoundaryRef raw_st_boundary, StBoundaryProto::DecisionType decision_type,
    StBoundaryProto::DecisionReason decision_reason)
    : raw_st_boundary_(std::move(raw_st_boundary)),
      st_boundary_(StBoundary::CopyInstance(*raw_st_boundary_)),
      decision_type_(decision_type),
      decision_reason_(decision_reason) {}

StBoundaryWithDecision::StBoundaryWithDecision(
    StBoundaryRef raw_st_boundary, StBoundaryProto::DecisionType decision_type,
    StBoundaryProto::DecisionReason decision_reason, std::string decision_info,
    double follow_standstill_distance, double lead_standstill_distance,
    double pass_time, double yield_time)
    : raw_st_boundary_(std::move(raw_st_boundary)),
      st_boundary_(StBoundary::CopyInstance(*raw_st_boundary_)),
      decision_type_(decision_type),
      decision_reason_(decision_reason),
      decision_info_(std::move(decision_info)),
      follow_standstill_distance_(follow_standstill_distance),
      lead_standstill_distance_(lead_standstill_distance) {
  SetTimeBuffer(pass_time, yield_time);
}

void StBoundaryWithDecision::SetTimeBuffer(double pass_time,
                                           double yield_time) {
  // Reset to raw_st_boundary when set time buffers.
  st_boundary_ = StBoundary::CopyInstance(*raw_st_boundary_);
  pass_time_ = pass_time;
  yield_time_ = yield_time;
  st_boundary_->ExpandByT(pass_time_, yield_time_);
}

std::optional<std::pair<double, double>>
StBoundaryWithDecision::GetUnblockSRange(double curr_time,
                                         double path_end_s) const {
  if (curr_time < st_boundary()->min_t() ||
      curr_time > st_boundary()->max_t()) {
    return std::nullopt;
  }

  int left = 0;
  int right = 0;
  const auto& lower_points = st_boundary()->lower_points();
  const auto& upper_points = st_boundary()->upper_points();
  if (!st_boundary()->GetLowerPointsIndexRange(curr_time, &left, &right)) {
    QLOG(WARNING) << "Fail to get index range.";
    return std::nullopt;
  }
  QCHECK_NE(left, right);

  const double alpha = (curr_time - upper_points[left].t()) /
                       (upper_points[right].t() - upper_points[left].t());
  const double upper_cross_s =
      upper_points[left].s() +
      alpha * (upper_points[right].s() - upper_points[left].s());
  const double lower_cross_s =
      lower_points[left].s() +
      alpha * (lower_points[right].s() - lower_points[left].s());

  double lower_s = 0.0;
  double upper_s = path_end_s;
  if (decision_type_ == StBoundaryProto::FOLLOW) {
    upper_s = std::fmin(upper_s, lower_cross_s);
  } else if (decision_type_ == StBoundaryProto::LEAD) {
    lower_s = std::fmax(lower_s, upper_cross_s);
  } else {
    QLOG(FATAL) << "boundary_type is not supported. boundary_type: "
                << StBoundaryProto::DecisionType_Name(decision_type_);
    return std::nullopt;
  }
  return std::make_pair(lower_s, upper_s);
}

}  // namespace qcraft::planner
