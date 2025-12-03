#ifndef ONBOARD_PLANNER_COMMON_MULTI_TIMER_UTIL_H_
#define ONBOARD_PLANNER_COMMON_MULTI_TIMER_UTIL_H_

#include "onboard/global/timer.h"
#include "onboard/proto/timer_report.pb.h"
#include "onboard/utils/source_location.h"
namespace qcraft::planner {

void GetMultiTimerReport(const ScopedMultiTimer& timer,
                         MultiTimerReportProto* report_proto);

void PrintMultiTimerReportStat(const ScopedMultiTimer& timer,
                               SourceLocation loc = QCRAFT_LOC);
void PrintMultiTimerReportStat(const MultiTimerReportProto& report_proto);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_COMMON_MULTI_TIMER_UTIL_H_
