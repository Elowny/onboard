#include "onboard/planner/router/util/async_retry.h"

#include <cstdint>
#include <sstream>
namespace qcraft::planner {

void AsyncRetry::Reset() {
  should_retry = false;
  retry_times = 0;
  last_retry_time_secs = 0.0;
}

std::string AsyncRetry::DebugString() const {
  std::stringstream ss;
  ss << "should_retry:" << should_retry << ", retry_times: " << retry_times
     << ", max_retry_times: " << max_retry_times << ", last_retry_time_secs: "
     << static_cast<std::int64_t>(last_retry_time_secs)
     << ", retry_interval_secs: " << retry_interval_secs;
  return ss.str();
}

}  // namespace qcraft::planner
