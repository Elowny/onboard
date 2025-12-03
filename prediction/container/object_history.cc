#include "onboard/prediction/container/object_history.h"

namespace qcraft {
namespace prediction {
bool ObjectHistory::UpdateStopTimeInfo() {
  const auto object_history = this->GetHistory();
  const auto len = object_history.size();
  if (len == 0) {
    return false;
  }
  const double last_time = object_history[len - 1].time;
  stop_time_info_.last_time = last_time;

  if (len == 1) {
    stop_time_info_.last_moved_since = last_time;
    stop_time_info_.prev_stopped_since = last_time;
    stop_time_info_.stopped_since = last_time;
    return true;
  }
  const bool is_stationary = object_history.back().val.IsStationary();
  const double prev_last_time = object_history[len - 2].time;
  if (!is_stationary) {
    if (stop_time_info_.stopped_since < prev_last_time) {  // stop - move
      stop_time_info_.prev_stopped_since = stop_time_info_.stopped_since;
      stop_time_info_.last_moved_since = stop_time_info_.last_time;
    }
    stop_time_info_.stopped_since = stop_time_info_.last_time;
  }
  return true;
}

}  // namespace prediction
}  // namespace qcraft
