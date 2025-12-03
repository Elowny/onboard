#include "onboard/planner/speed/vt_speed_limit.h"

#include "onboard/lite/check.h"

namespace qcraft::planner {

void MergeVtSpeedLimit(const VtSpeedLimit& source, VtSpeedLimit* target) {
  QCHECK_NOTNULL(target);
  if (source.size() != target->size()) return;
  for (int i = 0; i < source.size(); ++i) {
    if (source[i].speed_limit < (*target)[i].speed_limit) {
      (*target)[i] = source[i];
    }
  }
  return;
}

}  // namespace qcraft::planner
