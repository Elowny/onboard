#include "onboard/prediction/container/object_history_span.h"

#include <algorithm>
#include <iterator>

namespace qcraft {
namespace prediction {
ObjectHistorySpan ObjectHistorySpan::GetHistoryFrom(double t) const {
  auto it = std::upper_bound(begin(), end(), t);
  if (it != begin()) {
    std::advance(it, -1);
  }
  return ObjectHistorySpan(it, std::distance(it, end()));
}

}  // namespace prediction
}  // namespace qcraft
