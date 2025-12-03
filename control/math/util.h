#ifndef ONBOARD_CONTROL_MATH_UTIL_H_
#define ONBOARD_CONTROL_MATH_UTIL_H_

#include <numeric>

#include "boost/circular_buffer.hpp"

namespace qcraft::control {

template <typename T>
inline T ComputeMeanOfCircularBuffer(const boost::circular_buffer<T>& data) {
  if (data.empty()) return 0.0;
  return std::accumulate(data.begin(), data.end(), 0.0) / data.size();
}

}  // namespace qcraft::control

#endif  // ONBOARD_CONTROL_MATH_UTIL_H_
