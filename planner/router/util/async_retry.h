#ifndef ONBOARD_PLANNER_ROUTER_UTIL_ASYNC_RETRY_H_
#define ONBOARD_PLANNER_ROUTER_UTIL_ASYNC_RETRY_H_

#include <string>

/**
 * @brief It is an async retry, should set status after call Retry() each time .
 * IMPORTANT. To retry or not just depends on the status `should_retry` changed
 * by manual other than retry_fn().
 */
namespace qcraft::planner {
struct AsyncRetry {
  template <typename fn>
  void Retry(double now_secs, fn&& retry_fn) {
    if (CanRetry()) {
      last_retry_time_secs = now_secs;
      ++retry_times;
      retry_fn();
    }
  }

  bool CanRetry() const {
    return retry_times < max_retry_times && should_retry;
  }

  void Reset();
  std::string DebugString() const;

  bool should_retry = false;
  int retry_times = 0;
  int max_retry_times = 10;
  double last_retry_time_secs = 0.0;
  double retry_interval_secs = 1.0;
};

}  // namespace qcraft::planner
#endif  // ONBOARD_PLANNER_ROUTER_UTIL_ASYNC_RETRY_H_
