#include "onboard/planner/router/util/async_retry.h"

#include "gtest/gtest.h"

namespace qcraft::planner {
namespace {

TEST(AsyncRetryTest, Retry) {
  AsyncRetry async_retry;
  async_retry.retry_interval_secs = 1;
  constexpr double kNowSeconds = 100.0;
  {
    // Test shoule_retry settings.
    async_retry.should_retry = true;
    async_retry.Retry(kNowSeconds, [] {});
    EXPECT_EQ(async_retry.retry_times, 1);
    async_retry.should_retry = false;
    async_retry.Retry(kNowSeconds + 10, [] {});
    EXPECT_EQ(async_retry.retry_times, 1);
    EXPECT_FALSE(async_retry.DebugString().empty());
  }

  {
    // Test interval secs
    async_retry.Reset();
    async_retry.should_retry = true;
    async_retry.Retry(kNowSeconds, [] {});
    async_retry.Retry(kNowSeconds, [] {});
    EXPECT_EQ(async_retry.retry_times, 2);
    async_retry.Retry(kNowSeconds + 1, [] {});
    EXPECT_EQ(async_retry.retry_times, 3);
    async_retry.Retry(kNowSeconds + 2, [] {});
    EXPECT_EQ(async_retry.retry_times, 4);
    async_retry.retry_interval_secs = 2;
    async_retry.Retry(kNowSeconds + 3, [] {});
    EXPECT_EQ(async_retry.retry_times, 5);
  }

  {
    async_retry.Reset();
    async_retry.should_retry = true;
    async_retry.max_retry_times = 3;
    for (int i = 0; i < 3; i++) {
      async_retry.Retry(kNowSeconds + 2 * i, [] {});
      EXPECT_NEAR(async_retry.last_retry_time_secs, kNowSeconds + 2 * i, 1E-6);
      EXPECT_EQ(async_retry.retry_times, i + 1);
    }
  }
}

}  // namespace
}  // namespace qcraft::planner
