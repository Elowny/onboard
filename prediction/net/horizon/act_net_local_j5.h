#ifndef ONBOARD_PREDICTION_NET_HORIZON_ACT_NET_LOCAL_J5_H_
#define ONBOARD_PREDICTION_NET_HORIZON_ACT_NET_LOCAL_J5_H_

#include <vector>

#include "onboard/math/piecewise_linear_function.h"

namespace qcraft {
namespace prediction {
namespace actnetlocalj5 {
[[maybe_unused]] static constexpr struct ActNetLocalJ5Config {
  int max_other_objs_num = 31;
  int coord_num = 2;
  int lane_light_dim = 4;
  int max_lane_seg_num = 5;
  int hist_num = 10;
  int max_lc_num = 160;
  int max_lb_num = 64;
  int future_num = 80;
} kActNetLocalJ5Config;

inline constexpr int kMaxPredObjectsNum = 64;
inline constexpr int kOutCoords = 5;
inline constexpr int kFutureNum = 80;
inline constexpr int kTrajectoryNum = 3;
inline constexpr double kProbThreshold = 0.15;

// scale
inline constexpr float kSpeedScale = 30.0f;
inline constexpr float kCoordScale = 100.0f;
inline constexpr float kMaxSegLen = 10.0f;
inline constexpr float kSpeedLimitScale = 120.0f;
inline constexpr float kObjectWidthScale = 3.0f;
inline constexpr float kObjectLengthScale = 10.0f;

// Inv scale, do not modify here
inline const float kInvSpeedScale = 1.0f / kSpeedScale;
inline const float kInvCoordScale = 1.0f / kCoordScale;
inline const float kInvMaxSegLen = 1.0f / kMaxSegLen;
inline const float kInvSpeedLimitScale = 1.0f / kSpeedLimitScale;
inline const float kInvObjectWidthScale = 1.0f / kObjectWidthScale;
inline const float kInvObjectLengthScale = 1.0f / kObjectLengthScale;

const PiecewiseLinearFunction<double, double> kScanBoxFrontPlf(
    std::vector<double>{5.0, 10.0, 15.0, 20.0},
    std::vector<double>{60.0, 90.0, 120.0, 150.0});

const PiecewiseLinearFunction<double, double> kScanBoxBackPlf(
    std::vector<double>{5.0, 10.0, 15.0, 20.0},
    std::vector<double>{30.0, 20.0, 20.0, 15.0});

const PiecewiseLinearFunction<double, double> kScanBoxFrontHalfWidthPlf(
    std::vector<double>{5.0, 10.0, 15.0, 20.0},
    std::vector<double>{60.0, 40.0, 30.0, 30.0});

}  // namespace actnetlocalj5
}  // namespace prediction
}  // namespace qcraft

#endif  // ONBOARD_PREDICTION_NET_HORIZON_ACT_NET_LOCAL_J5_H_
