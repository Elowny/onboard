#ifndef ONBOARD_NETS_ACT_NET_J5_H_
#define ONBOARD_NETS_ACT_NET_J5_H_

#include <vector>

#include "onboard/math/piecewise_linear_function.h"

namespace qcraft {
namespace prediction {
namespace actnetj5 {
inline constexpr int kMaxPredObjectsNum = 64;

inline constexpr int kCoords = 2;
inline constexpr int kOutCoords = 5;
inline constexpr int kTrajectoryNum = 3;
inline constexpr int kObjectTypeDim = 16;
inline constexpr int kHistoryNum = 10;
inline constexpr float kHistoryStepLen = 0.1;  // Seconds.
inline constexpr int kFutureNum = 80;
inline constexpr int kStopInfoDim = 3;
// objects
inline constexpr int kMaxOtherObjsNum = 31;
// lane
inline constexpr int kSegDim = 4;
inline constexpr int kLaneSegsNum = 5;
inline constexpr int kLaneTypeDim = 16;
inline constexpr int kLaneLightDim = 4;
inline constexpr int kMaxLaneCenterNum = 160;
inline constexpr int kMaxLaneBoundaryNum = 64;
inline constexpr int kMaxCrossWalkNum = 16;
// scale
inline constexpr float kSpeedScale = 30.0f;
inline constexpr float kCoordScale = 100.0f;
inline constexpr float kMaxSegLen = 10.0f;
inline constexpr float kSpeedLimitScale = 120.0f;
inline constexpr float kObjectWidthScale = 3.0f;
inline constexpr float kObjectLengthScale = 10.0f;

// Inv scale, do not modify here
inline constexpr float kInvSpeedScale = 1.0f / kSpeedScale;
inline constexpr float kInvCoordScale = 1.0f / kCoordScale;
inline constexpr float kInvMaxSegLen = 1.0f / kMaxSegLen;
inline constexpr float kInvSpeedLimitScale = 1.0f / kSpeedLimitScale;
inline constexpr float kInvObjectWidthScale = 1.0f / kObjectWidthScale;
inline constexpr float kInvObjectLengthScale = 1.0f / kObjectLengthScale;

}  // namespace actnetj5
}  // namespace prediction
}  // namespace qcraft
#endif  // ONBOARD_NETS_ACT_NET_J5_H_
