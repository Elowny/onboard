#ifndef ONBOARD_NETS_CUTIN_SL_NET_J5_H_
#define ONBOARD_NETS_CUTIN_SL_NET_J5_H_

#include <vector>

#include "onboard/math/piecewise_linear_function.h"

namespace qcraft {
namespace prediction {
namespace cutin_sl_net_j5 {
inline constexpr int kMaxPredObjectsNum = 64;
inline constexpr int kShapetDim = 4;

inline constexpr int kCoords = 2;
inline constexpr int kOutCoords = 5;
inline constexpr int kTrajectoryNum = 3;
inline constexpr int kObjectTypeDim = 16;
inline constexpr int kHistoryNum = 10;
inline constexpr float kHistoryStepLen = 0.1;  // Seconds.
inline constexpr int kFutureNum = 80;
inline constexpr int kStopInfoDim = 3;
// objects
inline constexpr int kMaxOtherObjsNum = 7;
// lane
inline constexpr double kAvMapRadius = 210.0;
inline constexpr int kSegDim = 4;
inline constexpr int kLaneSegsNum = 5;
inline constexpr int kLaneTypeDim = 16;
inline constexpr int kLaneLightDim = 4;
inline constexpr int kMaxLaneCenterNum = 64;
inline constexpr int kMaxLaneBoundaryNum = 32;
inline constexpr int kMaxCrossWalkNum = 16;

// scale
inline constexpr float kObjectWidthScale = 3.0f;
inline constexpr float kObjectLengthScale = 10.0f;

// Inv scale, do not modify here
inline constexpr float kInvObjectWidthScale = 1.0f / kObjectWidthScale;
inline constexpr float kInvObjectLengthScale = 1.0f / kObjectLengthScale;

// Agent scale
inline constexpr float kAgentSLPosSScale = 100.0f;
inline constexpr float kAgentSLPosLScale = 20.0f;
inline constexpr float kAgentSLSpeedSScale = 40.0f;
inline constexpr float kAgentSLSpeedLScale = 5.0f;
inline constexpr float kAgentSLShapeSScale = 100.0f;
inline constexpr float kAgentSLShapeLScale = 20.0f;
inline constexpr float kAgentDistToLaneBoundaryScale = 20.0f;

// Actor scale
inline constexpr float kActorSLPosSScale = 200.0f;
inline constexpr float kActorSLPosLScale = 50.0f;
inline constexpr float kActorSLSpeedSScale = 40.0f;
inline constexpr float kActorSLSpeedLScale = 20.0f;
inline constexpr float kActorSLShapeSScale = 200.0f;
inline constexpr float kActorSLShapeLScale = 50.0f;
inline constexpr float kActorRelSLSpeedSScale = 80.0f;
inline constexpr float kActorRelSLSpeedLScale = 20.0f;
inline constexpr float kActorRelDistScale = 200.0f;

// Lane scale
inline constexpr float kLaneCoordScale = 200.0f;
inline constexpr float kLaneSegmentLengthScale = 10.0f;
inline constexpr float kLaneLightsNumScale = 5.0f;

// Agent inv scale
inline constexpr float kInvAgentSLPosSScale = 1.0f / kAgentSLPosSScale;
inline constexpr float kInvAgentSLPosLScale = 1.0f / kAgentSLPosLScale;
inline constexpr float kInvAgentSLSpeedSScale = 1.0f / kAgentSLSpeedSScale;
inline constexpr float kInvAgentSLSpeedLScale = 1.0f / kAgentSLSpeedLScale;
inline constexpr float kInvAgentSLShapeSScale = 1.0f / kAgentSLShapeSScale;
inline constexpr float kInvAgentSLShapeLScale = 1.0f / kAgentSLShapeLScale;
inline constexpr float kInvAgentDistToLaneBoundaryScale =
    1.0f / kAgentDistToLaneBoundaryScale;

// Actor inv scale
inline constexpr float kInvActorSLPosSScale = 1.0f / kActorSLPosSScale;
inline constexpr float kInvActorSLPosLScale = 1.0f / kActorSLPosLScale;
inline constexpr float kInvActorSLSpeedSScale = 1.0f / kActorSLSpeedSScale;
inline constexpr float kInvActorSLSpeedLScale = 1.0f / kActorSLSpeedLScale;
inline constexpr float kInvActorSLShapeSScale = 1.0f / kActorSLShapeSScale;
inline constexpr float kInvActorSLShapeLScale = 1.0f / kActorSLShapeLScale;
inline constexpr float kInvActorRelSLSpeedSScale =
    1.0f / kActorRelSLSpeedSScale;
inline constexpr float kInvActorRelSLSpeedLScale =
    1.0f / kActorRelSLSpeedLScale;
inline constexpr float kInvActorRelDistScale = 1.0f / kActorRelDistScale;

// Lane inv scale
inline constexpr float kInvLaneCoordScale = 1.0f / kLaneCoordScale;
inline constexpr float kInvLaneSegmentLengthScale =
    1.0f / kLaneSegmentLengthScale;
inline constexpr float kInvLaneLightsNumScale = 1.0f / kLaneLightsNumScale;

}  // namespace cutin_sl_net_j5
}  // namespace prediction
}  // namespace qcraft
#endif  // ONBOARD_NETS_CUTIN_SL_NET_J5_H_
