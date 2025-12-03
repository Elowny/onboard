#ifndef ONBOARD_NETS_LANE_SELECTION_NET_J5_H_
#define ONBOARD_NETS_LANE_SELECTION_NET_J5_H_

namespace qcraft {
namespace prediction {
namespace lane_selection_net {
inline constexpr int kLaneSelectionFeatureHistoryStepNum = 21;
inline constexpr double kSLSpeedMax = 50;
inline constexpr double kScanBoxFront = 150.0;
inline constexpr double kScanBoxBack = 30.0;
inline constexpr double kScanBoxFrontHalfWidth = 60.0;
inline constexpr int kMaxPredLanePathsNum = 64;
inline constexpr int kCoords = 2;
inline constexpr double kSLCorMax = 500.0;
inline constexpr int kOutCoords = 5;
inline constexpr int kTrajectoryNum = 3;
inline constexpr int kObjectTypeDim = 16;
inline constexpr int kHistoryNum = 10;
inline constexpr float kHistoryStepLen = 0.1;  // Seconds.
inline constexpr float kHistoryTime = 1.0;     // s
inline constexpr int kFutureNum = 80;
inline constexpr int kStopInfoDim = 3;
inline constexpr float kTrajGenThreshould = 0.8;
inline constexpr float kDefaultLaneSelectionHalfLaneWidth = 3.75 / 2.0;
constexpr int kMaxConsideredLanePathNum = 3;

// objects
inline constexpr int kMaxOtherObjsNum = 8;
inline constexpr int kLaneSelectionMaxPredObjNum =
    kMaxPredLanePathsNum / kMaxConsideredLanePathNum;

// lane
inline constexpr int kSegDim = 4;
inline constexpr int kLaneSegsNum = 5;
inline constexpr int kLaneTypeDim = 16;
inline constexpr int kLaneLightDim = 4;
inline constexpr int kMaxLaneCenterNum = 160;
inline constexpr int kMaxLaneBoundaryNum = 64;
inline constexpr int kMaxCrossWalkNum = 16;
// scale
inline constexpr float kAgentSBackDist = 50.0f;    // m
inline constexpr float kAgentSFrontDist = 200.0f;  // m
inline constexpr float kAgentSfullScale = kAgentSBackDist + kAgentSFrontDist;
inline constexpr float kAgentLScale = 20.0f;
inline constexpr float kAgentDistToLeftLaneBoundaryScale = 20.0f;
inline constexpr float kAgentDistToRightLaneBoundaryScale = 20.0f;
inline constexpr float kAgentLaneWidthScale = 4.0f;
inline constexpr float kAgentLLeapScale = 5.0f;
inline constexpr float kObjectWidthScale = 3.0f;
inline constexpr float kObjectLengthScale = 10.0f;

// Inv scale, do not modify here
inline constexpr float kInvAgentSfullScale = 1.0f / kAgentSfullScale;
inline constexpr float kInvAgentLScale = 1.0f / kAgentLScale;
inline constexpr float kInvAgentDistToLeftLaneBoundaryScale =
    1.0f / kAgentDistToLeftLaneBoundaryScale;
inline constexpr float kInvAgentDistToRightLaneBoundaryScale =
    1.0f / kAgentDistToRightLaneBoundaryScale;
inline constexpr float kInvAgentLaneWidthScale = 1.0f / kAgentLaneWidthScale;
inline constexpr float kInvAgentLLeapScale = 1.0f / kAgentLLeapScale;
inline constexpr float kInvObjectWidthScale = 1.0f / kObjectWidthScale;
inline constexpr float kInvObjectLengthScale = 1.0f / kObjectLengthScale;

}  // namespace lane_selection_net
}  // namespace prediction
}  // namespace qcraft
#endif  // ONBOARD_NETS_LANE_SELECTION_NET_J5_H_
