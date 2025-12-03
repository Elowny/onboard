#ifndef ONBOARD_PLANNER_ROUTER_ROUTER_DEFS_H_
#define ONBOARD_PLANNER_ROUTER_ROUTER_DEFS_H_

#include "onboard/planner/planner_defs.h"

namespace qcraft::planner {

inline constexpr int kLayerNumPerLcLen = 2;  // Should be larger than 1.
inline constexpr double kVertexSampleDist =
    kMinLcLaneLength / kLayerNumPerLcLen;

inline constexpr double kDefaultSpeedLimit = 40.0;  // kph.
// TODO(zuowei): Write to config.
inline constexpr double kHighwayLowestSpeedLimit = 80.0;               // kph.
inline constexpr double kHighwayOverwrittenSpeedLimit = 120.0;         // kph.
inline constexpr double kRampOverwrittenSpeedLimit = 60.0;             // kph.
inline constexpr double kHighwayJunctionOverwrittenSpeedLimit = 70.0;  // kph.

inline constexpr double kPreviewEgoInHighwayDist = 100.0;  // m.

// Better if use `constinit` when C++20 supported, this value should be read
// from the RouteParamProto config.
namespace route {
namespace noa {
inline constexpr double kNaviInfoPreviewDistance = 3000.0;     // m
inline constexpr double kNaviInfoBackwardExtendLength = 10.0;  // m
inline constexpr double kSearchLaneRadius = 20.0;              // m
namespace odc {
// The preconditions to calc hd distance.
inline constexpr int kSdHdDistDiffMaxError = 200;  // m
inline constexpr int kHdDistHorizon = 2000;        // m
inline constexpr int kHdMinBoundary = 50;          // m
// The odd event
inline constexpr int kEventHorizonAtMost = 2000;      // m
inline constexpr int kNavActionHorizonAtMost = 3000;  // m
}  // namespace odc
}  // namespace noa
}  // namespace route

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_ROUTER_ROUTER_DEFS_H_
