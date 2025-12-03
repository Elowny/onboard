#ifndef ONBOARD_CONTROL_CONTROL_DEFS_H_
#define ONBOARD_CONTROL_CONTROL_DEFS_H_

#include <math.h>

namespace qcraft::control {

inline constexpr double kControlInterval = 0.02;                     // s.
inline constexpr double kControlFrequency = 1.0 / kControlInterval;  // Hz.
inline constexpr double kGravitationalAcceleration = 9.80665;        // m/s^2.
inline constexpr double kSinSlopeLimit =
    0.2588;  // slope angle limit, 15 degrees.
inline constexpr int kTControlHorizon = 10;
inline constexpr int kSControlHorizon = 10;
inline constexpr double kDmSpeedThreshold =
    5.0;  // m/s, dynamic model speed threshold.

}  // namespace qcraft::control

#endif  // ONBOARD_CONTROL_CONTROL_DEFS_H_
