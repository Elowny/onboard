#ifndef ONBOARD_PREDICTION_PREDICTION_DEFS_H_
#define ONBOARD_PREDICTION_PREDICTION_DEFS_H_

#include <array>
#include <cmath>
#include <map>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"

#include "onboard/math/geometry/box2d.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/math/vec.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/prediction/proto/prediction_common.pb.h"
#include "onboard/proto/perception.pb.h"
namespace qcraft {
namespace prediction {

using ResampledObjectsHistory = std::vector<std::vector<qcraft::ObjectProto>>;

using ObjectIDType = std::string;
using ProbTrajPair = std::pair<double, std::vector<Vec2d>>;

using ObjectProbTrajs = std::vector<ProbTrajPair>;
using ObjectsProbTrajs = std::map<std::string, ObjectProbTrajs>;

enum class PredictTypePrio { HIGH, MED, LOW };  // NOLINT
using TypePrioMap =
    std::map<PredictTypePrio, const absl::flat_hash_set<ObjectType>>;

struct ObjectMotionState {
  double timestamp;
  Vec2d pos;
  double heading;
  Vec2d vel;
  Box2d bbox;

  std::string DebugString() const {
    return absl::StrFormat("timestamp: %.6f, pos: %s, heading: %.6f, vel: %s.",
                           timestamp, pos.DebugString(), heading,
                           vel.DebugString());
  }
};

struct ObjectMotionHistory {
  ObjectIDType id;
  ObjectType type;
  std::vector<ObjectMotionState> states;
};
using ObjectsMotionHistory = std::vector<ObjectMotionHistory>;

// Agent centric net outputs with uncertainty.
using NLLTrajPoint = std::array<double, 5>;  // x, y, s1, s2, c.
struct AgentCentricObjectProbTraj {
  double mode_prob = 0.0;
  std::array<double, 3> relation_probs = {0.0, 0.0, 0.0};  // void, yield, pass.
  std::vector<NLLTrajPoint> traj_points;
  double rot_rad = 0.0;
};
using AgentCentricObjectProbTrajs = std::vector<AgentCentricObjectProbTraj>;
using AgentCentricObjectsProbTrajs =
    std::map<std::string, AgentCentricObjectProbTrajs>;

struct AgentCentricObjectOut {
  AgentCentricObjectProbTrajs prob_trajs;
  std::optional<double> startup_prob;
};
using AgentCentricObjectsOut = std::map<std::string, AgentCentricObjectOut>;

// Cutin net outputs.
using CutinTrajPoint = std::array<double, 2>;  // x, y
struct CutinObjectProbTraj {
  double mode_prob = 0.0;
  std::vector<CutinTrajPoint> traj_points;
  double rot_rad = 0.0;
};

using CutinObjectProbTrajs = std::vector<CutinObjectProbTraj>;
using CutinObjectsProbTrajs = std::map<std::string, CutinObjectProbTrajs>;

struct CutinObjectOut {
  CutinObjectProbTrajs prob_trajs;
  std::vector<double> channle_probs;
  int predicted_channel;
  int cur_channel;
};

using CutinObjectsOut = std::map<std::string, CutinObjectOut>;

// CutinSLNet output
struct CutinSLObjectOut {
  std::vector<double> channle_probs;
  int predicted_channel;
  int cur_channel;
};
// LaneSelectionNet input
using AgentDrivePassagesMap =
    std::map<ObjectIDType, std::vector<const planner::DrivePassage*>>;

// LaneSelectionNet output
struct LaneSelectionInferencerOutputInfo {
  std::vector<float> dp_scores;
  std::vector<bool> is_valid;
};
using LaneSelectionInferencerOutputMap =
    std::map<ObjectIDType, LaneSelectionInferencerOutputInfo>;

struct LaneSelectionObjectOut {
  std::map<planner::DrivePassage, double> scores_for_dps;
};
using LaneSelectionObjectsOut =
    absl::flat_hash_map<std::string, LaneSelectionObjectOut>;

struct FeatureScaleConfig {
  std::vector<float> inv_scale;
  std::vector<float> zero;
  bool clamp = false;
  std::vector<float> min;
  std::vector<float> max;

  // Make sure to use the correct one!
  inline float ScaleOnly(float value, int idx) const {
    return value * inv_scale[idx];
  }

  inline float ScaleWithClamp(float value, int idx) const {
    return std::clamp(value * inv_scale[idx], min[idx], max[idx]);
  }

  inline float ScaleWithZeroAndClamp(float value, int idx) const {
    return std::clamp((value - zero[idx]) * inv_scale[idx], min[idx], max[idx]);
  }

  inline float ScaleWithZero(float value, int idx) const {
    return (value - zero[idx]) * inv_scale[idx];
  }

  void FromProto(const FeatureScaleConfigProto& scale_proto) {
    const int size = scale_proto.scale_size();
    if (scale_proto.zero_size() != 0) {
      QCHECK_EQ(scale_proto.zero_size(), size);
    }
    inv_scale.reserve(size);
    zero.reserve(scale_proto.zero_size());
    for (const auto& scale : scale_proto.scale()) {
      inv_scale.push_back(1.0 / static_cast<float>(scale));
    }
    for (const auto& zero_val : scale_proto.zero()) {
      zero.push_back(static_cast<float>(zero_val));
    }
    clamp = scale_proto.clamp();
    if (clamp) {
      QCHECK_EQ(scale_proto.min_size(), size);
      QCHECK_EQ(scale_proto.max_size(), size);
      min.reserve(scale_proto.min_size());
      max.reserve(scale_proto.max_size());
      for (const auto& min_val : scale_proto.min()) {
        min.push_back(min_val);
      }
      for (const auto& max_val : scale_proto.max()) {
        max.push_back(max_val);
      }
    }
  }
  std::string DebugString() const {
    return absl::StrFormat(
        "inv_scale: [%s], zero: [%s], clamp: %d, min: [%s], max: [%s]",
        absl::StrJoin(inv_scale, ","), absl::StrJoin(zero, ","), clamp,
        absl::StrJoin(min, ","), absl::StrJoin(max, ","));
  }
};

// Object history buffer related info
inline constexpr double kTTLSteps = 100;
inline constexpr double kShortTermDt = 0.01;
inline constexpr double kLongTermDt = 1.0;
inline constexpr double kLongTermHistoryLen = 15.0;

inline constexpr double kMaxVehicleAcc = 3.0;
inline constexpr double kMaxVehicleBrake = -8.0;

inline constexpr double kEpsilon = 1e-8;

inline constexpr char kAvObjectId[] = "AV";
inline constexpr char kInvalidObjectId[] = "NA";

// Feature 2.0 motion history configuration.
inline constexpr double kFeatureV2HistoryStepNum = 21;
inline constexpr double kLaneSelectionHistoryStepNum = 10;
inline constexpr double kFeatureV2HistoryStepLen = 0.1;  // Seconds.
// Feature 2.0 map sampling configuration.
inline constexpr double kFeatureV2MaxMapSampleLen = 10.0;  // m.
inline constexpr int kFeatureV2MapSegmentNum = 5;

inline constexpr double kPredictionTimeStep = 0.1;  // Seconds.
inline constexpr double kPredictionDuration =
    10.0;  // Prediction seconds for non-stationary trajectory.
inline constexpr double kPredictionMinDuration = 6.0;      // s
inline constexpr double kCutinSLHorizon = 3.0;             // s.
inline constexpr double kNormalVehicleCutinHorizon = 2.5;  // s.
inline constexpr double kLargeVehicleCutinHorizon = 1.0;   // s
inline constexpr double kCutinPredictionDuration =
    8.0;  // Prediction seconds for cutin non-stationary trajectory.
inline constexpr double kParkingPredictionDuration =
    3.0;  // Prediction seconds under parking scenario.
inline const int kPredictionPointNum =
    static_cast<int>(kPredictionDuration / kPredictionTimeStep);
inline const int kPredictionMinPointNum =
    static_cast<int>(kPredictionMinDuration / kPredictionTimeStep);

// for cutin prediction
inline constexpr int kCenterChannel = 2;
inline constexpr double kChannelWidth = 2.0;                    // m
inline constexpr double kHalfChannelWidth = kChannelWidth / 2;  // m

inline constexpr double kEmergencyGuardHorizon = 3.0;  // s.
inline constexpr double kSafeHorizon = 8.0;            // s.
inline constexpr double kComfortableHorizon = 8.0;     // s.

inline constexpr double kVehCurvatureLimit = 0.25;    // m^-1.
inline constexpr double kVehLateralAccelLimit = 2.5;  // m^-1.

inline constexpr double kHistoryLen = 1.5;
inline constexpr double kHistoryTime = 1.0;  // s.

inline constexpr double kTooShortTrajLen = 0.5;  // m.

inline constexpr double kAccelerationFitTime = 1.0;  // s.

inline constexpr double kLateralSpeedClamp = 2.5;          // m/s.
inline constexpr double kLateralSpeedLookAheadTime = 3.0;  // s.
inline constexpr double kDefaultHalfLaneWidth = 2.0;
inline constexpr double kDefaultLaneWidth = 2.0 * kDefaultHalfLaneWidth;

inline constexpr double kMinimalCutinProb = 0.2;
inline constexpr double kMinimalCenterCutinProb = 0.4;

// Rectify
inline constexpr double kLargeVehicleNoRectifyDist = 15.0;   // m
inline constexpr double kNormalVehicleNoRectifyDist = 20.0;  // m

// Slow cutin
inline constexpr double kSlowVelTreshold = 3.0;                        // m/s
inline constexpr double kSlowAngleDiffThreshold = 3.0 * M_PI / 180.0;  // rad
inline constexpr double kSlowCutinPredictionDuration = 10.0;           // s

// Region box parameters
inline constexpr double kPedestrianScanBoxFront = 40.0;
inline constexpr double kPedestrianScanBoxBack = 15.0;
inline constexpr double kPedestrianScanBoxFrontHalfWidth = 30.0;

const PiecewiseLinearFunction<double, double> kScanBoxFrontPlf(
    std::vector<double>{5.0, 10.0, 15.0, 20.0},
    std::vector<double>{60.0, 90.0, 120.0, 150.0});

inline constexpr double kScanBoxBack = 40.0;

const PiecewiseLinearFunction<double, double> kScanBoxFrontHalfWidthPlf(
    std::vector<double>{5.0, 10.0, 15.0, 20.0},
    std::vector<double>{60.0, 40.0, 30.0, 30.0});

// Drive passage realted
inline constexpr double kMaxLateralBoundaryForPredictionDp = 3.0;
inline constexpr double kMaplessObjectDrivePassageFrontLength = 100.0;
inline constexpr double kMaplessObjectDrivePassageBackLength = 30.0;
inline constexpr double kObjectDrivePassageFrontLength = 120.0;
inline constexpr double kObjectDrivePassageBackLength = 40.0;

// Time to maintain heuristic acceleration.
inline constexpr double kMaintainAccTime = 1.5;  // s.

// J5 trajectory number
inline constexpr int kActNetJ5ModelTrajNum = 2;

// Post processing related constants.
inline constexpr int kPolyFitDownSampleStep = 3;

// Rule-based prediction parameters
inline constexpr double kNormalAngleDiffBuffer = 1.5 * M_PI / 180.0;  // rad
// Angle diff buffer should be lower for large vehicles since their lateral
// projections are larger.
inline constexpr double kLargeVehicleAngleDiffBuffer =
    1.0 * M_PI / 180.0;                                        // rad
inline constexpr double kLatSpeedBuffer = 0.3;                 // m/s
inline constexpr double kNormalDistToBoundBuffer = -0.1;       // m
inline constexpr double kLargeVehicleDistToBoundBuffer = 0.1;  // m

// Pole placement related stuff.
// Length data for pedestrian, bike, shunfeng, aion, jinlv minibus, polerstar
// bus.
const std::vector<double> kObjectLengthDataPoint = {1.0,   1.57,  2.95,
                                                    4.786, 5.995, 10.48};
const PiecewiseLinearFunction<double, double> kLengthToWheelbasePlf(
    kObjectLengthDataPoint,
    std::vector<double>{0.6, 1.22, 2.1, 2.92, 3.85, 5.89});
const PiecewiseLinearFunction<double, double> kLengthToMaxFrontSteerPlf(
    kObjectLengthDataPoint,
    std::vector<double>{M_PI / 3.0, 0.49, 0.496, 0.546, 0.583, 0.67});
struct AVObjectRelation {
  double no_relation;
  double yield;
  double pass;

  std::string DebugString() const {
    return absl::StrFormat("void: %f, yield: %f, pass: %f", no_relation, yield,
                           pass);
  }
};

using CutinSLObjectsOut = std::map<std::string, CutinSLObjectOut>;

const PiecewiseLinearFunction<double, double> kLowSpeedUnderestimateRatioPlf(
    {0.0, 1.0, 2.0, 3.0}, {0.4, 0.5, 0.75, 1.0});
}  // namespace prediction
}  // namespace qcraft

#endif  // ONBOARD_PREDICTION_PREDICTION_DEFS_H_
