#ifndef ONBOARD_PREDICTION_FEATURE_EXTRACTOR_MAP_SAMPLER_H_
#define ONBOARD_PREDICTION_FEATURE_EXTRACTOR_MAP_SAMPLER_H_

#include <array>
#include <memory>
#include <optional>
#include <vector>

// IWYU pragma: no_include <algorithm>  // for max

#include "absl/base/thread_annotations.h"
#include "absl/container/flat_hash_map.h"
#include "absl/hash/hash.h"
#include "absl/synchronization/mutex.h"

#include "onboard/maps/maps_common.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/geometry/segment2d.h"
#include "onboard/math/vec.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/prediction/container/traffic_light_manager.h"

namespace qcraft {
namespace prediction {
// Simplified polyline type.
enum class SampledPointType {
  kNone = 0,
  kCenterlineGeneral = 1,
  kCenterlineBicycleOnly = 2,
  kCenterlineBusOnly = 3,
  kCenterlineVirtual = 4,
  kCenterlineEmergency = 5,
  kCenterlineRamp = 6,
  kCenterlineWalking = 7,
  kCenterlineParking = 8,
  kCenterlineMixedCyclist = 9,
  kSolidDouble = 10,
  kSolid = 11,
  kBrokenLeftDouble = 12,
  kBrokenRightDouble = 13,
  kCurb = 14,
  kCrosswalk = 15,
  kBrokenWhite = 16,
  kBrokenYellow = 17,
};

struct SampledPolylineSeg {
  Segment2d seg;
  Vec2d unit_tangent;
  SampledPointType type;
  std::vector<int> lights = {0, 0, 0, 0};  // Count of green yellow red unknown.
  int speed_limit_kph;
  // Assuming same distance for center point to left and right boundary are the
  std::optional<double> distance_to_boundary;
  std::array<SampledPointType, 2> boundary_types;
};

struct SampledPolyline {
  mapping::ElementId id;
  std::vector<SampledPolylineSeg> segs;
};

struct SampledPolylineWithConstSegRef {
  mapping::ElementId id;
  std::vector<const SampledPolylineSeg*> seg_ptrs;
};

class MapSampler {
 public:
  enum class SampleType {
    EQUAL_DIST = 0,  // NOLINT
    ADAPTIVE = 1,    // NOLINT
  };
  // For EQUAL_DIST version, map elements are sampled uniformly given the
  // sample_ds. For adaptive version, the number of elements will always be
  // N*desired_num, sample_ds is modified acoordingly. For example, a lane of
  // 50m length with sample ds 10 will get 5 points of each 5m interval; a lane
  // of 55m length with sample ds 10 will get 10 points of each 5.5m interval.

  // The following formular ensures seg number to always be N *
  // desire_seg_num.
  explicit MapSampler(
      const planner::PlannerSemanticMapManager& semantic_map_mgr,
      const TrafficLightManager::TLStateHashMap& tl_states, double sample_ds,
      int desire_seg_num, SampleType sample_type)
      : map_mgr_(&semantic_map_mgr),
        tl_states_(&tl_states),
        sample_ds_(sample_ds),
        desire_seg_num_(desire_seg_num),
        sample_type_(sample_type) {}
  // Lane center.
  std::vector<const mapping::LaneInfo*> GetLaneCentersWithRadius(
      const Vec2d& pos, double radius);
  std::vector<const SampledPolyline*> GetNearestLaneCenterPolylinesInBox2d(
      const Box2d& region_box, int max_num);
  std::vector<const SampledPolyline*>
  GetNearestLaneCenterPolylinesInBox2dWithLanes(
      const Box2d& region_box, int max_num,
      const std::vector<const mapping::LaneInfo*>& lanes,
      bool compute_boundary_distance);
  // Solid lane boundary.
  std::vector<const mapping::LaneBoundaryInfo*>
  GetSolidLaneBoundariesWithRadius(const Vec2d& pos, double radius);
  std::vector<const SampledPolyline*>
  GetNearestSolidLaneBoundaryPolylinesInBox2d(const Box2d& region_box,
                                              int max_num);
  std::vector<const SampledPolyline*>
  GetNearestLaneBoundaryPolylinesInBox2dWithLaneBoundaries(
      const Box2d& region_box, int max_num,
      const std::vector<const mapping::LaneBoundaryInfo*>& lane_boundaries);
  // Crosswalk.
  std::vector<const mapping::CrosswalkInfo*> GetCrosswalksWithRadius(
      const Vec2d& pos, double radius);
  std::vector<const SampledPolyline*> GetNearestCrosswalkPolylinesInBox2d(
      const Box2d& region_box, int max_num);
  std::vector<const SampledPolyline*>
  GetNearestCrosswalkPolylinesInBox2dWithCrosswalks(
      const Box2d& region_box, int max_num,
      const std::vector<const mapping::CrosswalkInfo*>& cws_in_radius);
  // Broken lane boundary.
  std::vector<const mapping::LaneBoundaryInfo*>
  GetBrokenLaneBoundariesWithRadius(const Vec2d& pos, double radius);
  std::vector<const SampledPolyline*>
  GetNearestBrokenLaneBoundaryPolylinesInBox2d(const Box2d& region_box,
                                               int max_num);

 private:
  mutable absl::Mutex mutex_;
  absl::flat_hash_map<mapping::ElementId, std::unique_ptr<SampledPolyline>>
      polylines_ ABSL_GUARDED_BY(mutex_);
  const planner::PlannerSemanticMapManager* map_mgr_;
  const TrafficLightManager::TLStateHashMap* tl_states_;
  double sample_ds_;
  int desire_seg_num_;  // number of segs in sampledPolyline must be
                        // N*unit_lane_seg_num_;
  SampleType sample_type_;
};
}  // namespace prediction
}  // namespace qcraft

#endif  // ONBOARD_PREDICTION_FEATURE_EXTRACTOR_MAP_SAMPLER_H_
