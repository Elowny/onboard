#include "onboard/prediction/feature_extractor/map_sampler.h"

#include <algorithm>
#include <cmath>
#include <utility>

#include "onboard/global/buffered_logger.h"
#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/maps/lane_speed_limit.h"
#include "onboard/maps/level_id.h"
#include "onboard/maps/maps_common.h"
#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/frenet_frame_util.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/prediction/prediction_util.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/utils/map_util.h"

namespace qcraft {
namespace prediction {
namespace {
using mapping::CrosswalkInfo;
using mapping::LaneBoundaryInfo;
using mapping::LaneBoundaryProto;
using mapping::LaneInfo;
using mapping::LaneProto;
using SampleType = MapSampler::SampleType;
constexpr double kDsEpsilon = 0.1;

// Map lane proto type (for lane centers) to customized sample point type.
SampledPointType LaneTypeToSamplePointType(LaneProto::Type lane_proto_type) {
  switch (lane_proto_type) {
    case LaneProto::GENERAL:
    case LaneProto::ENTRANCE:
    case LaneProto::EXIT:
    case LaneProto::VARIABLE_DIRECTION:
    case LaneProto::TIDAL:
      return SampledPointType::kCenterlineGeneral;
    case LaneProto::BICYCLE_ONLY:
      return SampledPointType::kCenterlineBicycleOnly;
    case LaneProto::BUS_ONLY:
      return SampledPointType::kCenterlineBusOnly;
    case LaneProto::VIRTUAL:
      return SampledPointType::kCenterlineVirtual;
    case LaneProto::EMERGENCY:
      return SampledPointType::kCenterlineEmergency;
    case LaneProto::RAMP:
    case LaneProto::RAMP_ENTRANCE:
    case LaneProto::RAMP_EXIT:
    case LaneProto::RAMP_JCT:
      return SampledPointType::kCenterlineRamp;
    case LaneProto::WALKING_STREET:
      return SampledPointType::kCenterlineWalking;
    case LaneProto::PARKING:
      return SampledPointType::kCenterlineParking;
    case LaneProto::MIXED_WITH_CYCLIST:
      return SampledPointType::kCenterlineMixedCyclist;
  }
}

// Map lane boundary type (for lane boundaries) to customized sample point type.
SampledPointType LaneBoundaryTypeToSamplePointType(
    LaneBoundaryProto::Type type) {
  switch (type) {
    case LaneBoundaryProto::SOLID_DOUBLE_YELLOW:
    case LaneBoundaryProto::SOLID_DOUBLE_WHITE:
      return SampledPointType::kSolidDouble;
    case LaneBoundaryProto::SOLID_YELLOW:
    case LaneBoundaryProto::SOLID_WHITE:
      return SampledPointType::kSolid;
    case LaneBoundaryProto::CURB:
      return SampledPointType::kCurb;
    case LaneBoundaryProto::BROKEN_LEFT_DOUBLE_WHITE:
      return SampledPointType::kBrokenLeftDouble;
    case LaneBoundaryProto::BROKEN_RIGHT_DOUBLE_WHITE:
      return SampledPointType::kBrokenRightDouble;
    case LaneBoundaryProto::BROKEN_YELLOW:
    case LaneBoundaryProto::BROKEN_DOUBLE_YELLOW:
      return SampledPointType::kBrokenYellow;
    case LaneBoundaryProto::BROKEN_WHITE:
    case LaneBoundaryProto::BROKEN_DOUBLE_WHITE:
      return SampledPointType::kBrokenWhite;
    case LaneBoundaryProto::UNKNOWN_TYPE:
    case LaneBoundaryProto::VIRTUAL:
      return SampledPointType::kNone;
  }
}

inline std::vector<LaneBoundaryProto::Type> GetSolidLaneBoundaryTypes() {
  std::vector<LaneBoundaryProto::Type> solid_lane_boundary_types(
      {LaneBoundaryProto::SOLID_DOUBLE_YELLOW, LaneBoundaryProto::SOLID_YELLOW,
       LaneBoundaryProto::SOLID_WHITE, LaneBoundaryProto::CURB,
       LaneBoundaryProto::BROKEN_LEFT_DOUBLE_WHITE,
       LaneBoundaryProto::BROKEN_RIGHT_DOUBLE_WHITE});
  return solid_lane_boundary_types;
}

inline std::vector<LaneBoundaryProto::Type> GetBrokenLaneBoundaryTypes() {
  std::vector<LaneBoundaryProto::Type> broken_lane_boundary_types(
      {LaneBoundaryProto::BROKEN_YELLOW, LaneBoundaryProto::BROKEN_WHITE,
       LaneBoundaryProto::BROKEN_DOUBLE_YELLOW,
       LaneBoundaryProto::BROKEN_DOUBLE_WHITE});
  return broken_lane_boundary_types;
}

SampledPolyline SampleLaneToPolyline(
    const planner::PlannerSemanticMapManager& semantic_map_mgr,
    mapping::LevelId /*level_id*/, const LaneInfo& lane_info,
    const TrafficLightManager::TLStateHashMap& light_map, double sample_ds,
    int desire_seg_num, MapSampler::SampleType sample_type,
    bool compute_boundary_distance) {
  // Lane type.
  auto lane_type = lane_info.Type();
  // virtual type should be determined by this value.
  if (lane_info.IsVirtual()) {
    lane_type = LaneProto::VIRTUAL;
  }
  const auto sample_point_type = LaneTypeToSamplePointType(lane_type);
  // Nearest left and right boundary type.
  auto left_boundary_type = SampledPointType::kNone;
  auto right_boundary_type = SampledPointType::kNone;
  if (!lane_info.lane_neighbors_on_left.empty()) {
    left_boundary_type = LaneBoundaryTypeToSamplePointType(
        lane_info.lane_neighbors_on_left[0].lane_boundary_type);
  }

  if (!lane_info.lane_neighbors_on_right.empty()) {
    right_boundary_type = LaneBoundaryTypeToSamplePointType(
        lane_info.lane_neighbors_on_right[0].lane_boundary_type);
  }
  // Get lane lights.
  std::vector<int> lane_lights(4, 0);
  auto& num_green = lane_lights[0];
  auto& num_yellow = lane_lights[1];
  auto& num_red = lane_lights[2];
  auto& num_unknown = lane_lights[3];
  for (const auto& id : lane_info.traffic_light_ids()) {
    const auto* light_stat = FindOrNull(light_map, id);
    if (light_stat == nullptr) {
      ++num_unknown;
      continue;
    }
    switch (light_stat->color()) {
      case TL_GREEN:
        ++num_green;
        break;
      case TL_YELLOW:
        ++num_yellow;
        break;
      case TL_RED:
        ++num_red;
        break;
      case TL_UNKNOWN:
        ++num_unknown;
        break;
    }
  }

  // Fill in polyline.
  const auto& lane_length = lane_info.length();
  std::vector<SampledPolylineSeg> segs;
  int num_segs = static_cast<int>(std::ceil(lane_length / sample_ds));
  if (sample_type == SampleType::ADAPTIVE) {
    // The following formular ensures seg number to always be N *
    // desire_seg_num.
    num_segs = static_cast<int>(
        desire_seg_num * std::ceil(lane_length / (desire_seg_num * sample_ds)));
    QCHECK_GT(num_segs, 0);
    sample_ds = lane_length / num_segs;
  }
  segs.reserve(num_segs);

  // For speed convertion
  constexpr double kMStoKPH = 3.6;

  for (int i = 0; i < num_segs; ++i) {
    const auto cur =
        lane_info.SLToSmoothXY(FrenetCoordinate{.s = i * sample_ds, .l = 0.0});
    const auto cur_ds = lane_info.SLToSmoothXY(
        FrenetCoordinate{.s = i * sample_ds + kDsEpsilon, .l = 0.0});
    const auto next = lane_info.SLToSmoothXY(
        FrenetCoordinate{.s = (i + 1) * sample_ds, .l = 0.0});
    std::optional<double> cur_distance_to_bound_option = std::nullopt;
    // This operation seems quite time consuming, we use a boolean flag to
    // control if we compute it.
    if (compute_boundary_distance) {
      cur_distance_to_bound_option =
          semantic_map_mgr.GetLeftLaneWidth(cur, lane_info.id);
    }
    // Calc dynamic speed limit
    const double fraction = i * sample_ds / lane_length;
    const double speed_limit =
        SpeedLimitAtFractionOrDefault(lane_info.speed_limits, fraction);
    int speed_limit_kph = static_cast<int>(speed_limit * kMStoKPH);

    SampledPolylineSeg seg{
        .seg = Segment2d(cur, next),
        .unit_tangent = (cur_ds - cur).normalized(),
        .type = sample_point_type,
        .lights = lane_lights,
        .speed_limit_kph = speed_limit_kph,
        .distance_to_boundary = cur_distance_to_bound_option,
        .boundary_types = {left_boundary_type, right_boundary_type},
    };

    segs.push_back(std::move(seg));
  }
  return SampledPolyline{
      .id = lane_info.id,
      .segs = std::move(segs),
  };
}

SampledPolyline SampleLaneBoundaryToPolyline(
    const LaneBoundaryInfo& lb, double sample_ds, int desire_seg_num,
    MapSampler::SampleType sample_type) {
  const auto sample_point_type = LaneBoundaryTypeToSamplePointType(lb.type);

  // Fill in polyline.
  const double length = lb.cumulative_lengths.back();
  std::vector<SampledPolylineSeg> segs;
  int num_segs = static_cast<int>(std::ceil(length / sample_ds));
  if (sample_type == SampleType::ADAPTIVE) {
    // The following formular ensures seg number to always be N *
    // desire_seg_num.
    num_segs = static_cast<int>(
        desire_seg_num * std::ceil(length / (desire_seg_num * sample_ds)));
    QCHECK_GT(num_segs, 0);
    sample_ds = length / num_segs;
  }
  segs.reserve(num_segs);

  for (int i = 0; i < num_segs; ++i) {
    const auto cur =
        lb.SLToSmoothXY(FrenetCoordinate{.s = i * sample_ds, .l = 0.0});
    const auto cur_ds = lb.SLToSmoothXY(
        FrenetCoordinate{.s = i * sample_ds + kDsEpsilon, .l = 0.0});
    const auto next =
        lb.SLToSmoothXY(FrenetCoordinate{.s = (i + 1) * sample_ds, .l = 0.0});
    SampledPolylineSeg seg{
        .seg = Segment2d(cur, next),
        .unit_tangent = (cur_ds - cur).normalized(),
        .type = sample_point_type,
        .lights = {0, 0, 0, 0},
        .speed_limit_kph = 0,
    };
    segs.push_back(std::move(seg));
  }
  return SampledPolyline{
      .id = lb.id,
      .segs = std::move(segs),
  };
}

SampledPolyline SampleCrosswalkToPolyline(const CrosswalkInfo& cw,
                                          double sample_ds, int desire_seg_num,
                                          MapSampler::SampleType sample_type) {
  const auto sample_point_type = SampledPointType::kCrosswalk;

  // Build frenet anchor points.
  auto points = cw.polygon_smooth.points();
  // Close the polygon.
  points.push_back(cw.polygon_smooth.points().front());
  std::vector<double> accumulated_dists;
  accumulated_dists.reserve(points.size());
  QCHECK(points.size() > 1);
  double len = 0.0;
  for (int i = 0; i < points.size(); ++i) {
    if (i > 0) {
      len += points[i - 1].DistanceTo(points[i]);
    }
    accumulated_dists.push_back(len);
  }

  // Fill in polyline.
  const double length = accumulated_dists.back();
  std::vector<SampledPolylineSeg> segs;
  int num_segs = static_cast<int>(std::ceil(length / sample_ds));
  if (sample_type == SampleType::ADAPTIVE) {
    // The following formular ensures seg number to always be N *
    // desire_seg_num.
    num_segs = static_cast<int>(
        desire_seg_num * std::ceil(length / (desire_seg_num * sample_ds)));
    QCHECK_GT(num_segs, 0);
    sample_ds = length / num_segs;
  }
  segs.reserve(num_segs);

  for (int i = 0; i < num_segs; ++i) {
    const auto cur = frenet_frame_util::SLToXY(
        FrenetCoordinate{.s = i * sample_ds, .l = 0.0}, points,
        accumulated_dists);
    const auto cur_ds = frenet_frame_util::SLToXY(
        FrenetCoordinate{.s = i * sample_ds + kDsEpsilon, .l = 0.0}, points,
        accumulated_dists);
    const auto next = frenet_frame_util::SLToXY(
        FrenetCoordinate{.s = (i + 1) * sample_ds, .l = 0.0}, points,
        accumulated_dists);
    SampledPolylineSeg seg{
        .seg = Segment2d(cur, next),
        .unit_tangent = (cur_ds - cur).normalized(),
        .type = sample_point_type,
        .lights = {0, 0, 0, 0},
        .speed_limit_kph = 0,
    };
    segs.push_back(std::move(seg));
  }
  return SampledPolyline{
      .id = cw.id,
      .segs = std::move(segs),
  };
}
}  // namespace

std::vector<const mapping::LaneInfo*> MapSampler::GetLaneCentersWithRadius(
    const Vec2d& pos, double radius) {
  FUNC_QTRACE();
  const auto level_id = map_mgr_->GetLevel();
  auto lane_centers = map_mgr_->GetLanesInfoAtLevel(level_id, pos, radius);
  std::sort(lane_centers.begin(), lane_centers.end(),
            [](const auto& lc1, const auto& lc2) { return lc1->id < lc2->id; });

  return lane_centers;
}

std::vector<const SampledPolyline*>
MapSampler::GetNearestLaneCenterPolylinesInBox2d(const Box2d& region_box,
                                                 int max_num) {
  const auto lanes_in_radius = GetLaneCentersWithRadius(
      region_box.center(), region_box.diagonal() * 0.5);
  return GetNearestLaneCenterPolylinesInBox2dWithLanes(
      region_box, max_num, lanes_in_radius, /*compute_boundary_distance=*/true);
}

std::vector<const SampledPolyline*>
MapSampler::GetNearestLaneCenterPolylinesInBox2dWithLanes(
    const Box2d& region_box, int max_num,
    const std::vector<const mapping::LaneInfo*>& lanes,
    bool compute_boundary_distance) {
  SCOPED_QTRACE_ARG2(
      "MapSampler::GetNearestLaneCenterPolylinesInBox2dWithLanes",
      "region box radius", region_box.radius(), "lane center num",
      lanes.size());
  const auto level_id = map_mgr_->GetLevel();
  const auto filtered_lanes = GetElementsInBox2d(lanes, region_box, max_num);
  std::vector<const SampledPolyline*> polylines_in_box;
  polylines_in_box.reserve(filtered_lanes.size());
  absl::WriterMutexLock lock(&mutex_);
  for (const auto& lane_ptr : filtered_lanes) {
    const auto res = polylines_.try_emplace(lane_ptr->id, nullptr);
    if (res.second) {
      res.first->second =
          std::make_unique<SampledPolyline>(SampleLaneToPolyline(
              *map_mgr_, level_id, *lane_ptr, *tl_states_, sample_ds_,
              desire_seg_num_, sample_type_, compute_boundary_distance));
    }
    polylines_in_box.push_back(res.first->second.get());
  }
  return polylines_in_box;
}

std::vector<const mapping::LaneBoundaryInfo*>
MapSampler::GetSolidLaneBoundariesWithRadius(const Vec2d& pos, double radius) {
  FUNC_QTRACE();
  const auto level_id = map_mgr_->GetLevel();
  const auto solid_lane_boundary_types = GetSolidLaneBoundaryTypes();
  auto lane_boundaries = map_mgr_->GetSpecifiedTypesLaneBoundariesInfoAtLevel(
      level_id, pos, radius, solid_lane_boundary_types);
  std::sort(lane_boundaries.begin(), lane_boundaries.end(),
            [](const auto& lb1, const auto& lb2) { return lb1->id < lb2->id; });
  return lane_boundaries;
}

std::vector<const SampledPolyline*>
MapSampler::GetNearestSolidLaneBoundaryPolylinesInBox2d(const Box2d& region_box,
                                                        int max_num) {
  const auto lbs_in_radius = GetSolidLaneBoundariesWithRadius(
      region_box.center(), region_box.diagonal() * 0.5);
  return GetNearestLaneBoundaryPolylinesInBox2dWithLaneBoundaries(
      region_box, max_num, lbs_in_radius);
}

std::vector<const SampledPolyline*>
MapSampler::GetNearestLaneBoundaryPolylinesInBox2dWithLaneBoundaries(
    const Box2d& region_box, int max_num,
    const std::vector<const mapping::LaneBoundaryInfo*>& lane_boundaries) {
  SCOPED_QTRACE_ARG2(
      "MapSampler::GetNearestLaneBoundaryPolylinesInBox2dWithLaneBoundaries",
      "region box radius", region_box.radius(), "lane bound num",
      lane_boundaries.size());
  const auto filtered_lane_boundaries =
      GetElementsInBox2d(lane_boundaries, region_box, max_num);
  std::vector<const SampledPolyline*> polylines_in_box;
  polylines_in_box.reserve(filtered_lane_boundaries.size());
  absl::WriterMutexLock lock(&mutex_);
  for (const auto& lane_boundary : filtered_lane_boundaries) {
    if (!polylines_.contains(lane_boundary->id)) {
      polylines_[lane_boundary->id] =
          std::make_unique<SampledPolyline>(SampleLaneBoundaryToPolyline(
              *lane_boundary, sample_ds_, desire_seg_num_, sample_type_));
    }
    polylines_in_box.push_back(polylines_.at(lane_boundary->id).get());
  }
  return polylines_in_box;
}

std::vector<const mapping::CrosswalkInfo*> MapSampler::GetCrosswalksWithRadius(
    const Vec2d& pos, double radius) {
  FUNC_QTRACE();
  const auto level_id = map_mgr_->GetLevel();
  auto crosswalks = map_mgr_->GetCrosswalksInfoAtLevel(level_id, pos, radius);
  std::sort(crosswalks.begin(), crosswalks.end(),
            [](const auto& cw1, const auto& cw2) { return cw1->id < cw2->id; });
  return crosswalks;
}

std::vector<const SampledPolyline*>
MapSampler::GetNearestCrosswalkPolylinesInBox2dWithCrosswalks(
    const Box2d& region_box, int max_num,
    const std::vector<const mapping::CrosswalkInfo*>& cws_in_radius) {
  SCOPED_QTRACE_ARG2(
      "MapSampler::GetNearestCrosswalkPolylinesInBox2dWithCrosswalks",
      "region box radius", region_box.radius(), "cws in radius",
      cws_in_radius.size());
  const auto cws =
      GetPolygonElementsInBox2d(cws_in_radius, region_box, max_num);
  std::vector<const SampledPolyline*> polylines_in_box;
  polylines_in_box.reserve(cws.size());
  absl::WriterMutexLock lock(&mutex_);
  for (const auto& cw : cws) {
    if (!polylines_.contains(cw->id)) {
      polylines_[cw->id] =
          std::make_unique<SampledPolyline>(SampleCrosswalkToPolyline(
              *cw, sample_ds_, desire_seg_num_, sample_type_));
    }
    polylines_in_box.push_back(polylines_.at(cw->id).get());
  }
  return polylines_in_box;
}

std::vector<const SampledPolyline*>
MapSampler::GetNearestCrosswalkPolylinesInBox2d(const Box2d& region_box,
                                                int max_num) {
  const auto cws_in_radius =
      GetCrosswalksWithRadius(region_box.center(), region_box.diagonal() * 0.5);
  return GetNearestCrosswalkPolylinesInBox2dWithCrosswalks(region_box, max_num,
                                                           cws_in_radius);
}

std::vector<const mapping::LaneBoundaryInfo*>
MapSampler::GetBrokenLaneBoundariesWithRadius(const Vec2d& pos, double radius) {
  FUNC_QTRACE();
  const auto level_id = map_mgr_->GetLevel();
  const auto broken_lane_boundary_types = GetBrokenLaneBoundaryTypes();
  auto lane_boundaries = map_mgr_->GetSpecifiedTypesLaneBoundariesInfoAtLevel(
      level_id, pos, radius, broken_lane_boundary_types);
  std::sort(lane_boundaries.begin(), lane_boundaries.end(),
            [](const auto& lb1, const auto& lb2) { return lb1->id < lb2->id; });
  return lane_boundaries;
}

std::vector<const SampledPolyline*>
MapSampler::GetNearestBrokenLaneBoundaryPolylinesInBox2d(
    const Box2d& region_box, int max_num) {
  const auto lbs_in_radius = GetBrokenLaneBoundariesWithRadius(
      region_box.center(), region_box.diagonal() * 0.5);
  return GetNearestLaneBoundaryPolylinesInBox2dWithLaneBoundaries(
      region_box, max_num, lbs_in_radius);
}

}  // namespace prediction
}  // namespace qcraft
