#include "onboard/planner/planner_semantic_map_manager.h"

// IWYU pragma: no_include <google/protobuf/repeated_ptr_field.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <ostream>
#include <utility>

#include "absl/container/flat_hash_map.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/strings/str_join.h"
#include "glog/logging.h"

#include "onboard/async/parallel_for.h"
#include "onboard/base/macros.h"
#include "onboard/container/strong_int.h"
#include "onboard/global/trace.h"
#include "onboard/lite/logging.h"
#include "onboard/maps/maps_helper.h"
#include "onboard/maps/v2/semantic_map_definition.h"
#include "onboard/maps/v2/semantic_map_object.h"
#include "onboard/math/geometry/segment2d.h"
#include "onboard/math/piecewise_const_function.h"
#include "onboard/math/util.h"
#include "onboard/planner/planner_flags.h"
#include "onboard/planner/router/route_speed_limit_util.h"
#include "onboard/utils/map_util.h"

namespace qcraft::planner {

namespace {

const PiecewiseConstFunction<int, double> kModSpeedLimitFraction(
    {0, 40, 60, 70, 80, 320}, {0.3, 0.2, 0.1, 0.05, 0.0});

std::optional<double> GetDistanceBetweenLinesWithTangents(
    const Vec2d& point, PlannerSemanticMapManager::Side side,
    const Segment2d& boundary_seg, const Segment2d& norm_seg,
    double lane_heading) {
  Vec2d intersection;
  if (!norm_seg.GetIntersect(boundary_seg, &intersection)) {
    return std::nullopt;
  }
  // Align the direction of boundary with lane.
  Segment2d aligned_boundary_seg = boundary_seg;
  if (std::fabs(NormalizeAngle(aligned_boundary_seg.heading() - lane_heading)) >
      M_PI_2) {
    aligned_boundary_seg.Reverse();
  }
  double prod = aligned_boundary_seg.ProductOntoUnit(point);
  if (side == PlannerSemanticMapManager::Side::kLEFT) prod *= -1.0;
  return std::copysign(aligned_boundary_seg.DistanceTo(point), prod);
}

std::vector<mapping::v2::Segment> FindMarginalBoundarySegments(
    const std::shared_ptr<mapping::v2::SemanticMapSpatialIndex>& smsi,
    const Vec2d& global_coord, double radius) {
  if (UNLIKELY(!smsi)) {
    return {};
  }
  const auto& semantic_map_v2 = smsi->semantic_manager();
  if (UNLIKELY(!semantic_map_v2)) {
    return {};
  }

  const auto segments_in_radius = smsi->FindLaneBoundarySegmentsInRadius(
      global_coord.x(), global_coord.y(), radius);

  std::vector<mapping::v2::Segment> results;
  results.reserve(segments_in_radius.size());
  for (const auto& segment : segments_in_radius) {
    const auto lane_boundary =
        semantic_map_v2->FindLaneBoundary(segment.element_id);
    if (lane_boundary == nullptr) continue;
    if (lane_boundary->ComputeLaneBoundaryType(segment.segment_id) ==
        mapping::LaneBoundaryProto::VIRTUAL)
      continue;
    if (lane_boundary->proto().has_is_drivable_boundary()) {
      if (lane_boundary->proto().is_drivable_boundary()) {
        results.push_back(segment);
      }
    } else if (lane_boundary->proto().lanes_on_left().empty() ||
               lane_boundary->proto().lanes_on_right().empty()) {
      results.push_back(segment);
    }
  }
  return results;
}

absl::StatusOr<mapping::v2::Segment> FindNearestMarginalBoundarySegment(
    const std::shared_ptr<mapping::v2::SemanticMapSpatialIndex>& smsi,
    const Vec2d& global_coord) {
  // BANDAID(zuowei): Magic number for navinfo map.
  constexpr double kSearchRadius = 15.0;  // m.
  const auto segments_in_radius =
      FindMarginalBoundarySegments(smsi, global_coord, kSearchRadius);
  if (segments_in_radius.empty()) {
    return absl::NotFoundError(
        absl::StrCat("Can not find nearest curb segment in navi info map, "
                     "global vec2d: ",
                     global_coord.DebugStringFullPrecision()));
  }
  const auto& smm_v2 = smsi->semantic_manager();
  if (!smm_v2) {
    return absl::NotFoundError("The input of semantic map is empty.");
  }
  int min_index = 0;
  double min_dist_sqr = std::numeric_limits<double>::infinity();
  for (int i = 0; i < segments_in_radius.size(); ++i) {
    const auto& cur_segment = segments_in_radius[i];
    const auto lane_boundary = smm_v2->FindLaneBoundary(cur_segment.element_id);
    if (lane_boundary == nullptr) continue;
    const Segment2d cur_seg(
        lane_boundary->segment_points()[cur_segment.segment_id.value()],
        lane_boundary->segment_points()[cur_segment.segment_id.value() + 1]);
    const double cur_dist_sqr = cur_seg.DistanceSquareTo(global_coord);
    if (cur_dist_sqr < min_dist_sqr) {
      min_index = i;
      min_dist_sqr = cur_dist_sqr;
    }
  }
  return segments_in_radius[min_index];
}

enum class QueryMode {
  kFractionSpeed = 0,
  kMinSpeed = 1,
  kMaxSpeed = 2,
  kAverageSpeed = 3,
};

double QueryLaneSpeedLimitByMode(const PlannerSemanticMapManager& psmm,
                                 const PlannerSemanticMapModification& modifier,
                                 mapping::ElementId id, double fraction,
                                 QueryMode mode) {
  const auto* lane_info_ptr = psmm.FindLaneInfoOrNull(id);
  if (lane_info_ptr == nullptr) {
    QLOG_EVERY_N_SEC(WARNING, 3)
        << "Can not find lane id: " << id
        << ", use modifier default speed limit: " << modifier.max_speed_limit;
    return modifier.max_speed_limit;
  }

  double speed_limit_kph = 0.0;
  switch (mode) {
    case QueryMode::kFractionSpeed: {
      speed_limit_kph = GetOverwrittenLaneFractionSpeedLimit(
          *psmm.semantic_map_manager(), id, fraction);
      break;
    }
    case QueryMode::kMinSpeed: {
      speed_limit_kph =
          GetOverwrittenLaneMinSpeedLimit(*psmm.semantic_map_manager(), id);
      break;
    }
    case QueryMode::kMaxSpeed: {
      speed_limit_kph =
          GetOverwrittenLaneMaxSpeedLimit(*psmm.semantic_map_manager(), id);
      break;
    }
    case QueryMode::kAverageSpeed: {
      speed_limit_kph =
          GetOverwrittenLaneAverageSpeedLimit(*psmm.semantic_map_manager(), id);
      break;
    }
  }

  const int lane_speed_limit_kph = RoundToInt(speed_limit_kph);

  const double speed_limit_crease_factor =
      FLAGS_planner_override_lane_speed_limit_proportion == 0.0
          ? (FLAGS_planner_enable_dynamic_lane_speed_limit &&
                     !psmm.IsThirdPartyMap()
                 ? kModSpeedLimitFraction(lane_speed_limit_kph)
                 : 0.0)
          : FLAGS_planner_override_lane_speed_limit_proportion;

  const double increased_lane_speed_limit =
      Kph2Mps(speed_limit_kph) * (1.0 + speed_limit_crease_factor);

  double speed_limit =
      std::min(modifier.max_speed_limit, increased_lane_speed_limit);

  if (const auto it = modifier.lane_speed_limit_map.find(id);
      it != modifier.lane_speed_limit_map.end()) {
    speed_limit = std::min(it->second, speed_limit);
  }

  return speed_limit;
}

}  // namespace

PlannerSemanticMapManager::PlannerSemanticMapManager(
    std::shared_ptr<const mapping::v2::SemanticMapMultilevelSpatialIndex> smmsi)
    : smmsi_(std::move(smmsi)) {}

PlannerSemanticMapManager::PlannerSemanticMapManager(
    std::shared_ptr<const mapping::v2::SemanticMapMultilevelSpatialIndex> smmsi,
    PlannerSemanticMapModification modifier)
    : smmsi_(std::move(smmsi)), modifier_(std::move(modifier)) {}

absl::Status PlannerSemanticMapManager::BuildSemanticMapInfo(
    ThreadPool* thread_pool) {
  SCOPED_QTRACE("PlannerSemanticMapManager::BuildSemanticMapInfo");
  // TODO(zuowei): BuildElements should not rely on coordinate converter.
  if (!coordinate_converter_.is_valid()) {
    return absl::InvalidArgumentError("CoordinateConverter is Invalid.");
  }
  const auto& semantic_map = semantic_map_manager()->semantic_map();

  // Lane info
  const auto& lanes = semantic_map.lanes;
  info_.lane_info.resize(lanes.size());
  ParallelFor(0, info_.lane_info.size(), thread_pool, [&](int i) {
    info_.lane_info[i].FromLaneObject(lanes[mapping::v2::LaneIndex(i)],
                                      coordinate_converter_);
  });

  // Lane boundary info
  const auto& lane_boundaries = semantic_map.lane_boundaries;
  info_.lane_boundary_info.resize(lane_boundaries.size());
  ParallelFor(0, info_.lane_boundary_info.size(), thread_pool, [&](int i) {
    info_.lane_boundary_info[i].FromLaneBoundaryObject(
        lane_boundaries[mapping::v2::LaneBoundaryIndex(i)],
        coordinate_converter_);
  });

  // Intersection info
  const auto& intersections = semantic_map.intersections;
  info_.intersection_info.resize(intersections.size());
  ParallelFor(0, info_.intersection_info.size(), thread_pool, [&](int i) {
    info_.intersection_info[i].FromIntersectionObject(
        intersections[mapping::v2::IntersectionIndex(i)],
        coordinate_converter_);
  });

  // Crosswalk info
  const auto& crosswalks = semantic_map.crosswalks;
  info_.crosswalk_info.resize(crosswalks.size());
  ParallelFor(0, info_.crosswalk_info.size(), thread_pool, [&](int i) {
    info_.crosswalk_info[i].FromCrosswalkObject(
        crosswalks[mapping::v2::CrosswalkIndex(i)], coordinate_converter_);
  });

  // Section info
  const auto& sections = semantic_map.sections;
  info_.section_info.resize(sections.size());
  ParallelFor(0, info_.section_info.size(), thread_pool, [&](int i) {
    info_.section_info[i].FromSectionObject(
        sections[mapping::v2::SectionIndex(i)]);
  });

  // Parking spot info
  const auto& parking_spots = semantic_map.parking_spots;
  info_.parking_spot_info.resize(parking_spots.size());
  ParallelFor(0, info_.parking_spot_info.size(), thread_pool,
              [&parking_spots, this](int i) {
                info_.parking_spot_info[i] = mapping::ParkingSpotInfo(
                    parking_spots[mapping::v2::ParkingSpotIndex(i)],
                    coordinate_converter_);
              });

  return absl::OkStatus();
}

void PlannerSemanticMapManager::UpdateSmoothInfoOfMapElements(
    ThreadPool* thread_pool) {
  SCOPED_QTRACE("PlannerSemanticMapManager::UpdateSmoothInfoOfMapElements");

  ParallelFor(0, info_.lane_info.size(), thread_pool, [this](int i) {
    info_.lane_info[i].UpdateSmoothCoordinates(coordinate_converter_);
  });
  ParallelFor(0, info_.lane_boundary_info.size(), thread_pool, [this](int i) {
    info_.lane_boundary_info[i].UpdateSmoothCoordinates(coordinate_converter_);
  });
  ParallelFor(0, info_.intersection_info.size(), thread_pool, [this](int i) {
    info_.intersection_info[i].UpdateSmoothCoordinates(coordinate_converter_);
  });
  ParallelFor(0, info_.crosswalk_info.size(), thread_pool, [this](int i) {
    info_.crosswalk_info[i].UpdateSmoothCoordinates(coordinate_converter_);
  });
  ParallelFor(0, info_.parking_spot_info.size(), thread_pool, [this](int i) {
    info_.parking_spot_info[i].UpdateSmoothCoordinates(coordinate_converter_);
  });
}

void PlannerSemanticMapManager::UpdateCoordinateConverter(
    CoordinateConverter coordinate_converter) {
  coordinate_converter_ = std::move(coordinate_converter);
}

double PlannerSemanticMapManager::QueryLaneSpeedLimitByFraction(
    mapping::ElementId id, double fraction) const {
  return QueryLaneSpeedLimitByMode(*this, modifier_, id, fraction,
                                   QueryMode::kFractionSpeed);
}

double PlannerSemanticMapManager::QueryAverageLaneSpeedLimitById(
    mapping::ElementId lane_id) const {
  return QueryLaneSpeedLimitByMode(*this, modifier_, lane_id, /*fraction=*/0.0,
                                   QueryMode::kAverageSpeed);
}

double PlannerSemanticMapManager::QueryMaxLaneSpeedLimitById(
    mapping::ElementId lane_id) const {
  return QueryLaneSpeedLimitByMode(*this, modifier_, lane_id, /*fraction=*/0.0,
                                   QueryMode::kMaxSpeed);
}

double PlannerSemanticMapManager::QueryMinLaneSpeedLimitById(
    mapping::ElementId lane_id) const {
  return QueryLaneSpeedLimitByMode(*this, modifier_, lane_id, /*fraction=*/0.0,
                                   QueryMode::kMinSpeed);
}

mapping::LevelId PlannerSemanticMapManager::GetLevel() const {
  return coordinate_converter_.GetLevel();
}

std::vector<const mapping::LaneInfo*>
PlannerSemanticMapManager::GetLanesInfoAtLevel(mapping::LevelId level_id,
                                               const Vec2d& smooth_coord,
                                               double radius) const {
  const auto& level_smsi = smmsi_->level(level_id);
  const auto global_coord = coordinate_converter_.SmoothToGlobal(smooth_coord);
  const auto lane_vec =
      level_smsi->FindLanesInRadius(global_coord.x(), global_coord.y(), radius);
  std::vector<const mapping::LaneInfo*> lane_infos;
  lane_infos.reserve(lane_vec.size());

  for (const auto& lane : lane_vec) {
    lane_infos.push_back(FindLaneInfoOrNull(lane->id()));
  }
  return lane_infos;
}

const mapping::LaneInfo*
PlannerSemanticMapManager::GetNearestLaneInfoWithHeadingAtLevel(
    mapping::LevelId level_id, const Vec2d& smooth_coord, double theta,
    double radius, double max_heading_diff) const {
  const auto& level_smsi = smmsi_->level(level_id);
  const auto global_coord = coordinate_converter_.SmoothToGlobal(smooth_coord);
  const auto found_segments = level_smsi->FindLaneSegmentsInRadius(
      global_coord.x(), global_coord.y(), radius);
  if (UNLIKELY(found_segments.empty())) return nullptr;

  double min_dist = std::numeric_limits<double>::infinity();
  mapping::ElementId lane_id = mapping::kInvalidElementId;
  for (const auto& segment : found_segments) {
    const auto* lane_info = FindLaneInfoOrNull(segment.element_id);
    if (LIKELY(lane_info != nullptr)) {
      DCHECK_GE(segment.segment_id.value(), 0);
      DCHECK_LT(segment.segment_id.value(),
                lane_info->points_smooth.size() - 1);
      const Segment2d smooth_seg(
          lane_info->points_smooth[segment.segment_id.value()],
          lane_info->points_smooth[segment.segment_id.value() + 1]);
      const double heading_diff = NormalizeAngle(theta - smooth_seg.heading());
      if (std::fabs(heading_diff) > max_heading_diff) {
        continue;
      }
      const double dist = smooth_seg.DistanceTo(smooth_coord);
      if (dist < min_dist) {
        min_dist = dist;
        lane_id = segment.element_id;
      }
    }
  }
  return FindLaneInfoOrNull(lane_id);
}

std::vector<const mapping::LaneInfo*>
PlannerSemanticMapManager::GetLanesInfoWithHeadingAtLevel(
    mapping::LevelId level_id, const Vec2d& smooth_coord, double theta,
    double radius, double max_heading_diff) const {
  const auto& level_smsi = smmsi_->level(level_id);
  const auto global_coord = coordinate_converter_.SmoothToGlobal(smooth_coord);
  const auto found_segments = level_smsi->FindLaneSegmentsInRadius(
      global_coord.x(), global_coord.y(), radius);
  if (UNLIKELY(found_segments.empty())) return {};

  std::vector<const mapping::LaneInfo*> lane_infos;
  lane_infos.reserve(found_segments.size());

  for (const auto& lane_segment : found_segments) {
    const auto* lane_info = FindLaneInfoOrNull(lane_segment.element_id);
    if (LIKELY(lane_info != nullptr)) {
      DCHECK_GE(lane_segment.segment_id.value(), 0);
      DCHECK_LT(lane_segment.segment_id.value(),
                lane_info->points_smooth.size() - 1);
      const Segment2d smooth_seg(
          lane_info->points_smooth[lane_segment.segment_id.value()],
          lane_info->points_smooth[lane_segment.segment_id.value() + 1]);
      const double heading_diff = NormalizeAngle(theta - smooth_seg.heading());
      if (std::fabs(heading_diff) < max_heading_diff) {
        lane_infos.push_back(lane_info);
      }
    }
  }

  std::sort(lane_infos.begin(), lane_infos.end(),
            [](const auto* this_lane_info, const auto* other_lane_info) {
              return this_lane_info->proto->id() < other_lane_info->proto->id();
            });
  int count = 0;
  for (int i = 0; i < lane_infos.size(); ++i) {
    if (i == 0 ||
        lane_infos[i - 1]->proto->id() != lane_infos[i]->proto->id()) {
      lane_infos[count++] = lane_infos[i];
    }
  }
  lane_infos.resize(count);
  return lane_infos;
}

bool PlannerSemanticMapManager::GetLaneProjection(
    const Vec2d& smooth_coord, mapping::ElementId lane_id,
    double* const fraction, Vec2d* const point, double* const min_dist,
    Segment2d* const segment) const {
  const auto* lane_info_ptr = FindLaneInfoOrNull(lane_id);
  if (UNLIKELY(lane_info_ptr == nullptr)) return false;

  const auto& points_smooth = lane_info_ptr->points_smooth;
  if (UNLIKELY(points_smooth.size() < 1)) return false;

  std::vector<Segment2d> segments = mapping::Vec2dToSegments(points_smooth);

  double min_distance = std::numeric_limits<double>::infinity();
  Vec2d proj_point;
  Vec2d* const point_ptr = point == nullptr ? nullptr : &proj_point;
  int min_idx = -1;
  for (int i = 0; i < segments.size(); ++i) {
    const double cur_dist = segments[i].DistanceTo(smooth_coord, point_ptr);
    if (cur_dist < min_distance) {
      min_idx = i;
      min_distance = cur_dist;
      if (point != nullptr) {
        *point = proj_point;
      }
    }
  }
  if (segment != nullptr) {
    *segment = segments[min_idx];
  }
  if (min_dist != nullptr) {
    *min_dist = min_distance;
  }

  if (fraction != nullptr) {
    double accum_s = 0.0;
    for (int i = 0; i < min_idx; ++i) {
      accum_s += segments[i].length();
    }
    accum_s += segments[min_idx].ProjectOntoUnit(smooth_coord);
    *fraction = std::clamp(accum_s / lane_info_ptr->length(), 0.0, 1.0);
  }
  return true;
}

bool PlannerSemanticMapManager::GetLaneBoundaryProjection(
    const Vec2d& smooth_coord, mapping::ElementId lane_boundary_id,
    double* const fraction, Vec2d* const point, double* const min_dist,
    Segment2d* const segment) const {
  const auto* lane_boundary_info_ptr =
      FindLaneBoundaryByIdOrNull(lane_boundary_id);
  if (UNLIKELY(lane_boundary_info_ptr == nullptr)) return false;

  const auto& points_smooth = lane_boundary_info_ptr->points_smooth;
  if (UNLIKELY(points_smooth.size() < 1)) return false;

  const std::vector<Segment2d> segments =
      mapping::Vec2dToSegments(points_smooth);

  double min_distance = std::numeric_limits<double>::infinity();
  Vec2d proj_point;
  Vec2d* const point_ptr = point == nullptr ? nullptr : &proj_point;
  int min_idx = -1;
  for (int i = 0; i < segments.size(); ++i) {
    const double cur_dist = segments[i].DistanceTo(smooth_coord, point_ptr);
    if (cur_dist < min_distance) {
      min_idx = i;
      min_distance = cur_dist;
      if (point != nullptr) {
        *point = proj_point;
      }
    }
  }
  if (segment != nullptr) {
    *segment = segments[min_idx];
  }
  if (min_dist != nullptr) {
    *min_dist = min_distance;
  }

  if (fraction != nullptr) {
    double accum_s = 0.0;
    for (int i = 0; i < min_idx; ++i) {
      accum_s += segments[i].length();
    }
    accum_s += segments[min_idx].ProjectOntoUnit(smooth_coord);
    *fraction =
        std::clamp(accum_s / lane_boundary_info_ptr->length(), 0.0, 1.0);
  }
  return true;
}

const mapping::LaneInfo* PlannerSemanticMapManager::GetNearestLaneInfoAtLevel(
    mapping::LevelId level_id, const Vec2d& smooth_coord) const {
  const auto& level_smsi = smmsi_->level(level_id);
  const auto global_coord = coordinate_converter_.SmoothToGlobal(smooth_coord);
  const auto lane =
      level_smsi->FindNearestLane(global_coord.x(), global_coord.y());
  return lane == nullptr ? nullptr : FindLaneInfoOrNull(lane->id());
}

std::optional<double> PlannerSemanticMapManager::ComputeLaneWidth(
    const Vec2d& smooth_coord, mapping::ElementId lane_id, Side side) const {
  const mapping::LaneProto* lane_proto = FindLaneByIdOrNull(lane_id);
  if (lane_proto == nullptr) return std::nullopt;

  Segment2d nearest_lane_segment;
  if (!GetLaneProjection(smooth_coord, lane_id,
                         /*fraction=*/nullptr,
                         /*point=*/nullptr,
                         /*min_dist=*/nullptr, &nearest_lane_segment)) {
    return std::nullopt;
  }

  constexpr double kMaxLaneWidth = 5.0;  // m.
  const Vec2d normal =
      Vec2d::FastUnitFromAngle(nearest_lane_segment.heading()).Perp();
  const Segment2d normal_seg(smooth_coord + normal * kMaxLaneWidth,
                             smooth_coord - normal * kMaxLaneWidth);

  const auto& boundary_infos = side == Side::kLEFT
                                   ? lane_proto->lane_boundaries_on_left()
                                   : lane_proto->lane_boundaries_on_right();
  std::vector<Segment2d> boundary_segments;
  for (const auto& info : boundary_infos) {
    const auto* boundary =
        FindLaneBoundaryByIdOrNull(mapping::ElementId(info.lane_boundary_id()));
    if (boundary == nullptr || boundary->points_smooth.size() < 2) continue;
    const auto segs = mapping::Vec2dToSegments(boundary->points_smooth);
    boundary_segments.insert(boundary_segments.end(), segs.begin(), segs.end());
  }

  std::optional<double> width;
  for (const auto& boundary_seg : boundary_segments) {
    if (boundary_seg.DistanceTo(smooth_coord) > kMaxLaneWidth) {
      continue;
    }
    const auto dist = GetDistanceBetweenLinesWithTangents(
        smooth_coord, side, boundary_seg, normal_seg,
        nearest_lane_segment.heading());
    if (!dist.has_value()) continue;
    if (!width.has_value() || std::fabs(*dist) < std::fabs(*width)) {
      width = *dist;
    }
  }
  return width;
}

std::optional<double> PlannerSemanticMapManager::GetLeftLaneWidth(
    const Vec2d& smooth_coord, mapping::ElementId lane_id) const {
  return ComputeLaneWidth(smooth_coord, lane_id, Side::kLEFT);
}

std::optional<double> PlannerSemanticMapManager::GetRightLaneWidth(
    const Vec2d& smooth_coord, mapping::ElementId lane_id) const {
  return ComputeLaneWidth(smooth_coord, lane_id, Side::kRIGHT);
}

std::vector<const mapping::LaneBoundaryInfo*>
PlannerSemanticMapManager::GetLaneBoundariesInfoAtLevel(
    mapping::LevelId level_id, const Vec2d& smooth_coord, double radius) const {
  const auto& level_smsi = smmsi_->level(level_id);
  const auto global_coord = coordinate_converter_.SmoothToGlobal(smooth_coord);
  const auto boundary_vec = level_smsi->FindLaneBoundariesInRadius(
      global_coord.x(), global_coord.y(), radius);

  std::vector<const mapping::LaneBoundaryInfo*> lane_boundary_infos;
  lane_boundary_infos.reserve(boundary_vec.size());
  for (const auto& lane_boundary : boundary_vec) {
    const auto* lane_boundary_ptr =
        FindLaneBoundaryByIdOrNull(lane_boundary->id());
    if (lane_boundary_ptr == nullptr) continue;
    lane_boundary_infos.push_back(lane_boundary_ptr);
  }
  return lane_boundary_infos;
}

std::vector<const mapping::LaneBoundaryInfo*>
PlannerSemanticMapManager::GetSpecifiedTypesLaneBoundariesInfoAtLevel(
    mapping::LevelId level_id, const Vec2d& smooth_coord, double radius,
    const absl::Span<const mapping::LaneBoundaryProto::Type>
        lane_boundary_types) const {
  const auto& level_smsi = smmsi_->level(level_id);
  const auto global_coord = coordinate_converter_.SmoothToGlobal(smooth_coord);
  std::vector<std::shared_ptr<const mapping::v2::LaneBoundary>> boundary_vec;
  for (const auto& type : lane_boundary_types) {
    const auto type_boundary_vec =
        level_smsi->FindLaneBoundariesInRadiusWithSpecifiedType(
            global_coord.x(), global_coord.y(), radius, type);
    boundary_vec.insert(boundary_vec.end(), type_boundary_vec.begin(),
                        type_boundary_vec.end());
  }

  std::vector<const mapping::LaneBoundaryInfo*> lane_boundary_infos;
  lane_boundary_infos.reserve(boundary_vec.size());
  for (const auto& lane_boundary : boundary_vec) {
    const auto* lane_boundary_ptr =
        FindLaneBoundaryByIdOrNull(lane_boundary->id());
    if (lane_boundary_ptr == nullptr) continue;
    lane_boundary_infos.push_back(lane_boundary_ptr);
  }
  return lane_boundary_infos;
}

std::vector<Segment2d>
PlannerSemanticMapManager::GetImpassableBoundariesAtLevel(
    mapping::LevelId level_id, const Vec2d& smooth_coord, double radius) const {
  const auto& level_smsi = smmsi_->level(level_id);
  if (!level_smsi) {
    return {};
  }

  const auto global_coord = coordinate_converter_.SmoothToGlobal(smooth_coord);
  const auto curb_vec =
      semantic_map_manager()->GetDataSource() !=
              OnlineMapProto_DataSource_NAVINFO_HDMAP
          ? level_smsi->FindCurbSegmentsInRadius(global_coord.x(),
                                                 global_coord.y(), radius)
          : FindMarginalBoundarySegments(level_smsi, global_coord, radius);

  std::vector<Segment2d> segments;
  segments.reserve(curb_vec.size());
  for (const auto& curb_seg : curb_vec) {
    const auto* lane_boundary = FindLaneBoundaryByIdOrNull(curb_seg.element_id);
    if (lane_boundary != nullptr) {
      segments.emplace_back(
          lane_boundary->points_smooth[curb_seg.segment_id.value()],
          lane_boundary->points_smooth[curb_seg.segment_id.value() + 1]);
    }
  }
  return segments;
}

std::vector<ImpassableBoundaryInfo>
PlannerSemanticMapManager::GetImpassableBoundariesInfoAtLevel(
    mapping::LevelId level_id, const Vec2d& smooth_coord, double radius) const {
  const auto& level_smsi = smmsi_->level(level_id);
  if (!level_smsi) {
    return {};
  }
  const auto global_coord = coordinate_converter_.SmoothToGlobal(smooth_coord);
  const auto curb_vec =
      semantic_map_manager()->GetDataSource() !=
              OnlineMapProto_DataSource_NAVINFO_HDMAP
          ? level_smsi->FindCurbSegmentsInRadius(global_coord.x(),
                                                 global_coord.y(), radius)
          : FindMarginalBoundarySegments(level_smsi, global_coord, radius);

  std::vector<ImpassableBoundaryInfo> boundary_infos;
  boundary_infos.reserve(curb_vec.size());
  for (const auto& curb_seg : curb_vec) {
    const auto* lane_boundary = FindLaneBoundaryByIdOrNull(curb_seg.element_id);
    if (lane_boundary != nullptr) {
      auto& boundary_info = boundary_infos.emplace_back();
      boundary_info.segment = Segment2d(
          lane_boundary->points_smooth[curb_seg.segment_id.value()],
          lane_boundary->points_smooth[curb_seg.segment_id.value() + 1]);
      boundary_info.height =
          lane_boundary->proto->has_height()
              ? std::make_optional(lane_boundary->proto->height())
              : std::nullopt;
      boundary_info.id = absl::StrFormat("CURB|%lld|%d", curb_seg.element_id,
                                         curb_seg.segment_id);
      boundary_info.lane_boundary_id = curb_seg.element_id;
    }
  }
  return boundary_infos;
}

std::vector<LaneBoundarySegmentInfo>
PlannerSemanticMapManager::GetSpecifiedTypeBoundarySegmentInfosAtLevel(
    mapping::LevelId level_id, const Vec2d& smooth_coord, double radius,
    mapping::LaneBoundaryProto::Type lane_boundary_type) const {
  const auto& level_smsi = smmsi_->level(level_id);
  if (!level_smsi) {
    return {};
  }
  const auto global_coord = coordinate_converter_.SmoothToGlobal(smooth_coord);
  const auto segments_vec =
      lane_boundary_type != mapping::LaneBoundaryProto::CURB
          ? level_smsi->FindLaneBoundarySegmentsInRadiusWithSpecifiedType(
                global_coord.x(), global_coord.y(), radius, lane_boundary_type)
          : (semantic_map_manager()->GetDataSource() !=
                     OnlineMapProto_DataSource_NAVINFO_HDMAP
                 ? level_smsi->FindCurbSegmentsInRadius(
                       global_coord.x(), global_coord.y(), radius)
                 : FindMarginalBoundarySegments(level_smsi, global_coord,
                                                radius));
  std::vector<LaneBoundarySegmentInfo> boundary_segment_infos;
  boundary_segment_infos.reserve(segments_vec.size());
  for (const auto& segment : segments_vec) {
    const auto* lane_boundary = FindLaneBoundaryByIdOrNull(segment.element_id);
    if (lane_boundary != nullptr) {
      auto& boundary_segment_info = boundary_segment_infos.emplace_back();
      boundary_segment_info.segment = Segment2d(
          lane_boundary->points_smooth[segment.segment_id.value()],
          lane_boundary->points_smooth[segment.segment_id.value() + 1]);

      boundary_segment_info.lane_boundary_id = segment.element_id;
      boundary_segment_info.segment_id = segment.segment_id;
    }
  }
  return boundary_segment_infos;
}

bool PlannerSemanticMapManager::GetNearestNamedImpassableBoundaryAtLevel(
    mapping::LevelId level_id, const Vec2d& smooth_coord, Segment2d* seg,
    std::string* id) const {
  QCHECK_NOTNULL(seg);
  QCHECK_NOTNULL(id);
  const auto& level_smsi = smmsi_->level(level_id);
  if (!level_smsi) {
    return {};
  }

  const auto global_coord = coordinate_converter_.SmoothToGlobal(smooth_coord);
  const auto segment_or =
      semantic_map_manager()->GetDataSource() !=
              OnlineMapProto_DataSource_NAVINFO_HDMAP
          ? level_smsi->FindNearestCurbSegment(global_coord.x(),
                                               global_coord.y())
          : FindNearestMarginalBoundarySegment(level_smsi, global_coord);
  if (!segment_or.ok()) return false;

  const auto* lane_boundary =
      FindLaneBoundaryByIdOrNull(segment_or->element_id);
  if (lane_boundary == nullptr) return false;
  DCHECK_GE(segment_or->segment_id.value(), 0);
  DCHECK_LT(segment_or->segment_id.value(),
            lane_boundary->points_smooth.size() - 1);
  *seg = Segment2d(
      lane_boundary->points_smooth[segment_or->segment_id.value()],
      lane_boundary->points_smooth[segment_or->segment_id.value() + 1]);
  *id = absl::StrFormat("CURB|%lld|%d", segment_or->element_id,
                        segment_or->segment_id);
  return true;
}

std::vector<const mapping::CrosswalkInfo*>
PlannerSemanticMapManager::GetCrosswalksInfoAtLevel(mapping::LevelId level_id,
                                                    const Vec2d& smooth_coord,
                                                    double radius) const {
  const auto& level_smsi = smmsi_->level(level_id);
  const auto global_coord = coordinate_converter_.SmoothToGlobal(smooth_coord);
  const auto crosswalk_vec = level_smsi->FindCrosswalksInRadius(
      global_coord.x(), global_coord.y(), radius);

  std::vector<const mapping::CrosswalkInfo*> crosswalk_infos;
  crosswalk_infos.reserve(crosswalk_vec.size());
  for (const auto& crosswalk : crosswalk_vec) {
    const auto* crosswalk_ptr = FindCrosswalkByIdOrNull(crosswalk->id());
    if (crosswalk_ptr == nullptr) continue;
    crosswalk_infos.push_back(crosswalk_ptr);
  }
  return crosswalk_infos;
}

const mapping::IntersectionInfo*
PlannerSemanticMapManager::GetNearestIntersectionInfoAtLevel(
    mapping::LevelId level_id, const Vec2d& smooth_coord) const {
  const auto& level_smsi = smmsi_->level(level_id);
  const auto global_coord = coordinate_converter_.SmoothToGlobal(smooth_coord);
  const auto intersection =
      level_smsi->FindNearestIntersection(global_coord.x(), global_coord.y());
  return intersection == nullptr
             ? nullptr
             : FindIntersectionByIdOrNull(intersection->id());
}

const mapping::BusStationStopAreaProto*
PlannerSemanticMapManager::GetNearestBusStationStopAreaAtLevel(
    mapping::LevelId level_id, const Vec2d& smooth_coord) const {
  const auto& level_smsi = smmsi_->level(level_id);
  const auto global_coord = coordinate_converter_.SmoothToGlobal(smooth_coord);
  const auto stop_area = level_smsi->FindNearestBusStationStopArea(
      global_coord.x(), global_coord.y());
  return stop_area == nullptr ? nullptr : &stop_area->proto();
}

const mapping::LaneProto* PlannerSemanticMapManager::FindLaneByIdOrNull(
    mapping::ElementId id) const {
  const auto* idx = FindOrNull(
      semantic_map_manager()->semantic_map().lane_id_to_index_map, id);
  return idx == nullptr ? nullptr : info_.lane_info[idx->value()].proto;
}

const mapping::LaneInfo* PlannerSemanticMapManager::FindLaneInfoOrNull(
    mapping::ElementId id) const {
  const auto* idx = FindOrNull(
      semantic_map_manager()->semantic_map().lane_id_to_index_map, id);
  return idx == nullptr ? nullptr : &info_.lane_info[idx->value()];
}

const std::vector<mapping::LaneInfo>& PlannerSemanticMapManager::lane_info()
    const {
  return info_.lane_info;
}

const mapping::LaneBoundaryInfo*
PlannerSemanticMapManager::FindLaneBoundaryByIdOrNull(
    mapping::ElementId id) const {
  const auto* idx = FindOrNull(
      semantic_map_manager()->semantic_map().lane_boundary_id_to_index_map, id);
  return idx == nullptr ? nullptr : &info_.lane_boundary_info[idx->value()];
}

const mapping::IntersectionInfo*
PlannerSemanticMapManager::FindIntersectionByIdOrNull(
    mapping::ElementId id) const {
  const auto* idx = FindOrNull(
      semantic_map_manager()->semantic_map().intersection_id_to_index_map, id);
  return idx == nullptr ? nullptr : &info_.intersection_info[idx->value()];
}

const mapping::IntersectionInfo& PlannerSemanticMapManager::IntersectionAt(
    int index) const {
  return info_.intersection_info[index];
}

const mapping::CrosswalkInfo*
PlannerSemanticMapManager::FindCrosswalkByIdOrNull(
    mapping::ElementId id) const {
  const auto* idx = FindOrNull(
      semantic_map_manager()->semantic_map().crosswalk_id_to_index_map, id);
  return idx == nullptr ? nullptr : &info_.crosswalk_info[idx->value()];
}

const mapping::CrosswalkInfo& PlannerSemanticMapManager::CrosswalkAt(
    int index) const {
  return info_.crosswalk_info[index];
}

const mapping::TrafficLightProto*
PlannerSemanticMapManager::FindTrafficLightByIdOrNull(
    mapping::ElementId id) const {
  const auto traffic_light = semantic_map_manager()->FindTrafficLight(id);
  return traffic_light == nullptr ? nullptr : &traffic_light->proto();
}

const mapping::SectionInfo* PlannerSemanticMapManager::FindSectionInfoOrNull(
    mapping::SectionId id) const {
  const auto* idx = FindOrNull(
      semantic_map_manager()->semantic_map().section_id_to_index_map, id);
  return idx == nullptr ? nullptr : &info_.section_info[idx->value()];
}

const std::vector<mapping::SectionInfo>&
PlannerSemanticMapManager::section_info() const {
  return info_.section_info;
}

const std::vector<mapping::ParkingSpotInfo>&
PlannerSemanticMapManager::parking_spot_info() const {
  return info_.parking_spot_info;
}

const mapping::ParkingSpotInfo*
PlannerSemanticMapManager::FindParkingSpotByIdOrNull(
    mapping::ElementId id) const {
  const auto* idx = FindOrNull(
      semantic_map_manager()->semantic_map().parking_spot_id_to_index_map, id);
  return idx == nullptr ? nullptr : &info_.parking_spot_info[idx->value()];
}

std::string PlannerSemanticMapManager::DebugString() const {
  std::stringstream ss;
  ss << std::endl;
  ss << "----------PlannerSemanticMapManager DebugInfo-----------" << std::endl;
  ss << "DataSourceType: " << OnlineMapProto::DataSource_Name(GetDataSource())
     << std::endl;
  ss << "Planner loaded section size: " << info_.section_info.size()
     << ", section ids: "
     << absl::StrJoin(info_.section_info, ", ",
                      [](std::string* out, const auto& i) {
                        out->append(std::to_string(i.proto->id()));
                      })
     << std::endl;
  ss << "Planner loaded lane size: " << info_.lane_info.size() << std::endl;

  ss << "*******Original Semantic Map Info*******" << std::endl;
  ss << "Map patch ids: "
     << absl::StrJoin(semantic_map_manager()->patch_ids(), ", ") << std::endl;
  const auto& sections_proto =
      semantic_map_manager()->semantic_map_proto().sections();
  const int origin_map_section_size =
      semantic_map_manager()->semantic_map_proto().sections().size();
  ss << "Map section size: " << origin_map_section_size << std::endl;
  ss << "Map lane size: "
     << semantic_map_manager()->semantic_map_proto().lanes_size() << std::endl;

  if (info_.section_info.size() != origin_map_section_size) {
    ss << "Map loaded by planner is different from the original." << std::endl;
    ss << "Map section ids: "
       << absl::StrJoin(sections_proto, ", ",
                        [](std::string* out, const auto& i) {
                          out->append(std::to_string(i.id()));
                        })
       << std::endl;
  }

  ss << "----------------End of Psmm DebugInfo----------------" << std::endl;
  return ss.str();
}

std::string PlannerSemanticMapManager::LaneInfoDebugString(
    mapping::ElementId lane_id) const {
  const auto* lane_info = FindLaneInfoOrNull(lane_id);
  std::stringstream ss;
  ss << std::endl;
  ss << "-------------Psmm lane info DebugInfo------------------" << std::endl;
  ss << "lane id: " << lane_id << std::endl;
  const auto& points_smooth = lane_info->points_smooth;
  const auto& points_global = lane_info->points_global;
  ss << "points smooth size: " << points_smooth.size() << std::endl;
  for (int i = 0; i < points_smooth.size(); ++i) {
    ss << "points_smooth[" << i << "] x: " << points_smooth[i].x()
       << ", y: " << points_smooth[i].y() << '\n'
       << "points_global[" << i << "] x: " << points_global[i].x()
       << ", y: " << points_global[i].y() << std::endl;
  }
  ss << "---------End of Psmm lane info DebugInfo---------" << std::endl;
  return ss.str();
}

}  // namespace qcraft::planner
