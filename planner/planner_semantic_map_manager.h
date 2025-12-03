#ifndef ONBOARD_PLANNER_PLANNER_SEMANTIC_MAP_MANAGER_H_
#define ONBOARD_PLANNER_PLANNER_SEMANTIC_MAP_MANAGER_H_

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/types/span.h"

#include "onboard/async/thread_pool.h"
#include "onboard/lite/check.h"
#include "onboard/maps/level_id.h"
#include "onboard/maps/maps_common.h"
#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/maps/v2/semantic_map_manager.h"
#include "onboard/maps/v2/semantic_map_object.h"
#include "onboard/maps/v2/semantic_map_spatial_index.h"
#include "onboard/math/coordinate_converter.h"
#include "onboard/math/geometry/segment2d.h"
#include "onboard/math/vec.h"
#include "onboard/proto/online_map.pb.h"

namespace qcraft::planner {

constexpr double kDefaultMaxSpeedLimit = 50;  // m/s

struct PlannerSemanticMapModification {
  std::map<mapping::ElementId, double> lane_speed_limit_map;
  double max_speed_limit = kDefaultMaxSpeedLimit;

  bool IsEmpty() const {
    return max_speed_limit >= kDefaultMaxSpeedLimit &&
           lane_speed_limit_map.empty();
  }
};

struct ImpassableBoundaryInfo {
  Segment2d segment;
  std::string id;
  std::optional<double> height;
  mapping::ElementId lane_boundary_id;
};

struct LaneBoundarySegmentInfo {
  Segment2d segment;
  mapping::ElementId lane_boundary_id;
  mapping::SegmentId segment_id;
};

class PlannerSemanticMapManager {
 public:
  // NOTE(lidong): We use a `shared_ptr` for `semantic_map_manager` because
  // this class may run in async mode. It has to own the underlying semantic
  // map when running in another thread with a different life cycle.
  explicit PlannerSemanticMapManager(
      std::shared_ptr<const mapping::v2::SemanticMapMultilevelSpatialIndex>
          smmsi);

  explicit PlannerSemanticMapManager(
      std::shared_ptr<const mapping::v2::SemanticMapMultilevelSpatialIndex>
          smmsi,
      PlannerSemanticMapModification modifier);

  absl::Status BuildSemanticMapInfo(ThreadPool* thread_pool);

  // NOTE(zuowei): Should called after UpdateCoordinateConverter;
  void UpdateSmoothInfoOfMapElements(ThreadPool* thread_pool);

  void UpdateCoordinateConverter(CoordinateConverter coordinate_converter);

  const CoordinateConverter& coordinate_converter() const {
    return coordinate_converter_;
  }

  // NOTE(zuowei): smm v2 = elements + spatial_index + no_coordinate_converter,
  // use semantic map manager to find elements by id, use
  // semantic_map_multilevel_spatial_index to find elements by spatial index.
  const mapping::v2::SemanticMapManager* semantic_map_manager() const {
    return smmsi_->semantic_manager().get();
  }

  const mapping::SemanticMapProto& semantic_map_proto() const {
    return smmsi_->semantic_manager()->semantic_map_proto();
  }

  const mapping::v2::SemanticMap& semantic_map() const {
    return smmsi_->semantic_manager()->semantic_map();
  }

  const std::shared_ptr<const mapping::v2::SemanticMapMultilevelSpatialIndex>&
  semantic_map_multilevel_spatial_index() const {
    return smmsi_;
  }

  const PlannerSemanticMapModification& GetSemanticMapModifier() const {
    return modifier_;
  }

  // Should called before GetXXX
  void SetSemanticMapModifier(PlannerSemanticMapModification modifier) {
    modifier_ = std::move(modifier);
  }

  // Returned value unit is m/s.
  double QueryLaneSpeedLimitByFraction(mapping::ElementId lane_id,
                                       double fraction) const;

  double QueryAverageLaneSpeedLimitById(mapping::ElementId lane_id) const;

  double QueryMaxLaneSpeedLimitById(mapping::ElementId lane_id) const;

  double QueryMinLaneSpeedLimitById(mapping::ElementId lane_id) const;

  mapping::LevelId GetLevel() const;

  OnlineMapProto::DataSource GetDataSource() const {
    return QCHECK_NOTNULL(semantic_map_manager())->GetDataSource();
  }

  bool IsOnVisionMap() const {
    return GetDataSource() == OnlineMapProto::QCRAFT_VISIONMAP;
  }

  bool IsThirdPartyMap() const {
    return GetDataSource() == OnlineMapProto::NAVINFO_HDMAP;
  }

  // Spatial search
  /************************* lane *****************************/
  // TODO(zixuan): Change function naming following the style guide.
  std::vector<const mapping::LaneInfo*> GetLanesInfoAtLevel(
      mapping::LevelId level_id, const Vec2d& smooth_coord,
      double radius) const;

  const mapping::LaneInfo* GetNearestLaneInfoWithHeadingAtLevel(
      mapping::LevelId level_id, const Vec2d& smooth_coord, double theta,
      double radius, double max_heading_diff) const;

  std::vector<const mapping::LaneInfo*> GetLanesInfoWithHeadingAtLevel(
      mapping::LevelId level_id, const Vec2d& smooth_coord, double theta,
      double radius, double max_heading_diff) const;

  bool GetLaneProjection(const Vec2d& smooth_coord, mapping::ElementId lane_id,
                         double* const fraction, Vec2d* const point,
                         double* const min_dist,
                         Segment2d* const segment) const;

  bool GetLaneBoundaryProjection(const Vec2d& smooth_coord,
                                 mapping::ElementId lane_boundary_id,
                                 double* const fraction, Vec2d* const point,
                                 double* const min_dist,
                                 Segment2d* const segment) const;

  const mapping::LaneInfo* GetNearestLaneInfoAtLevel(
      mapping::LevelId level_id, const Vec2d& smooth_coord) const;

  enum class Side {
    kLEFT = 0,
    kRIGHT = 1,
  };

  std::optional<double> ComputeLaneWidth(const Vec2d& smooth_coord,
                                         mapping::ElementId lane_id,
                                         Side side) const;

  std::optional<double> GetLeftLaneWidth(const Vec2d& smooth_coord,
                                         mapping::ElementId lane_id) const;

  std::optional<double> GetRightLaneWidth(const Vec2d& smooth_coord,
                                          mapping::ElementId lane_id) const;

  /********************* lane boundary ***********************/
  std::vector<const mapping::LaneBoundaryInfo*> GetLaneBoundariesInfoAtLevel(
      mapping::LevelId level_id, const Vec2d& smooth_coord,
      double radius) const;

  std::vector<const mapping::LaneBoundaryInfo*>
  GetSpecifiedTypesLaneBoundariesInfoAtLevel(
      mapping::LevelId level_id, const Vec2d& smooth_coord, double radius,
      const absl::Span<const mapping::LaneBoundaryProto::Type>
          lane_boundary_types) const;

  std::vector<Segment2d> GetImpassableBoundariesAtLevel(
      mapping::LevelId level_id, const Vec2d& smooth_coord,
      double radius) const;

  std::vector<ImpassableBoundaryInfo> GetImpassableBoundariesInfoAtLevel(
      mapping::LevelId level_id, const Vec2d& smooth_coord,
      double radius) const;

  std::vector<LaneBoundarySegmentInfo>
  GetSpecifiedTypeBoundarySegmentInfosAtLevel(
      mapping::LevelId level_id, const Vec2d& smooth_coord, double radius,
      mapping::LaneBoundaryProto::Type lane_boundary_type) const;

  bool GetNearestNamedImpassableBoundaryAtLevel(mapping::LevelId level_id,
                                                const Vec2d& smooth_coord,
                                                Segment2d* seg,
                                                std::string* id) const;

  /************************ crosswalk ***********************/
  std::vector<const mapping::CrosswalkInfo*> GetCrosswalksInfoAtLevel(
      mapping::LevelId level_id, const Vec2d& smooth_coord,
      double radius) const;

  /************************ intersection ********************/
  const mapping::IntersectionInfo* GetNearestIntersectionInfoAtLevel(
      mapping::LevelId level_id, const Vec2d& smooth_coord) const;

  /************************ bus station stop area ********************/
  const mapping::BusStationStopAreaProto* GetNearestBusStationStopAreaAtLevel(
      mapping::LevelId level_id, const Vec2d& smooth_coord) const;

  // Query elements.
  // TODO(zuowei): Adapt to smm v2.
  const mapping::LaneProto* FindLaneByIdOrNull(mapping::ElementId id) const;

  const mapping::LaneInfo* FindLaneInfoOrNull(mapping::ElementId id) const;

  const std::vector<mapping::LaneInfo>& lane_info() const;

  const mapping::LaneBoundaryInfo* FindLaneBoundaryByIdOrNull(
      mapping::ElementId id) const;

  const mapping::IntersectionInfo* FindIntersectionByIdOrNull(
      mapping::ElementId id) const;

  const mapping::IntersectionInfo& IntersectionAt(int index) const;

  const mapping::CrosswalkInfo* FindCrosswalkByIdOrNull(
      mapping::ElementId id) const;

  const mapping::CrosswalkInfo& CrosswalkAt(int index) const;

  const mapping::TrafficLightProto* FindTrafficLightByIdOrNull(
      mapping::ElementId id) const;

  const mapping::SectionInfo* FindSectionInfoOrNull(
      mapping::SectionId id) const;

  // TODO(zixuan): Replace std::vector with absl::Span.
  const std::vector<mapping::SectionInfo>& section_info() const;

  const std::vector<mapping::ParkingSpotInfo>& parking_spot_info() const;

  const mapping::ParkingSpotInfo* FindParkingSpotByIdOrNull(
      mapping::ElementId id) const;

  std::string DebugString() const;

  std::string LaneInfoDebugString(mapping::ElementId lane_id) const;

 public:
  struct SemanticMapInfo {
    std::vector<mapping::LaneInfo> lane_info;
    std::vector<mapping::LaneBoundaryInfo> lane_boundary_info;
    std::vector<mapping::IntersectionInfo> intersection_info;
    std::vector<mapping::CrosswalkInfo> crosswalk_info;
    std::vector<mapping::ParkingSpotInfo> parking_spot_info;
    std::vector<mapping::SectionInfo> section_info;
  };

 private:
  std::shared_ptr<const mapping::v2::SemanticMapMultilevelSpatialIndex> smmsi_;
  CoordinateConverter coordinate_converter_;
  // Modifier contains speed limits, unit is m/s.
  PlannerSemanticMapModification modifier_;
  SemanticMapInfo info_;
};
}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_PLANNER_SEMANTIC_MAP_MANAGER_H_
