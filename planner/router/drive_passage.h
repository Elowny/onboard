#ifndef ONBOARD_PLANNER_ROUTER_DRIVE_PASSAGE_H_
#define ONBOARD_PLANNER_ROUTER_DRIVE_PASSAGE_H_

#include <algorithm>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "glog/logging.h"

#include "onboard/container/strong_int.h"
#include "onboard/container/strong_vector.h"
#include "onboard/global/trace.h"
#include "onboard/maps/lane_path.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/maps_common.h"
#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/frenet_frame.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/geometry/segment2d.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"

namespace qcraft::planner {

DECLARE_STRONG_VECTOR(Station);  // NOLINT

struct StationCenter {
  mapping::ElementId lane_id = mapping::kInvalidElementId;
  double fraction = 0.0;
  Vec2d xy;
  Vec2d tangent;
  double accum_s;
  double speed_limit;
  bool is_virtual;
  bool is_merging;
  bool is_in_intersection;
  mapping::LaneProto::Direction direction;

  mapping::LanePoint GetLanePoint() const {
    return mapping::LanePoint(lane_id, fraction);
  }

  Vec2d lat_point(double signed_offset) const {
    return xy + tangent.Perp() * signed_offset;
  }
  Vec2d lon_point(double signed_offset) const {
    return xy + tangent * signed_offset;
  }
  double lat_offset(const Vec2d& v) const { return tangent.CrossProd(v - xy); }
  double lon_offset(const Vec2d& v) const { return tangent.Dot(v - xy); }
};

struct CurbOffsetAndHeight {
  std::pair<double, double> offset;
  std::pair<double, double> height;
};

enum class StationBoundaryType {
  BROKEN_WHITE = 0,              // NOLINT
  SOLID_WHITE = 1,               // NOLINT
  BROKEN_YELLOW = 2,             // NOLINT
  SOLID_YELLOW = 3,              // NOLINT
  SOLID_DOUBLE_YELLOW = 4,       // NOLINT
  CURB = 5,                      // NOLINT
  VIRTUAL_CURB = 6,              // NOLINT
                                 // Left broken right solid
  BROKEN_LEFT_DOUBLE_WHITE = 7,  // NOLINT
  // Left solid right broken
  BROKEN_RIGHT_DOUBLE_WHITE = 8,  // NOLINT
  // Virtual curb specified for prediction to limit search range.
  PREDICTION_VIRTUAL_CURB = 9,  // NOLINT
  UNKNOWN_TYPE = 99             // NOLINT
};

inline std::string StationBoundaryTypeName(StationBoundaryType type) {
  switch (type) {
    case StationBoundaryType::SOLID_WHITE:
      return "SOLID_WHITE";
    case StationBoundaryType::SOLID_YELLOW:
      return "SOLID_YELLOW";
    case StationBoundaryType::SOLID_DOUBLE_YELLOW:
      return "SOLID_DOUBLE_YELLOW";
    case StationBoundaryType::CURB:
      return "CURB";
    case StationBoundaryType::VIRTUAL_CURB:
      return "VIRTUAL_CURB";
    case StationBoundaryType::UNKNOWN_TYPE:
      return "UNKNOWN_TYPE";
    case StationBoundaryType::BROKEN_WHITE:
      return "BROKEN_WHITE";
    case StationBoundaryType::BROKEN_YELLOW:
      return "BROKEN_YELLOW";
    case StationBoundaryType::BROKEN_LEFT_DOUBLE_WHITE:
      return "BROKEN_LEFT_DOUBLE_WHITE";
    case StationBoundaryType::BROKEN_RIGHT_DOUBLE_WHITE:
      return "BROKEN_RIGHT_DOUBLE_WHITE";
    case StationBoundaryType::PREDICTION_VIRTUAL_CURB:
      return "PREDICTION_VIRTUAL_CURB";
  }
}

struct StationBoundary {
  StationBoundaryType type;
  double lat_offset;
  double height = 0.0;

  bool IsSolid(double query_lat_offset) const {
    constexpr double kEpsilon = 0.5;  // m.
    switch (type) {
      case StationBoundaryType::SOLID_WHITE:
      case StationBoundaryType::SOLID_YELLOW:
      case StationBoundaryType::SOLID_DOUBLE_YELLOW:
      case StationBoundaryType::CURB:
      case StationBoundaryType::VIRTUAL_CURB:
      case StationBoundaryType::PREDICTION_VIRTUAL_CURB:
        return true;
      case StationBoundaryType::BROKEN_LEFT_DOUBLE_WHITE:
        return query_lat_offset < lat_offset - kEpsilon;
      case StationBoundaryType::BROKEN_RIGHT_DOUBLE_WHITE:
        return query_lat_offset > lat_offset + kEpsilon;
      case StationBoundaryType::UNKNOWN_TYPE:
      case StationBoundaryType::BROKEN_WHITE:
      case StationBoundaryType::BROKEN_YELLOW:
        return false;
    }
  }
};

using OptionalBoundary = std::optional<StationBoundary>;
struct BoundaryQueryResponse {
  OptionalBoundary right;
  OptionalBoundary left;
};

mapping::LaneBoundaryProto::Type GetBoundaryType(
    const mapping::LaneBoundaryInfo& boundary, int index);

StationBoundaryType MapBoundaryTypeToStationBoundaryType(
    mapping::LaneBoundaryProto::Type map_type);

struct PredictionLaneBoundary {
  double right_bound;
  StationBoundaryType right_type = StationBoundaryType::BROKEN_WHITE;
  double left_bound;
  StationBoundaryType left_type = StationBoundaryType::BROKEN_WHITE;
};
class PredictionLaneBoundaryLerper {
 public:
  PredictionLaneBoundary operator()(const PredictionLaneBoundary& a,
                                    const PredictionLaneBoundary& b,
                                    double alpha) const {
    return PredictionLaneBoundary{
        .right_bound = Lerp(a.right_bound, b.right_bound, alpha),
        .right_type = a.right_type,
        .left_bound = Lerp(a.left_bound, b.left_bound, alpha),
        .left_type = a.left_type,
    };
  }
};
// Boundary cache.
using LaneBoundaryCache = absl::flat_hash_map<
    mapping::ElementId,
    PiecewiseLinearFunction<planner::PredictionLaneBoundary, double,
                            planner::PredictionLaneBoundaryLerper>>;

class Station {
 public:
  explicit Station(StationCenter center, std::vector<StationBoundary> bounds)
      : center_(std::move(center)), boundaries_(std::move(bounds)) {}

  mapping::ElementId lane_id() const { return center_.lane_id; }
  const Vec2d& xy() const { return center_.xy; }
  const Vec2d& tangent() const { return center_.tangent; }
  double accumulated_s() const { return center_.accum_s; }
  double speed_limit() const { return center_.speed_limit; }
  bool is_virtual() const { return center_.is_virtual; }

  bool is_merging() const { return center_.is_merging; }
  bool is_in_intersection() const { return center_.is_in_intersection; }
  mapping::LaneProto::Direction direction() const { return center_.direction; }

  Vec2d lat_point(double signed_offset) const {
    return center_.lat_point(signed_offset);
  }
  Vec2d lon_point(double signed_offset) const {
    return center_.lon_point(signed_offset);
  }
  double lat_offset(const Vec2d& v) const { return center_.lat_offset(v); }
  double lon_offset(const Vec2d& v) const { return center_.lon_offset(v); }

  absl::Span<const StationBoundary> boundaries() const { return boundaries_; }

  absl::StatusOr<std::pair<double, double>> QueryCurbOffsetAt(
      double signed_lat) const;
  absl::StatusOr<CurbOffsetAndHeight> QueryCurbOffsetAndHeightAt(
      double signed_lat) const;

  absl::StatusOr<std::pair<StationBoundary, StationBoundary>>
  QueryCurbBoundariesAt(double signed_lat) const;
  // Including curb.
  absl::StatusOr<BoundaryQueryResponse> QueryEnclosingLaneBoundariesAt(
      double signed_lat) const;

  mapping::LanePoint GetLanePoint() const { return center_.GetLanePoint(); }

 private:
  // Stations are sampled on the target lane path.
  StationCenter center_;
  // Ordered by offset from right to left
  std::vector<StationBoundary> boundaries_;
};

struct StationWaypoint {
  StationIndex station_index;
  double lon_offset;
  double accum_s;
};

class DrivePassage {
 public:
  DrivePassage() = default;
  DrivePassage(StationVector<Station> stations, mapping::LanePath lane_path,
               mapping::LanePath extend_lane_path, double lane_path_start_s,
               bool reach_destination, FrenetFrameType type);

  DrivePassage(DrivePassage const& o)
      : stations_(o.stations_),
        last_real_station_index_(o.last_real_station_index_),
        center_seg_inv_len_(o.center_seg_inv_len_),
        lane_path_(o.lane_path_),
        extend_lane_path_(o.extend_lane_path_),
        beyond_lane_path_(o.beyond_lane_path_),
        reach_destination_(o.reach_destination_),
        lane_path_start_s_(o.lane_path_start_s_),
        segments_(o.segments_),
        type_(o.type_) {
    SCOPED_QTRACE("DrivePassage::copy constructor");
    BuildFrenetFrame();
  }

  DrivePassage& operator=(DrivePassage const& o) {
    SCOPED_QTRACE("DrivePassage::operator=");
    stations_ = o.stations_;
    last_real_station_index_ = o.last_real_station_index_;
    center_seg_inv_len_ = o.center_seg_inv_len_;
    lane_path_ = o.lane_path_;
    extend_lane_path_ = o.extend_lane_path_;
    beyond_lane_path_ = o.beyond_lane_path_;
    reach_destination_ = o.reach_destination_;
    lane_path_start_s_ = o.lane_path_start_s_;
    segments_ = o.segments_;
    type_ = o.type_;
    BuildFrenetFrame();
    return *this;
  }

  DrivePassage(DrivePassage&& o) = default;
  DrivePassage& operator=(DrivePassage&& o) = default;

  // ########## query operations ##########
  // NOTE (boqian): each pair.first represents the right side and .second the
  // left side, with all lateral offsets on the right side always smaller than 0
  absl::StatusOr<double> QuerySpeedLimitAt(const Vec2d& point) const;
  absl::StatusOr<double> QuerySpeedLimitAtS(double s) const;

  absl::StatusOr<std::pair<double, double>> QueryCurbOffsetAt(
      const Vec2d& point) const;
  absl::StatusOr<CurbOffsetAndHeight> QueryCurbOffsetAndHeightAt(
      const Vec2d& point) const;
  absl::StatusOr<std::pair<double, double>> QueryCurbOffsetAtS(double s) const;
  absl::StatusOr<CurbOffsetAndHeight> QueryCurbOffsetAndHeightAtS(
      double s) const;

  absl::StatusOr<std::pair<StationBoundary, StationBoundary>>
  QueryCurbBoundariesAt(const Vec2d& point) const;
  absl::StatusOr<std::pair<StationBoundary, StationBoundary>>
  QueryCurbBoundariesAtS(double s) const;

  absl::StatusOr<std::pair<double, double>> QueryNearestBoundaryLateralOffset(
      double s) const;

  absl::StatusOr<std::pair<Vec2d, Vec2d>> QueryCurbPointAt(
      const Vec2d& point) const;

  absl::StatusOr<std::pair<Vec2d, Vec2d>> QueryCurbPointAtS(double s) const;

  absl::StatusOr<BoundaryQueryResponse> QueryEnclosingLaneBoundariesAt(
      const Vec2d& point) const;
  BoundaryQueryResponse QueryEnclosingLaneBoundariesAtS(double s) const;

  absl::StatusOr<Vec2d> QueryLaterallyUnboundedTangentAt(
      const Vec2d& point) const;

  absl::StatusOr<Vec2d> QueryTangentAt(const Vec2d& point) const;
  absl::StatusOr<Vec2d> QueryTangentAtS(double s) const;
  absl::StatusOr<double> QueryTangentAngleAtS(double s) const;

  absl::StatusOr<Vec2d> QueryPointXYAtS(double s) const;

  absl::StatusOr<Vec2d> QueryPointXYAtSL(double s, double l) const;

  absl::StatusOr<StationWaypoint> QueryFrenetLonOffsetAt(
      const Vec2d& point) const;

  absl::StatusOr<double> QueryFrenetLatOffsetAt(const Vec2d& point) const;

  absl::StatusOr<FrenetCoordinate> QueryFrenetCoordinateAt(
      const Vec2d& point) const;

  absl::StatusOr<FrenetBox> QueryFrenetBoxAt(const Box2d& box) const;

  absl::StatusOr<FrenetBox> QueryFrenetBoxAtContour(
      const Polygon2d& contour) const;

  // Returns projection that is not bounded to the drive passage when object is
  // not near drive passage.
  absl::StatusOr<FrenetCoordinate> QueryLaterallyUnboundedFrenetCoordinateAt(
      const Vec2d& point) const;

  absl::StatusOr<FrenetCoordinate> QueryUnboundedFrenetCoordinateAt(
      const Vec2d& point) const;

  // Query a set of points
  absl::StatusOr<std::vector<FrenetCoordinate>> BatchQueryFrenetCoordinates(
      absl::Span<const Vec2d> points) const;

  absl::StatusOr<std::vector<std::optional<FrenetBox>>> BatchQueryFrenetBoxes(
      absl::Span<const Box2d> boxes, bool laterally_bounded) const;

  // ######## query operations end ########

  // Find the nearest point on center line.
  absl::StatusOr<Vec2d> FindNearestPointOnCenterLine(const Vec2d& point) const;

  // Find the nearest station from a point.
  StationIndex FindNearestStationIndex(const Vec2d& point) const;
  const Station& FindNearestStation(const Vec2d& point) const {
    return stations_[FindNearestStationIndex(point)];
  }
  // Find the nearest station from s.
  StationIndex FindNearestStationIndexAtS(double s) const;
  const Station& FindNearestStationAtS(double s) const {
    return stations_[FindNearestStationIndexAtS(s)];
  }

  double end_s() const { return stations_.back().accumulated_s(); }
  double front_s() const { return stations_.front().accumulated_s(); }
  double lane_path_start_s() const { return lane_path_start_s_; }
  bool beyond_lane_path() const { return beyond_lane_path_; }
  bool reach_destination() const { return reach_destination_; }

  bool ContainIntersection(const int check_step) const;

  bool empty() const { return stations_.size() == 0; }
  int size() const { return stations_.size(); }

  const Station& station(StationIndex index) const { return stations_[index]; }
  const StationVector<Station>& stations() const { return stations_; }
  StationIndex last_real_station_index() const {
    return last_real_station_index_;
  }

  // Based on which the drive passage is built.
  // extend_lane_path has more segments on both sides than lane path. drive
  // passage's length is equal to extend_lane_path length. extend_lane_path is
  // also used to calculate traffic lights info on route as we want to
  // investigate the stoplines behind us.
  const mapping::LanePath& extend_lane_path() const {
    return extend_lane_path_;
  }

  // lane_path starts from plan_start_state and ends at route end or horizon
  // end. Route end stopline is calculated based on lane path.
  const mapping::LanePath& lane_path() const { return lane_path_; }

  const std::vector<Segment2d>& segments() const { return segments_; }

  const FrenetFrame* frenet_frame() const { return frenet_frame_.get(); }

 private:
  struct ProjectionResult {
    StationIndex station_index_1;
    StationIndex station_index_2;
    StationIndex near_station_index;
    double accum_s;
    double signed_l;
    double lerp_factor;
  };

  absl::StatusOr<ProjectionResult> ProjectPointToStations(
      const Vec2d& point, bool allow_extrapolation) const;
  absl::StatusOr<ProjectionResult> ProjectPointToStationsWithinRadius(
      const Vec2d& point, const ProjectionResult& projection,
      const Vec2d& prev_point, double search_radius) const;
  absl::Status IsProjectionResultOnDrivePassage(
      const ProjectionResult& res) const;
  absl::StatusOr<std::optional<FrenetBox>> QueryFrenetBoxWithinRadius(
      const Box2d& box, const ProjectionResult& center_projection,
      double search_radius, bool laterally_bounded) const;
  absl::StatusOr<FrenetBox> QueryFrenetBoxAtContourPoints(
      absl::Span<const Vec2d> contour_points) const;

  struct BinarySeachResult {
    StationIndex station_index_1;
    StationIndex station_index_2;
    StationIndex near_station_index;
    double ds;  // s - prev_s
  };

  BinarySeachResult BinarySearchForNearStation(double s) const;

  void BuildFrenetFrame();

  // WARNING! The c'tors and assiggnment ops defined above must also be updated
  // once a new field be added!
  StationVector<Station> stations_;
  StationIndex last_real_station_index_;

  std::vector<double> center_seg_inv_len_;  // One less than stations.
  mapping::LanePath lane_path_;
  mapping::LanePath extend_lane_path_;
  bool beyond_lane_path_;
  bool reach_destination_;
  double lane_path_start_s_;
  std::vector<Segment2d> segments_;

  FrenetFrameType type_;
  std::unique_ptr<FrenetFrame> frenet_frame_;
};

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_ROUTER_DRIVE_PASSAGE_H_
