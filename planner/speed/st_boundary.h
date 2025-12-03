#ifndef ONBOARD_PLANNER_SPEED_ST_BOUNDARY_H_
#define ONBOARD_PLANNER_SPEED_ST_BOUNDARY_H_

#include <stddef.h>

#include <iterator>
#include <limits>
#include <memory>
#include <optional>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

#include "glog/logging.h"

#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/vec.h"
#include "onboard/planner/speed/overlap_info.h"
#include "onboard/planner/speed/proto/speed_finder.pb.h"
#include "onboard/planner/speed/st_point.h"
#include "onboard/planner/speed/vt_point.h"
#include "onboard/proto/perception/fusion/object.pb.h"

namespace qcraft::planner {

class StBoundary;

using StBoundaryRef = std::unique_ptr<StBoundary>;

struct StBoundaryPoints {
  std::vector<StPoint> lower_points;
  std::vector<StPoint> upper_points;
  std::vector<VtPoint> speed_points;
  std::vector<OverlapInfo> overlap_infos;
  StBoundaryProto::ProtectionType protection_type =
      StBoundaryProto::NON_PROTECTIVE;
  void Reserve(size_t num) {
    lower_points.reserve(num);
    upper_points.reserve(num);
    speed_points.reserve(num);
    overlap_infos.reserve(num);
  }
};

class StBoundary : public Polygon2d {
 public:
  explicit StBoundary(const Box2d& box) = delete;
  explicit StBoundary(std::vector<Vec2d> points) = delete;
  StBoundary(const StBoundary&) = delete;
  StBoundary(StBoundary&&) = delete;
  StBoundary& operator=(const StBoundary&) const = delete;
  StBoundary& operator=(StBoundary&&) const = delete;

  static StBoundaryRef CreateInstance(
      const StBoundaryPoints& st_boundary_points,
      StBoundaryProto::ObjectType object_type, std::string id,
      double probability, bool is_stationary,
      StBoundaryProto::ProtectionType protection_type, bool is_large_vehicle);

  static StBoundaryRef CopyInstance(const StBoundary& st_boundary);

  virtual ~StBoundary() = default;

  bool IsEmpty() const { return lower_points_.empty(); }

  bool IsPointInBoundary(const StPoint& st_point) const;

  StPoint upper_left_point() const {
    QCHECK(!upper_points_.empty());
    return upper_points_.front();
  }

  StPoint upper_right_point() const {
    QCHECK(!upper_points_.empty());
    return upper_points_.back();
  }

  StPoint bottom_left_point() const {
    QCHECK(!lower_points_.empty());
    return lower_points_.front();
  }

  StPoint bottom_right_point() const {
    QCHECK(!lower_points_.empty());
    return lower_points_.back();
  }

  void ExpandByT(double left, double right);

  StBoundarySourceTypeProto::Type source_type() const { return source_type_; }
  void set_source_type(StBoundarySourceTypeProto::Type source_type) {
    source_type_ = source_type;
  }

  static StBoundarySourceTypeProto::Type ObjectTypeToSourceType(
      StBoundaryProto::ObjectType object_type);

  StBoundaryProto::ObjectType object_type() const { return object_type_; }

  void set_object_type(StBoundaryProto::ObjectType type) {
    QCHECK(type != StBoundaryProto::UNKNOWN_OBJECT);

    object_type_ = type;
    set_source_type(ObjectTypeToSourceType(type));
  }

  const std::string& id() const { return id_; }
  void set_id(const std::string& id);
  const std::optional<std::string>& traj_id() const { return traj_id_; }
  const std::optional<std::string>& object_id() const { return object_id_; }

  double probability() const { return probability_; }
  void set_probability(double probability) {
    DCHECK_GE(probability, 0.0);
    DCHECK_LE(probability, 1.0);
    probability_ = probability;
  }

  bool is_stationary() const { return is_stationary_; }
  void set_is_stationary(bool is_stationary) { is_stationary_ = is_stationary; }

  bool is_protective() const {
    return protection_type_ != StBoundaryProto::NON_PROTECTIVE;
  }

  StBoundaryProto::ProtectionType protection_type() const {
    return protection_type_;
  }
  void set_protection_type(StBoundaryProto::ProtectionType protection_type) {
    protection_type_ = protection_type;
  }

  // The corresponding original st-boundary id. Only has value when protection
  // type equals StBoundaryProto::SMALL_ANGLE_CUT_IN.
  std::optional<std::string> protected_st_boundary_id() const {
    return protected_st_boundary_id_;
  }
  void set_protected_st_boundary_id(std::string protected_st_boundary_id) {
    protected_st_boundary_id_ = std::move(protected_st_boundary_id);
  }

  bool is_large_vehicle() const { return is_large_vehicle_; }
  void set_is_large_vehicle_vehicle(bool is_large_vehicle) {
    is_large_vehicle_ = is_large_vehicle;
  }

  // Return value: <s_upper, s_lower>.
  std::optional<std::pair<double, double>> GetBoundarySRange(
      double curr_time) const;

  std::optional<double> GetStBoundarySpeedAtT(double t) const;

  bool GetLowerPointsIndexRange(double t, int* left, int* right) const;

  bool GetUpperPointsIndexRange(double t, int* left, int* right) const;

  bool GetSpeedPointsIndexRange(double t, int* left, int* right) const;

  double min_s() const { return min_s_; }
  double min_t() const { return min_t_; }
  double max_s() const { return max_s_; }
  double max_t() const { return max_t_; }

  const std::vector<StPoint>& upper_points() const { return upper_points_; }
  const std::vector<StPoint>& lower_points() const { return lower_points_; }
  const std::vector<VtPoint>& speed_points() const { return speed_points_; }
  const std::vector<OverlapInfo>& overlap_infos() const {
    return overlap_infos_;
  }

  void set_speed_points(std::vector<VtPoint> speed_points) {
    speed_points_ = std::move(speed_points);
  }

  const std::optional<StOverlapMetaProto>& overlap_meta() const {
    return overlap_meta_;
  }
  void set_overlap_meta(StOverlapMetaProto meta) {
    overlap_meta_ = std::move(meta);
  }

  void Init(std::vector<std::pair<StPoint, StPoint>> point_pairs);

  // Try to recover the original object id based on st_boundary id for
  // st_objects. If the input is not a st_object, or if it is st_object but have
  // wrong id format(doesn't include "-idx"), return false. Otherwise, return
  // true and the recovered id.
  static std::optional<std::string> RecoverObjectId(
      const std::string& st_boundary_id,
      StBoundarySourceTypeProto::Type source_type);

  static std::optional<std::string> RecoverTrajId(
      const std::string& st_boundary_id,
      StBoundarySourceTypeProto::Type source_type);

 private:
  StBoundary(std::vector<std::pair<StPoint, StPoint>> point_pairs,
             std::vector<VtPoint> speed_points,
             std::vector<OverlapInfo> overlap_infos,
             StBoundaryProto::ObjectType object_type, std::string id,
             double probability, bool is_stationary,
             StBoundaryProto::ProtectionType protection_type,
             bool is_large_vehicle,
             std::optional<StOverlapMetaProto> overlap_meta,
             std::optional<std::string> protected_st_boundary_id);

  template <typename T>
  bool QueryIndexRange(const std::vector<T>& points, double t, int* left,
                       int* right) const;

 private:
  StBoundaryProto::ObjectType object_type_ = StBoundaryProto::UNKNOWN_OBJECT;
  StBoundarySourceTypeProto::Type source_type_ =
      StBoundarySourceTypeProto::UNKNOWN;
  StBoundaryProto::ProtectionType protection_type_ =
      StBoundaryProto::NON_PROTECTIVE;

  std::vector<StPoint> lower_points_;
  std::vector<StPoint> upper_points_;
  // Size not necessarily equal to lower_points and upper_points.
  std::vector<VtPoint> speed_points_;
  // Size equal to lower_points and upper_points only before ExpandByT.
  std::vector<OverlapInfo> overlap_infos_;

  std::string id_;

  // Only valid for of ST_OBJECT.
  std::optional<std::string> traj_id_;
  std::optional<std::string> object_id_;

  double probability_ = 0.0;
  bool is_stationary_ = false;
  bool is_large_vehicle_ = false;

  double min_s_ = std::numeric_limits<double>::max();
  double max_s_ = std::numeric_limits<double>::lowest();
  double min_t_ = std::numeric_limits<double>::max();
  double max_t_ = std::numeric_limits<double>::lowest();

  std::optional<StOverlapMetaProto> overlap_meta_;
  std::optional<std::string> protected_st_boundary_id_ = std::nullopt;
};

// If source_type is StObject, integration_id is object_id,
// otherwise, integration_id is st_boundary_id.
std::string GetStBoundaryIntegrationId(const StBoundary& st_boundary);

StBoundaryProto::ObjectType ToStBoundaryObjectType(ObjectType type);

template <typename T>
bool StBoundary::QueryIndexRange(const std::vector<T>& points, double t,
                                 int* left, int* right) const {
  QCHECK_NOTNULL(left);
  QCHECK_NOTNULL(right);
  QCHECK_GT(points.size(), 1);
  if (t < points.front().t() || t > points.back().t()) {
    QLOG(WARNING) << "t is out of range. t = " << t
                  << " range: " << points.front().t() << ", "
                  << points.back().t();
    return false;
  }
  const auto it =
      std::lower_bound(points.begin(), points.end(), t,
                       [](const T& p, double t) { return p.t() < t; });
  const int index = std::distance(points.begin(), it);
  if (index == 0) {
    *left = 0;
    *right = *left + 1;
  } else if (it == points.end()) {
    *left = points.size() - 1;
    *right = *left - 1;
  } else {
    *left = index - 1;
    *right = index;
  }
  return true;
}

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_SPEED_ST_BOUNDARY_H_
