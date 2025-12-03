#ifndef ONBOARD_PLANNER_SPEED_ST_CLOSE_TRAJECTORY_H_
#define ONBOARD_PLANNER_SPEED_ST_CLOSE_TRAJECTORY_H_

#include <limits>
#include <optional>
#include <string>
#include <vector>

#include "absl/strings/string_view.h"

#include "onboard/planner/speed/proto/speed_finder.pb.h"

namespace qcraft::planner {

class StCloseTrajectory {
 public:
  struct StNearestPoint {
    double s = 0.0;
    double t = 0.0;
    // Relative to nearest path_segment direction.
    double v = 0.0;
    double lat_dist = 0.0;
    int obj_idx = 0;
  };

  // The input 'st_nearest_points' must sort by t.
  StCloseTrajectory(std::vector<StNearestPoint> st_nearest_points,
                    std::string id, std::string traj_id, std::string object_id,
                    StBoundaryProto::ObjectType object_type, double probability,
                    bool is_stationary);

  std::optional<StNearestPoint> GetNearestPointByTime(double time) const;

  absl::string_view object_id() const { return object_id_; }

  StBoundaryProto::ObjectType object_type() const { return object_type_; }

  double probability() const { return probability_; }

  bool is_stationary() const { return is_stationary_; }

  double min_t() const { return min_t_; }

  double max_t() const { return max_t_; }

 private:
  void Init();

  std::vector<StNearestPoint> st_nearest_points_;
  std::string id_;
  std::string traj_id_;
  std::string object_id_;
  StBoundaryProto::ObjectType object_type_;
  bool is_stationary_;
  double probability_ = 0.0;

  double min_t_ = std::numeric_limits<double>::max();
  double max_t_ = std::numeric_limits<double>::lowest();
  double min_s_ = std::numeric_limits<double>::max();
  double max_s_ = std::numeric_limits<double>::lowest();
};

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_SPEED_ST_CLOSE_TRAJECTORY_H_
