#ifndef ONBOARD_PREDICTION_POST_PROCESS_OBJECT_ST_MAP_H_
#define ONBOARD_PREDICTION_POST_PROCESS_OBJECT_ST_MAP_H_

#include <memory>
#include <set>
#include <string>
#include <vector>

#include "absl/status/status.h"
#include "absl/types/span.h"

#include "onboard/async/thread_pool.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/segment_matcher/segment_matcher_kdtree.h"
#include "onboard/planner/common/discretized_path.h"
#include "onboard/planner/speed/speed_vector.h"
#include "onboard/planner/speed/st_boundary.h"
#include "onboard/prediction/post_process/conflict_resolver_params.h"
#include "onboard/prediction/post_process/object_conflict_manager.h"
#include "onboard/prediction/predicted_trajectory.h"
#include "onboard/prediction/proto/conflict_resolver.pb.h"
#include "onboard/proto/perception.pb.h"

namespace qcraft {
namespace prediction {

class ObjectStMap {
  // Define a s-t region for a specified predicted trajectory of an
  // object.
 public:
  explicit ObjectStMap(const ObjectConflictManager& conflict_mgr,
                       const ObjectProto& object_proto,
                       const planner::DiscretizedPath& path,
                       const planner::SpeedVector& ref_speed,
                       const PredictedTrajectory& predicted_trajectory,
                       const ConflictResolverParams* params,
                       ThreadPool* thread_pool);

  absl::Status MapStationaryObjects(absl::Span<const std::string> traj_ids);
  void MapTLRedLanes(const std::set<mapping::ElementId>& red_tls,
                     absl::Span<const mapping::ElementId> possible_lane_path);

  std::vector<double> QueryStoplines() const;
  std::vector<double> QueryStationaryObjects() const;
  inline bool HasConflict() const {
    return !stationary_objects_.empty() || !stop_lines_.empty();
  }

  std::string Annotation() const;

 private:
  ThreadPool* thread_pool_ = nullptr;
  const ObjectConflictManager* conflict_mgr_;
  const PredictedTrajectory* predicted_traj_;
  const ConflictResolutionConfigProto::ConflictResolverConfig* general_config_;
  const ConflictResolutionConfigProto::ObjectConflictResolverConfig*
      object_config_;

  // From object prediction results.
  const planner::DiscretizedPath* path_;
  const planner::SpeedVector* ref_speed_;
  const ObjectProto* av_proto_;

  std::vector<Box2d> av_boxes_on_path_;

  std::unique_ptr<SegmentMatcherKdtree> path_kd_tree_;
  double av_radius_;
  double max_plan_time_ = 10.0;
  double max_length_;

  // St boundaries mapped onto ObjectStMap.
  std::vector<planner::StBoundaryRef> stationary_objects_;
  std::vector<planner::StBoundaryRef> stop_lines_;
};

}  // namespace prediction
}  // namespace qcraft
#endif  // ONBOARD_PREDICTION_POST_PROCESS_OBJECT_ST_MAP_H_
