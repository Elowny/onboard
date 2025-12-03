#ifndef ONBOARD_PREDICTION_POST_PROCESS_RELATION_ANALYZER_H_
#define ONBOARD_PREDICTION_POST_PROCESS_RELATION_ANALYZER_H_

#include <optional>
#include <string>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/hash/hash.h"
#include "absl/types/span.h"
#include "glog/logging.h"

#include "onboard/container/strong_int.h"
#include "onboard/container/strong_vector.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/geometry/proto/box2d.pb.h"
#include "onboard/math/segment_matcher/segment_matcher_kdtree.h"
#include "onboard/prediction/predicted_trajectory.h"
#include "onboard/prediction/proto/prediction_common.pb.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/proto/trajectory.pb.h"
#include "onboard/proto/trajectory_point.pb.h"

namespace qcraft {
namespace prediction {
enum class AgentRelationType {
  // Av passing agent.
  ART_PASS = 1,  // NOLINT
  // Av yield to agent.
  ART_YIELD = 2,  // NOLINT
  // No relation.
  ART_NONE = 3,  // NOLINT
};

// Object conflict manager will build this input.
struct AgentRelationAnalyzerInput {
  std::string traj_id;
  const ObjectProto* object_proto = nullptr;
  const PredictedTrajectory* trajectory = nullptr;
  const SegmentMatcherKdtree* segments = nullptr;
};

struct AvObjMeetingPoints {
  ApolloTrajectoryPointProto av_point;
  PredictedTrajectoryPoint obj_point;
};

DECLARE_STRONG_VECTOR(TrajectoryNode);
struct TrajectoryNode {
  TrajectoryNodeIndex index =
      TrajectoryNodeVector<TrajectoryNode>::kInvalidIndex;
  std::string traj_id;
  int influencers_count = 0;
  absl::flat_hash_set<TrajectoryNodeIndex> influencers;
  absl::flat_hash_set<TrajectoryNodeIndex> reactors;
};

AgentRelationType AnalyzeTrajectoryRelation(
    const AgentRelationAnalyzerInput& agent_1,
    const AgentRelationAnalyzerInput& agent_2);

std::optional<AvObjMeetingPoints> FindAvObjMeetingPoints(
    const TrajectoryProto& av_trajectory,
    const SegmentMatcherKdtree& av_segment_matcher,
    const ObjectProto& object_proto,
    const PredictedTrajectory& object_trajectory,
    absl::Span<const qcraft::Box2dProto* const> object_shapes);

std::optional<AvObjMeetingPoints> FindAvObjMeetingPointsWithShape(
    const TrajectoryProto& av_trajectory,
    const SegmentMatcherKdtree& av_segment_matcher, const Box2d& av_box,
    const ObjectProto& object_proto,
    const PredictedTrajectory& object_trajectory,
    absl::Span<const qcraft::Box2dProto* const> object_shapes);

// Add label in oracle module.
ObjectRelationProto AnalyzeObjectRelation(
    const TrajectoryProto& av_trajectory,
    const SegmentMatcherKdtree& av_segment_matcher,
    const ObjectProto& object_proto,
    const PredictedTrajectory& object_trajectory,
    absl::Span<const qcraft::Box2dProto* const> object_shapes,
    std::optional<Box2d> av_box);

std::vector<TrajectoryNode> BatchOnPathAnalyze(
    absl::Span<const AgentRelationAnalyzerInput> inputs);

std::vector<std::vector<TrajectoryNodeIndex>> BuildPriorityGraph(
    std::vector<TrajectoryNode>* nodes);

}  // namespace prediction
}  // namespace qcraft

#endif  // ONBOARD_PREDICTION_POST_PROCESS_RELATION_ANALYZER_H_
