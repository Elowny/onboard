#include "onboard/prediction/post_process/relation_analyzer.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type
// IWYU pragma: no_include <google/protobuf/repeated_ptr_field.h>
#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <ostream>
#include <utility>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"

#include "onboard/global/buffered_logger.h"
#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/math/fast_math.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/geometry/segment2d.h"
#include "onboard/math/vec.h"
#include "onboard/planner/second_order_trajectory_point.h"
#include "onboard/planner/trajectory_point.h"
#include "onboard/planner/trajectory_util.h"
#include "onboard/proto/perception/fusion/object.pb.h"
#include "onboard/proto/prediction.pb.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/utils/status_macros.h"

namespace qcraft {
namespace prediction {

namespace {
constexpr double kInfluenceAngleThreshold = 7.0 / 16.0 * M_PI;
const double kCosInfluenceAngleThreshold =
    fast_math::Cos(kInfluenceAngleThreshold);
constexpr double kYieldIgnoreAngleRad = 3.0 / 4.0 * M_PI;  // Rad.
constexpr double kYieldIgnoreDistance = 5.0;               // m.

using TrajectoryPoint = planner::TrajectoryPoint;

ApolloTrajectoryPointProto PredictedTrajectoryPointToApolloTrajectoryPointProto(
    const PredictedTrajectoryPoint& pred_point) {
  TrajectoryPointProto pt_proto;
  pred_point.SecondOrderTrajectoryPoint::ToProto(&pt_proto);
  return planner::ToApolloTrajectoryPointProto(pt_proto);
}

double GetAvailableTimeHorizon(const TrajectoryProto& av_trajectory,
                               const PredictedTrajectory& object_trajectory) {
  const auto& av_traj_size = av_trajectory.trajectory_point().size();
  if (av_traj_size == 0 || object_trajectory.points().size() == 0) {
    return 0.0;
  }
  const auto& traj_point = av_trajectory.trajectory_point(av_traj_size - 1);
  const auto& av_max_time = traj_point.relative_time();
  const auto& object_max_time = object_trajectory.points().back().t();
  return std::min(av_max_time, object_max_time);
}

bool IsObjectRelationAnalyzeType(ObjectType type) {
  // Object type needs to have analyze relation.
  switch (type) {
    case ObjectType::OT_VEHICLE:
    case ObjectType::OT_LARGE_VEHICLE:
    case ObjectType::OT_MOTORCYCLIST:
    case ObjectType::OT_CYCLIST:
    case ObjectType::OT_TRICYCLIST:
    case ObjectType::OT_PEDESTRIAN:
    case ObjectType::OT_UNKNOWN_MOVABLE:
      return true;
    case ObjectType::OT_UNKNOWN_STATIC:
    case ObjectType::OT_BARRIER:
    case ObjectType::OT_BARRIER_ANTI_COLLISION_BUCKET:
    case ObjectType::OT_BARRIER_ANTI_COLLISION_POST:
    case ObjectType::OT_FOD:
    case ObjectType::OT_VEGETATION:

    case ObjectType::OT_CONE:
    case ObjectType::OT_WARNING_TRIANGLE:
      return false;
  }
}

bool IsObjectCurrentPositionBehindAVAndFarAway(
    const TrajectoryProto& av_trajectory,
    const PredictedTrajectory& object_trajectory) {
  if (av_trajectory.trajectory_point().size() == 0 ||
      object_trajectory.points().size() == 0) {
    return false;
  }
  const auto& av_point = av_trajectory.trajectory_point()[0];
  const auto& object_point = object_trajectory.points().front();
  const Vec2d av_pos(av_point.path_point().x(), av_point.path_point().y());
  const auto& object_pos = object_point.pos();
  const auto rel_pos = object_pos - av_pos;
  const Vec2d av_heading(fast_math::Cos(av_point.path_point().theta()),
                         fast_math::Sin(av_point.path_point().theta()));
  return av_heading.Dot(rel_pos.Unit()) <
             fast_math::Cos(kYieldIgnoreAngleRad) &&
         rel_pos.Length() > kYieldIgnoreDistance;
}

ApolloTrajectoryPointProto FindClosestTrajectoryPointToSegment(
    const TrajectoryProto& av_trajectory, const Segment2d& av_segment) {
  const ApolloTrajectoryPointProto* ptr_point = nullptr;
  double min_distance = std::numeric_limits<double>::max();
  constexpr double kEpsilon = 0.1;
  for (const auto& pt : av_trajectory.trajectory_point()) {
    const auto pt_seg_distance =
        av_segment.DistanceTo(Vec2d(pt.path_point().x(), pt.path_point().y()));
    if (pt_seg_distance < min_distance - kEpsilon) {
      min_distance = pt_seg_distance;
      ptr_point = &pt;
    }
  }
  QCHECK_NOTNULL(ptr_point);
  return *ptr_point;
}

std::pair<ApolloTrajectoryPointProto, int> FindClosestTrajectoryPointToPosition(
    const TrajectoryProto& trajectory, const Vec2d& position) {
  const ApolloTrajectoryPointProto* ptr_point = nullptr;
  int min_pt_idx = trajectory.trajectory_point_size();
  double min_distance = std::numeric_limits<double>::max();
  for (int i = 0; i < trajectory.trajectory_point_size(); ++i) {
    const auto& pt = trajectory.trajectory_point(i);
    const auto pt_pos_distance =
        position.DistanceTo(Vec2d(pt.path_point().x(), pt.path_point().y()));
    constexpr double kDistanceEpsilon = 1e-3;
    if (pt_pos_distance < min_distance - kDistanceEpsilon) {
      min_distance = pt_pos_distance;
      ptr_point = &pt;
      min_pt_idx = i;
    }
  }
  QCHECK_NOTNULL(ptr_point);
  return std::make_pair(*ptr_point, min_pt_idx);
}

double DecideInfluenceThreshold(const ObjectProto& agent_1,
                                const ObjectProto& agent_2) {
  const Box2d bb_box_1(agent_1.bounding_box());
  const Box2d bb_box_2(agent_2.bounding_box());
  return bb_box_1.diagonal() * 0.5 + bb_box_2.diagonal() * 0.5;
}

absl::StatusOr<std::vector<int>> IndexOrderedSegmentWithinRadius(
    const SegmentMatcherKdtree& segment_matcher, const Vec2d& pos,
    double radius) {
  auto segment_idxes =
      segment_matcher.GetSegmentIndexInRadius(pos.x(), pos.y(), radius);
  if (segment_idxes.empty()) {
    return absl::NotFoundError("No segments within radius.");
  }
  std::stable_sort(segment_idxes.begin(), segment_idxes.end());
  return segment_idxes;
}

// pair.first, closest distance b/t segments (within a radius from the point)
// and the point. pair.second, the closest segment idx.
// Search segments within a radius from a position
// (x, y), calculate the closest distance between the segments and the
// point. Return the distance and the closest segment idx as a pair.
absl::StatusOr<std::pair<double, int>> ClosestDistanceWithinRadius(
    const SegmentMatcherKdtree& segment_matcher, const Vec2d& pos,
    double radius) {
  const auto segment_idxes =
      segment_matcher.GetSegmentIndexInRadius(pos.x(), pos.y(), radius);
  if (segment_idxes.empty()) {
    return absl::NotFoundError(absl::StrFormat(
        "Can not find close segments within influence threshold: %f.", radius));
  }
  int closest_idx = -1;
  double closest_distance = std::numeric_limits<double>::infinity();
  for (const auto& idx : segment_idxes) {
    const auto* segment = segment_matcher.GetSegmentByIndex(idx);
    if (segment == nullptr) {
      continue;
    }
    const double distance = segment->DistanceTo(pos);
    if (distance < closest_distance) {
      closest_distance = distance;
      closest_idx = idx;
    }
  }
  if (closest_idx < 0) {
    return absl::NotFoundError(
        "Can not find closest segment, search radius might be too small.");
  }
  return std::make_pair(closest_distance, closest_idx);
}

enum class OnPathAnalyzerType {
  kOnPath = 1,
  kNone = 2,
};

OnPathAnalyzerType OnPathAnalyzer(const AgentRelationAnalyzerInput& agent_1,
                                  const AgentRelationAnalyzerInput& agent_2) {
  const auto& traj_1 = *agent_1.trajectory;
  const auto& traj_2 = *agent_2.trajectory;
  const auto& object_proto_1 = *agent_1.object_proto;
  const auto& object_proto_2 = *agent_2.object_proto;

  if (object_proto_1.id() == object_proto_2.id()) {
    return OnPathAnalyzerType::kNone;
  }
  if (traj_2.type() == PT_STATIONARY) {
    return OnPathAnalyzerType::kNone;
  }
  // Only check the first point of agent 1 against predicted trajectory of
  // agent 2. Do not use segment matcher.
  if (traj_1.points().empty() || traj_2.points().empty()) {
    return OnPathAnalyzerType::kNone;
  }

  const auto& first_point_1 = traj_1.points().front();
  const auto& first_point_2 = traj_2.points().front();
  const auto& max_length_2 = traj_2.points().back().s();
  const auto distance = first_point_1.pos().DistanceTo(first_point_2.pos());
  if (max_length_2 < distance) {
    return OnPathAnalyzerType::kNone;
  }

  const Box2d box_1(object_proto_1.bounding_box());
  const Box2d box_2(object_proto_2.bounding_box());

  // Strictly on path.
  for (const auto& point : traj_2.points()) {
    const auto distance = first_point_1.pos().DistanceTo(point.pos());
    if (distance < box_1.half_width() || distance < box_2.half_width()) {
      return OnPathAnalyzerType::kOnPath;
    }
  }
  return OnPathAnalyzerType::kNone;
}

// Sampled points: start_s = 0, end_s = s, sample n points.
std::vector<ApolloTrajectoryPointProto>
UniformlySamplePredictedTrajectoryUntilSToNPoints(
    const PredictedTrajectory& pred_traj, double s, int n) {
  QCHECK_GE(n, 2);
  std::vector<ApolloTrajectoryPointProto> pts;
  pts.reserve(n);
  const double max_s = std::min((*pred_traj.points().rbegin()).s(), s);
  const double ds = max_s / (n - 1);
  for (int i = 0; i < n; ++i) {
    const double query_s = std::min(ds * i, max_s);
    const auto pred_pt = planner::QueryTrajectoryPointByS(
        absl::MakeSpan(pred_traj.points()), query_s);
    pts.push_back(
        PredictedTrajectoryPointToApolloTrajectoryPointProto(pred_pt));
  }
  return pts;
}

std::vector<ApolloTrajectoryPointProto>
UniformlySampleTrajectoryProtoUntilSToNPoints(
    const TrajectoryProto& trajectory_proto, double s, int n) {
  QCHECK_GE(n, 2);
  std::vector<ApolloTrajectoryPointProto> pts;
  pts.reserve(n);
  const auto& last_pt = *trajectory_proto.trajectory_point().rbegin();
  const double max_s = std::min(last_pt.path_point().s(), s);
  const double ds = max_s / (n - 1);
  for (int i = 0; i < n; ++i) {
    const double query_s = std::min(ds * i, max_s);
    pts.push_back(
        planner::QueryApolloTrajectoryPointByS(trajectory_proto, query_s));
  }
  return pts;
}

void FillPathsForObjectRelationInfo(const TrajectoryProto& av_traj,
                                    const PredictedTrajectory& obj_traj,
                                    ObjectRelationProto* relation_proto) {
  constexpr int kGroundTruthTrajSampleNum = 20;
  // Should have interaction point results here!
  QCHECK(relation_proto->has_av_point());
  QCHECK(relation_proto->has_obj_point());
  const double av_s = relation_proto->av_point().path_point().s();
  const double obj_s = relation_proto->obj_point().path_point().s();
  auto av_path = UniformlySampleTrajectoryProtoUntilSToNPoints(
      av_traj, av_s, kGroundTruthTrajSampleNum);
  auto obj_path = UniformlySamplePredictedTrajectoryUntilSToNPoints(
      obj_traj, obj_s, kGroundTruthTrajSampleNum);
  for (auto& av_pt : av_path) {
    *relation_proto->add_av_path() = std::move(av_pt);
  }
  for (auto& obj_pt : obj_path) {
    *relation_proto->add_obj_path() = std::move(obj_pt);
  }
  QCHECK_EQ(relation_proto->obj_path_size(), kGroundTruthTrajSampleNum);
  QCHECK_EQ(relation_proto->av_path_size(), kGroundTruthTrajSampleNum);
}

}  // namespace

AgentRelationType AnalyzeTrajectoryRelation(
    const AgentRelationAnalyzerInput& agent_1,
    const AgentRelationAnalyzerInput& agent_2) {
  const auto& traj_1 = *agent_1.trajectory;
  const auto& traj_2 = *agent_2.trajectory;
  if (traj_1.type() == PT_STATIONARY && traj_2.type() == PT_STATIONARY) {
    return AgentRelationType::ART_NONE;
  }

  if (traj_2.type() == PT_STATIONARY) {
    const auto reversed_relation = AnalyzeTrajectoryRelation(agent_2, agent_1);
    if (reversed_relation == AgentRelationType::ART_NONE) {
      return reversed_relation;
    }
    return reversed_relation == AgentRelationType::ART_PASS
               ? AgentRelationType::ART_YIELD
               : AgentRelationType::ART_PASS;
  }

  const double influence_threshold =
      DecideInfluenceThreshold(*agent_1.object_proto, *agent_2.object_proto);
  if (traj_1.type() == PT_STATIONARY) {
    // If one is stationary, only check if closest distance is within threshold.
    const auto& object_proto = *agent_1.object_proto;
    const Vec2d pos(object_proto.pos());
    const auto& segment_matcher = *agent_2.segments;
    const auto closest_pair_or =
        ClosestDistanceWithinRadius(segment_matcher, pos,
                                    /*radius=*/2.0 * influence_threshold);
    if (!closest_pair_or.ok()) {
      return AgentRelationType::ART_NONE;
    }
    if (closest_pair_or->first < influence_threshold) {
      // Influencing. Object 1 is stationary, always the influencer.
      return AgentRelationType::ART_PASS;
    }
    return AgentRelationType::ART_NONE;
  }

  const PredictedTrajectoryPoint* closest_point_on_traj_1 = nullptr;
  int closest_segment_idx = -1;
  double min_distance = std::numeric_limits<double>::infinity();

  // Use trajectory point on object 1 to check against segment matcher from
  // object 2.
  const auto& segment_matcher = *agent_2.segments;
  for (const auto& point : traj_1.points()) {
    const auto& pos = point.pos();
    // TODO(changqing) : Influence threshold too wide?
    const auto closest_pair_or = ClosestDistanceWithinRadius(
        segment_matcher, pos, /*radius=*/2.0 * influence_threshold);
    if (!closest_pair_or.ok()) {
      // Cannot find matching segment within influence radius just for this
      // point. So continue.
      continue;
    }
    const auto& distance = closest_pair_or->first;
    const auto& segment_idx = closest_pair_or->second;
    if (distance < min_distance) {
      min_distance = distance;
      closest_point_on_traj_1 = &point;
      closest_segment_idx = segment_idx;
    }
  }
  if (closest_point_on_traj_1 == nullptr) {
    // Cannot find point on trajectory 1 close enough to trajectory 2.
    return AgentRelationType::ART_NONE;
  }
  // Segments are built based on predicted trajectory points.
  QCHECK_LT(closest_segment_idx, traj_2.points().size());
  // Try query trajectory point directly using segment_idx.
  const auto& closest_point_on_traj_2 = traj_2.points()[closest_segment_idx];

  const auto t1 = closest_point_on_traj_1->t();
  const auto t2 = closest_point_on_traj_2.t();
  if (t1 < t2) {
    return AgentRelationType::ART_PASS;
  }
  return AgentRelationType::ART_YIELD;
}

std::optional<AvObjMeetingPoints> FindAvObjMeetingPoints(
    const TrajectoryProto& av_trajectory,
    const SegmentMatcherKdtree& av_segment_matcher,
    const ObjectProto& object_proto,
    const PredictedTrajectory& object_trajectory,
    absl::Span<const qcraft::Box2dProto* const> object_shapes) {
  int closest_segment_idx = av_segment_matcher.segments().size();
  const PredictedTrajectoryPoint* closest_point_ptr = nullptr;
  bool closest_segment_found = false;
  VLOG(3) << "---- > Analyze AV with " << object_proto.id() << " trajectory.";
  QCHECK_EQ(object_shapes.size(), object_trajectory.points().size());
  for (int point_idx = 0; point_idx < object_trajectory.points().size();
       ++point_idx) {
    const auto& pt = object_trajectory.points()[point_idx];
    const Box2d object_box(*object_shapes[point_idx]);  // From perception.
    const auto search_radius = object_box.diagonal();
    const auto& pos = pt.pos();
    ASSIGN_OR_CONTINUE(
        const auto pair,
        ClosestDistanceWithinRadius(av_segment_matcher, pos, search_radius));
    const auto& segment_idx = pair.second;
    // Create object box at possible intersection point.
    const Box2d pt_object_box(object_box.half_length(), object_box.half_width(),
                              pt.pos(), pt.theta());
    // Get the corresponding av segment.
    const auto* av_segment = av_segment_matcher.GetSegmentByIndex(segment_idx);
    if (av_segment == nullptr) continue;
    if (segment_idx < closest_segment_idx &&
        pt_object_box.HasOverlap(*av_segment)) {
      closest_segment_found = true;
      closest_segment_idx = segment_idx;
      closest_point_ptr = &pt;
    }
  }
  if (closest_segment_found == false) {
    VLOG(3) << "---- > No true intersection found in time horizon";
    return std::nullopt;
  }
  VLOG(3) << "Close segment found at " << closest_segment_idx << " idx.";
  const auto* av_segment =
      av_segment_matcher.GetSegmentByIndex(closest_segment_idx);
  const auto& av_closest_point =
      FindClosestTrajectoryPointToSegment(av_trajectory, *av_segment);
  return AvObjMeetingPoints(
      {.av_point = av_closest_point, .obj_point = *closest_point_ptr});
}

std::optional<AvObjMeetingPoints> FindAvObjMeetingPointsWithShape(
    const TrajectoryProto& av_trajectory,
    const SegmentMatcherKdtree& av_segment_matcher, const Box2d& av_box,
    const ObjectProto& /*object_proto*/,
    const PredictedTrajectory& object_trajectory,
    absl::Span<const qcraft::Box2dProto* const> object_shapes) {
  // Find AV and Object meeting points considering shapes of the object and AV.
  QCHECK_EQ(object_shapes.size(), object_trajectory.points().size());
  for (int obj_idx = 0; obj_idx < object_trajectory.points().size();
       ++obj_idx) {
    const auto& obj_pt = object_trajectory.points()[obj_idx];
    const Box2d obj_box(*object_shapes[obj_idx]);
    const auto search_radius = 1.0 * (obj_box.diagonal() + av_box.diagonal());
    ASSIGN_OR_CONTINUE(const auto time_ordered_segs,
                       IndexOrderedSegmentWithinRadius(
                           av_segment_matcher, obj_pt.pos(), search_radius));
    const Box2d obj_query_box(obj_box.half_length(), obj_box.half_width(),
                              obj_pt.pos(), obj_pt.theta());
    for (const auto& av_seg_idx : time_ordered_segs) {
      const auto* av_closest_seg =
          av_segment_matcher.GetSegmentByIndex(av_seg_idx);
      if (av_closest_seg == nullptr) continue;
      const auto start_pt_pair = FindClosestTrajectoryPointToPosition(
          av_trajectory, av_closest_seg->start());
      const auto end_pt_pair = FindClosestTrajectoryPointToPosition(
          av_trajectory, av_closest_seg->end());
      if (start_pt_pair.second >= end_pt_pair.second) {
        // Wrong order of segment start and end point trajectory indices.
        continue;
      }
      constexpr double kMinTrajectorySegmentLength = 0.1;
      // Including start and end point.
      constexpr int kSegmentExtrapolationSize = 11;
      const auto start_pt = TrajectoryPoint(start_pt_pair.first);
      const auto end_pt = TrajectoryPoint(end_pt_pair.first);
      const double distance = start_pt.pos().DistanceTo(end_pt.pos());
      const double fine_seg_length =
          std::max(distance / (kSegmentExtrapolationSize - 1),
                   kMinTrajectorySegmentLength);
      const int fine_seg_size =
          static_cast<int>(distance / fine_seg_length) + 1;
      const double alpha = 1.0 / fine_seg_size;
      // Lerp trajectory point on this segment to find shape overlap.
      for (int i = 0; i <= fine_seg_size; ++i) {
        TrajectoryPoint lerped_pt;
        planner::LerpTrajPoint(start_pt, end_pt, std::min(i * alpha, 1.0),
                               &lerped_pt);
        const Box2d fine_av_box =
            Box2d(/*center=*/lerped_pt.pos(), /*heading=*/lerped_pt.theta(),
                  av_box.length(), av_box.width());
        if (fine_av_box.HasOverlap(obj_query_box)) {
          // Finely extrapolated av trajectory point with shape has overlap with
          // object box.
          AvObjMeetingPoints meeting_points;
          lerped_pt.ToProto(&meeting_points.av_point);
          meeting_points.obj_point = obj_pt;
          return meeting_points;
        }
      }
    }
  }
  return std::nullopt;
}

ObjectRelationProto AnalyzeObjectRelation(
    const TrajectoryProto& av_trajectory,
    const SegmentMatcherKdtree& av_segment_matcher,
    const ObjectProto& object_proto,
    const PredictedTrajectory& object_trajectory,
    absl::Span<const qcraft::Box2dProto* const> object_shapes,
    std::optional<Box2d> av_box) {
  ObjectRelationProto relation_proto;
  if (!IsObjectRelationAnalyzeType(object_proto.type())) {
    // Filter some types. Unknown movable will be analyzed.
    relation_proto.set_av_relation(ObjectRelation::OR_VOID);
    relation_proto.set_av_s(std::numeric_limits<double>::max());
    relation_proto.set_object_s(std::numeric_limits<double>::max());
    return relation_proto;
  }

  constexpr double kInteractionMaxTimeWindow = 10.0;  // s.
  const auto av_obj_points =
      av_box.has_value()
          ? FindAvObjMeetingPointsWithShape(av_trajectory, av_segment_matcher,
                                            *av_box, object_proto,
                                            object_trajectory, object_shapes)
          : FindAvObjMeetingPoints(av_trajectory, av_segment_matcher,
                                   object_proto, object_trajectory,
                                   object_shapes);
  if (av_obj_points == std::nullopt) {
    // No meeting points.
    const auto max_time =
        GetAvailableTimeHorizon(av_trajectory, object_trajectory);
    if (max_time < kInteractionMaxTimeWindow) {
      // If available time < 10.0s due to log/sim length, mask this case. Not
      // enough data for analysis.
      relation_proto.set_av_relation(ObjectRelation::OR_VOID);
    } else {
      relation_proto.set_av_relation(ObjectRelation::OR_NO_RELATION);
    }
    relation_proto.set_av_s(std::numeric_limits<double>::max());
    relation_proto.set_object_s(std::numeric_limits<double>::max());
    return relation_proto;
  }
  const ApolloTrajectoryPointProto& av_meeting_point =
      av_obj_points.value().av_point;
  const PredictedTrajectoryPoint& obj_meeting_point =
      av_obj_points.value().obj_point;

  // Valid yield/pass position relation, compare time.
  const auto av_arrive_time = av_meeting_point.relative_time();
  const double object_arrive_time = obj_meeting_point.t();
  if (std::fabs(object_arrive_time - av_arrive_time) >
      kInteractionMaxTimeWindow) {
    relation_proto.set_av_relation(ObjectRelation::OR_NO_RELATION);
    return relation_proto;
  }
  if (object_arrive_time > av_arrive_time) {
    relation_proto.set_av_relation(ObjectRelation::OR_PASS);
    *relation_proto.mutable_av_point() = av_meeting_point;
    *relation_proto.mutable_obj_point() =
        PredictedTrajectoryPointToApolloTrajectoryPointProto(obj_meeting_point);
    FillPathsForObjectRelationInfo(av_trajectory, object_trajectory,
                                   &relation_proto);
    return relation_proto;
  }
  // Don't label objects that are currently behind us to yield.
  if (IsObjectCurrentPositionBehindAVAndFarAway(av_trajectory,
                                                object_trajectory)) {
    relation_proto.set_av_relation(ObjectRelation::OR_NO_RELATION);
    return relation_proto;
  }
  relation_proto.set_av_relation(ObjectRelation::OR_YIELD);
  *relation_proto.mutable_av_point() = av_meeting_point;
  *relation_proto.mutable_obj_point() =
      PredictedTrajectoryPointToApolloTrajectoryPointProto(obj_meeting_point);
  FillPathsForObjectRelationInfo(av_trajectory, object_trajectory,
                                 &relation_proto);
  return relation_proto;
}

std::vector<TrajectoryNode> BatchOnPathAnalyze(
    absl::Span<const AgentRelationAnalyzerInput> inputs) {
  FUNC_QTRACE();
  std::vector<TrajectoryNode> nodes;
  nodes.reserve(inputs.size());
  for (const auto& input : inputs) {
    nodes.push_back(TrajectoryNode({
        .index = TrajectoryNodeIndex(nodes.size()),
        .traj_id = input.traj_id,
    }));
  }
  for (int i = 0; i < nodes.size(); ++i) {
    for (int j = i + 1; j < nodes.size(); ++j) {
      const auto result = OnPathAnalyzer(inputs[i], inputs[j]);
      const auto result_reversed = OnPathAnalyzer(inputs[j], inputs[i]);

      if (result == OnPathAnalyzerType::kOnPath &&
          result_reversed == OnPathAnalyzerType::kNone) {
        nodes[i].reactors.insert(nodes[j].index);
        nodes[j].influencers.insert(nodes[i].index);
        // Update count using set size to avoid failed insertion.
        nodes[j].influencers_count = nodes[j].influencers.size();
      } else if (result == OnPathAnalyzerType::kNone &&
                 result_reversed == OnPathAnalyzerType::kOnPath) {
        nodes[i].influencers.insert(nodes[j].index);
        nodes[i].influencers_count = nodes[i].influencers.size();
        nodes[j].reactors.insert(nodes[i].index);
      }
      // Do not consider both on path case. (interactive)
    }
  }
  return nodes;
}

std::vector<std::vector<TrajectoryNodeIndex>> BuildPriorityGraph(
    std::vector<TrajectoryNode>* ptr_nodes) {
  FUNC_QTRACE();
  std::vector<std::vector<TrajectoryNodeIndex>> results;
  std::vector<TrajectoryNodeIndex> q;
  auto& nodes = *ptr_nodes;
  for (const auto& node : nodes) {
    if (node.influencers_count == 0) {
      q.push_back(node.index);
    }
  }
  int start_idx = 0;
  while (start_idx < q.size()) {
    const int cur_size = q.size() - start_idx;
    std::vector<TrajectoryNodeIndex> output;
    for (int i = 0; i < cur_size; ++i) {
      const auto& node_index = q[start_idx + i];
      const auto& influencing_node = nodes[node_index.value()];
      for (const auto& reactor_idx : influencing_node.reactors) {
        auto& reactor = nodes[reactor_idx.value()];
        reactor.influencers_count--;
        if (reactor.influencers_count == 0) {
          q.push_back(reactor.index);
        }
      }
      output.push_back(influencing_node.index);
    }
    results.push_back(std::move(output));
    start_idx += cur_size;
  }
  return results;
}
}  // namespace prediction
}  // namespace qcraft
