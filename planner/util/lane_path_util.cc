#include "onboard/planner/util/lane_path_util.h"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <queue>
#include <set>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/hash/hash.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/types/span.h"

#include "onboard/base/macros.h"
#include "onboard/container/strong_int.h"
#include "onboard/global/logging.h"
#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/lite/qissue_trans.h"
#include "onboard/maps/lane_path_data.h"
#include "onboard/maps/map_or_die_macros.h"
#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/maps/semantic_map_util.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/util.h"
#include "onboard/planner/util/lane_point_util.h"
#include "onboard/planner/util/planner_semantic_map_util.h"
#include "onboard/planner/util/spatial_search_util.h"
#include "onboard/proto/q_issue.pb.h"
#include "onboard/utils/source_location.h"
#include "onboard/utils/status_macros.h"

namespace qcraft::planner {
namespace {
// TODO(zuowei): consider to use macros to replace this.
mapping::LanePath BuildLanePathFromDataOrReportIssue(
    const mapping::LanePathData& data, const PlannerSemanticMapManager& psmm) {
  auto lane_path_or = BuildLanePathFromData(data, psmm);
  if (UNLIKELY(!lane_path_or.ok())) {
    QISSUEX_WITH_ARGS(
        QIssueSeverity::QIS_FATAL, QIssueType::QIT_BUSINESS,
        QIssueSubType::QIST_SEMANTIC_MAP,
        "Build LanePath From LanePathData failed.",
        absl::StrCat(data.DebugString(), ", loc: ", QCRAFT_LOC.ToString()));
    return mapping::LanePath(psmm.semantic_map_manager(), data);
  }
  return *lane_path_or;
}

mapping::LanePath ConnectLanePathOrReportIssue(
    const mapping::LanePath& lane_path, const mapping::LanePath& other,
    const PlannerSemanticMapManager& psmm, bool fail_return_this,
    double distance_threshold = 0.01) {
  auto lane_path_or =
      ConnectLanePath(lane_path, other, psmm, distance_threshold);
  if (UNLIKELY(!lane_path_or.ok())) {
    QISSUEX_WITH_ARGS(
        QIssueSeverity::QIS_FATAL, QIssueType::QIT_BUSINESS,
        QIssueSubType::QIST_SEMANTIC_MAP, "Connect LanePath failed.",
        absl::StrCat(lane_path.DebugString(), ", ", other.DebugString(),
                     ", loc: ", QCRAFT_LOC.ToString()));
    return fail_return_this ? lane_path : other;
  }
  return *lane_path_or;
}

void CollectWithCurrentSeq(
    const DrivingMapTopo& dm, const mapping::ElementId cur_id,
    std::vector<mapping::ElementId>* so_far_seq,
    std::vector<std::vector<mapping::ElementId>>* all_seqs) {
  const auto* lane = dm.GetLaneById(cur_id);
  if (lane == nullptr) {
    return;
  }
  if (lane->outgoing_lane_ids.empty()) {
    all_seqs->push_back(*so_far_seq);
    return;
  }
  for (const auto& out_id : lane->outgoing_lane_ids) {
    so_far_seq->push_back(out_id);
    CollectWithCurrentSeq(dm, out_id, so_far_seq, all_seqs);
    so_far_seq->pop_back();
  }
}
}  // namespace

absl::StatusOr<mapping::LanePath> BuildLanePathFromData(
    const mapping::LanePathData& data, const PlannerSemanticMapManager& psmm) {
  const int n = data.size();

  std::vector<double> lane_lengths;
  lane_lengths.reserve(n);
  std::vector<double> lane_end_s;
  lane_end_s.reserve(n + 1);
  lane_end_s.push_back(0.0);
  if (n == 1) {
    const auto* lane_info_ptr =
        psmm.FindLaneInfoOrNull(data.lane_ids().front());
    if (lane_info_ptr == nullptr) {
      return absl::NotFoundError(
          absl::StrCat("lane ", data.lane_ids().front(), " not found."));
    }
    lane_lengths.push_back(lane_info_ptr->length());
    lane_end_s.push_back(lane_info_ptr->length() *
                         (data.end_fraction() - data.start_fraction()));
    return mapping::LanePath(data, std::move(lane_end_s),
                             std::move(lane_lengths));
  }
  for (int i = 0; i < n; ++i) {
    const auto id = data.lane_ids()[i];
    const auto* lane_info_ptr = psmm.FindLaneInfoOrNull(id);
    if (lane_info_ptr == nullptr) {
      return absl::NotFoundError(absl::StrCat("lane ", id, " not found."));
    }
    lane_lengths.push_back(lane_info_ptr->length());

    double fraction = 1.0;
    if (i == 0) {
      fraction = data.lane_path_in_forward_direction()
                     ? (1.0 - data.start_fraction())
                     : data.start_fraction();
    } else if (i + 1 == n) {
      fraction = data.lane_path_in_forward_direction()
                     ? data.end_fraction()
                     : (1.0 - data.end_fraction());
    }
    lane_end_s.push_back(lane_end_s.back() +
                         lane_info_ptr->length() * fraction);
  }

  return mapping::LanePath(data, std::move(lane_end_s),
                           std::move(lane_lengths));
}

bool IsLanePathConnectedTo(const mapping::LanePath& lane_path,
                           const mapping::LanePath& other,
                           const PlannerSemanticMapManager& psmm,
                           double distance_threshold) {
  // NOTE: below are copied from lane_path.cc and slightly refactored.
  if (lane_path.end_fraction() == 1.0 && other.start_fraction() == 0.0) {
    // TODO(weijun): call smm util later.
    const auto* lane_info =
        psmm.FindLaneInfoOrNull(lane_path.lane_ids().back());
    if (lane_info == nullptr) {
      return false;
    }
    const auto& outgoing_lanes_ids = lane_info->outgoing_lanes();
    const mapping::ElementId front_id = other.lane_ids().front();
    return std::find_if(outgoing_lanes_ids.begin(), outgoing_lanes_ids.end(),
                        [front_id](mapping::ElementId id) {
                          return id == front_id;
                        }) != outgoing_lanes_ids.end();
  }
  if (lane_path.lane_ids().back() != other.lane_ids().front()) {
    return false;
  }

  const auto* last_lane_info =
      psmm.FindLaneInfoOrNull(lane_path.lane_ids().back());
  if (last_lane_info == nullptr) {
    return false;
  }
  return std::abs(lane_path.end_fraction() - other.start_fraction()) *
             last_lane_info->length() <
         distance_threshold;
}

absl::StatusOr<mapping::LanePath> ConnectLanePath(
    const mapping::LanePath& lane_path, const mapping::LanePath& other,
    const PlannerSemanticMapManager& psmm, double distance_threshold) {
  if (!IsLanePathConnectedTo(lane_path, other, psmm, distance_threshold)) {
    return absl::FailedPreconditionError("lane path not connected.");
  }

  std::vector<mapping::ElementId> ids = lane_path.lane_ids();
  if (lane_path.lane_ids().back() == other.lane_ids().front()) {
    ids.pop_back();
  }
  ids.insert(ids.end(), other.lane_ids().begin(), other.lane_ids().end());

  return BuildLanePathFromData(
      mapping::LanePathData(lane_path.start_fraction(), other.end_fraction(),
                            std::move(ids)),
      psmm);
}

mapping::LanePath BackwardExtendTargetAlignedRouteLanePath(
    const PlannerSemanticMapManager& psmm, bool left,
    const mapping::LanePoint& start_point, const mapping::LanePath& target) {
  SMM_ASSIGN_LANE_OR_RETURN(start_lane, psmm, start_point.lane_id(),
                            mapping::LanePath());
  const auto start_range = mapping::GetNeighborRange(left, target, start_lane);
  const double min_fraction_error = 1.0 / start_lane.length();

  if (start_point.fraction() > start_range.second + min_fraction_error) {
    return mapping::LanePath();
  }

  mapping::LanePath extended_path = BuildLanePathFromDataOrReportIssue(
      mapping::LanePathData(start_range.first, start_point.fraction(),
                            {start_lane.id}),
      psmm);

  while (extended_path.start_fraction() == 0.0) {
    bool extended = false;
    auto lane_id = extended_path.front().lane_id();
    const auto* lane_ptr = psmm.FindLaneInfoOrNull(lane_id);
    if (lane_ptr == nullptr) {
      QISSUEX_WITH_ARGS(QIssueSeverity::QIS_FATAL, QIssueType::QIT_BUSINESS,
                        QIssueSubType::QIST_SEMANTIC_MAP,
                        "Cannot find the map ", std::to_string(lane_id));
      break;
    }
    for (const auto& id : lane_ptr->incoming_lanes()) {
      SMM_ASSIGN_LANE_OR_CONTINUE(in_lane, psmm, id);
      const auto range = mapping::GetNeighborRange(left, target, in_lane);
      if (range.second == 1.0) {
        const auto tmp_lane_path = BuildLanePathFromDataOrReportIssue(
            mapping::LanePathData(range.first, 1.0, {in_lane.id}), psmm);
        extended_path = ConnectLanePathOrReportIssue(
            tmp_lane_path, extended_path, psmm, /*fail_return_this=*/false);
        extended = true;
        break;
      }
    }
    if (!extended) break;
    lane_id = extended_path.front().lane_id();
    lane_ptr = psmm.FindLaneInfoOrNull(lane_id);
    if (lane_ptr == nullptr) {
      QISSUEX_WITH_ARGS(QIssueSeverity::QIS_FATAL, QIssueType::QIT_BUSINESS,
                        QIssueSubType::QIST_SEMANTIC_MAP,
                        "Cannot find the map ", std::to_string(lane_id));
      continue;
    }
  }
  return extended_path;
}

mapping::LanePath BackwardExtendLanePath(
    const PlannerSemanticMapManager& psmm,
    const mapping::LanePath& raw_lane_path, double extend_len,
    const std::function<bool(const mapping::LaneInfo&)>*
        nullable_should_stop_and_avoid_extend) {
  if (extend_len <= 0.0) {
    return raw_lane_path;
  }

  mapping::LanePoint start_lp = raw_lane_path.front();
  mapping::LanePath backward_path =
      BuildLanePathFromDataOrReportIssue(mapping::LanePathData(start_lp), psmm);

  while (extend_len > 0.0) {
    SMM_ASSIGN_LANE_OR_BREAK(lane_info, psmm, start_lp.lane_id());
    if (nullable_should_stop_and_avoid_extend != nullptr &&
        (*nullable_should_stop_and_avoid_extend)(lane_info)) {
      break;
    }
    if (start_lp.fraction() == 0.0) {
      if (lane_info.incoming_lanes().empty()) break;
      if (lane_info.incoming_lanes().size() == 1) {
        start_lp = mapping::LanePoint(lane_info.incoming_lanes().front(), 1.0);
      } else {
        constexpr double kSampleLen = 4.0;  // m.
        const Vec2d origin_pt = lane_info.points_smooth.front();
        const Vec2d next_pt = lane_info.LerpPointFromFraction(
            std::min(1.0, kSampleLen / lane_info.length()));
        const Vec2d heading = (next_pt - origin_pt).normalized();

        double max_projection = std::numeric_limits<double>::lowest();
        mapping::ElementId opt_incoming_id = lane_info.incoming_lanes().front();
        for (int i = 0; i < lane_info.incoming_lanes().size(); ++i) {
          const mapping::ElementId tmp_lane_id = lane_info.incoming_lanes()[i];
          SMM_ASSIGN_LANE_OR_BREAK(tmp_lane_info, psmm, tmp_lane_id);
          const Vec2d prev_pt = tmp_lane_info.LerpPointFromFraction(
              std::max(0.0, 1.0 - kSampleLen / tmp_lane_info.length()));

          const Vec2d tmp_heading = (origin_pt - prev_pt).normalized();

          const double proj = heading.Dot(tmp_heading);

          if (proj > max_projection) {
            max_projection = proj;
            opt_incoming_id = tmp_lane_id;
          }
        }
        start_lp = mapping::LanePoint(opt_incoming_id, 1.0);
      }

    } else {
      const double len = lane_info.length();
      if (len * start_lp.fraction() > extend_len) {
        const double fraction = start_lp.fraction() - extend_len / len;
        const auto tmp_lane_path = BuildLanePathFromDataOrReportIssue(
            mapping::LanePathData(fraction, start_lp.fraction(),
                                  {lane_info.id}),
            psmm);
        auto tmp_lane_path_ext =
            ConnectLanePath(tmp_lane_path, backward_path, psmm);
        if (tmp_lane_path_ext.ok()) {
          backward_path = std::move(tmp_lane_path_ext).value();
        }

        break;
      } else {
        extend_len -= len * start_lp.fraction();
        const auto tmp_lane_path = BuildLanePathFromDataOrReportIssue(
            mapping::LanePathData(0.0, start_lp.fraction(), {lane_info.id}),
            psmm);
        auto tmp_lane_path_ext =
            ConnectLanePath(tmp_lane_path, backward_path, psmm);
        if (tmp_lane_path_ext.ok()) {
          backward_path = std::move(tmp_lane_path_ext).value();
        } else {
          break;
        }

        start_lp = mapping::LanePoint(start_lp.lane_id(), 0.0);
      }
    }
  }

  auto tmp_lane_path_ext = ConnectLanePath(backward_path, raw_lane_path, psmm);

  return tmp_lane_path_ext.ok() ? *tmp_lane_path_ext : raw_lane_path;
}

absl::StatusOr<mapping::LanePath> ForwardExtendLanePathWithMinimumHeadingDiff(
    const PlannerSemanticMapManager& psmm,
    const mapping::LanePath& raw_lane_path, double extend_len,
    bool allow_virtual) {
  if (!allow_virtual) {
    // Check whether raw lane path contains virtual lanes.
    const auto& raw_lane_path_ids = raw_lane_path.lane_ids();
    for (const auto& id : raw_lane_path_ids) {
      const auto* lane_proto = psmm.FindLaneByIdOrNull(id);
      if (lane_proto == nullptr) {
        return absl::InternalError(
            absl::StrFormat("Cannot find LaneProto for %d.", id));
      }
      if (lane_proto->type() == mapping::LaneProto::VIRTUAL ||
          (lane_proto->has_is_virtual() && lane_proto->is_virtual())) {
        return absl::InternalError("Raw lane path contains virtual lanes.");
      }
    }
  }
  if (extend_len <= 0.0) {
    return raw_lane_path;
  }
  mapping::LanePoint start_lp = raw_lane_path.back();
  ASSIGN_OR_RETURN(
      auto forward_path,
      BuildLanePathFromData(mapping::LanePathData(start_lp), psmm));
  constexpr double kEpsilon = 1e-6;  // m.
  while (extend_len > kEpsilon) {
    SMM_ASSIGN_LANE_OR_BREAK(lane_info, psmm, start_lp.lane_id());
    if (!allow_virtual && lane_info.IsVirtual()) break;
    const double len = lane_info.length();
    const double fraction =
        std::min<double>(1.0, extend_len / len + start_lp.fraction());
    ASSIGN_OR_BREAK(
        const auto tmp_lane_path,
        BuildLanePathFromData(mapping::LanePathData(start_lp.fraction(),
                                                    fraction, {lane_info.id}),
                              psmm));
    ASSIGN_OR_BREAK(forward_path,
                    ConnectLanePath(forward_path, tmp_lane_path, psmm));
    // Decrease length to extend if building forward path succeeded.
    extend_len -= tmp_lane_path.length();
    start_lp = forward_path.back();
    if (start_lp.fraction() == 1.0) {
      // Assign new start_lp finding next connecting lane disallow/allow
      // virtual.
      ASSIGN_OR_BREAK(start_lp, FindOutgoingLanePointWithMinimumHeadingDiff(
                                    psmm, start_lp.lane_id(), allow_virtual));
    }
  }
  return ConnectLanePath(raw_lane_path, forward_path, psmm);
}

mapping::LanePath ForwardExtendLanePathWithoutFork(
    const PlannerSemanticMapManager& psmm,
    const mapping::LanePath& raw_lane_path, double extend_len) {
  if (extend_len <= 0.0) {
    return raw_lane_path;
  }

  mapping::LanePoint start_lp = raw_lane_path.back();
  mapping::LanePath forward_path =
      BuildLanePathFromDataOrReportIssue(mapping::LanePathData(start_lp), psmm);
  while (extend_len > 0.0) {
    SMM_ASSIGN_LANE_OR_BREAK(lane_info, psmm, start_lp.lane_id());
    if (start_lp.fraction() == 1.0) {
      if (lane_info.outgoing_lanes().size() != 1) break;
      const auto out_lane_id = lane_info.outgoing_lanes().front();
      start_lp = mapping::LanePoint(out_lane_id, 0.0);

    } else {
      const double len = lane_info.length();
      if (len * (1.0 - start_lp.fraction()) > extend_len) {
        const double fraction = extend_len / len + start_lp.fraction();
        const auto tmp_lane_path = BuildLanePathFromDataOrReportIssue(
            mapping::LanePathData(start_lp.fraction(), fraction,
                                  {lane_info.id}),
            psmm);
        forward_path = ConnectLanePathOrReportIssue(
            forward_path, tmp_lane_path, psmm, /*fail_return_this=*/true);
        break;
      } else {
        extend_len -= len * (1.0 - start_lp.fraction());
        const auto tmp_lane_path = BuildLanePathFromDataOrReportIssue(
            mapping::LanePathData(start_lp.fraction(), 1.0, {lane_info.id}),
            psmm);
        forward_path = ConnectLanePathOrReportIssue(
            forward_path, tmp_lane_path, psmm, /*fail_return_this=*/true);
        start_lp = mapping::LanePoint(start_lp.lane_id(), 1.0);
      }
    }
  }
  return ConnectLanePathOrReportIssue(raw_lane_path, forward_path, psmm,
                                      /*fail_return_this=*/true);
}

absl::StatusOr<mapping::LanePath> FindNearestLanePathFromEgoPose(
    const PoseProto& pose, const PlannerSemanticMapManager& psmm,
    double required_min_length) {
  const auto close_points =
      FindCloseLanePointsToSmoothPointWithHeadingBoundAmongLanesAtLevel(
          psmm.GetLevel(), psmm,
          Vec2d(pose.pos_smooth().x(), pose.pos_smooth().y()), pose.yaw(),
          /*heading_penalty_weight=*/0.0, /*spatial_distance_threshold=*/5.0,
          /*angle_error_threshold=*/M_PI_2);
  if (close_points.empty()) {
    return absl::NotFoundError(
        absl::StrCat("Can not find lane point around pose [",
                     pose.pos_smooth().x(), ",", pose.pos_smooth().y(), "]."));
  }

  return ForwardExtendLanePathWithMinimumHeadingDiff(
      psmm,
      BuildLanePathFromDataOrReportIssue(
          mapping::LanePathData(close_points.front()), psmm),
      required_min_length, /*allow_virtual=*/true);
}

absl::StatusOr<std::vector<mapping::LanePath>> FindNearLanePathsFromEgoPose(
    const PlannerSemanticMapManager& psmm, const Vec2d& pos, double heading,
    double required_min_length, double heading_penalty_weight,
    double distance_threshold, double angle_error_threshold) {
  const auto close_points =
      FindCloseLanePointsToSmoothPointWithHeadingBoundAmongLanesAtLevel(
          psmm.GetLevel(), psmm, pos, heading, heading_penalty_weight,
          distance_threshold, angle_error_threshold);

  std::vector<mapping::LanePath> lane_paths;
  absl::flat_hash_set<mapping::ElementId> lane_ids;
  for (const auto& point : close_points) {
    if (lane_ids.contains(point.lane_id())) continue;
    lane_ids.emplace(point.lane_id());
    ASSIGN_OR_CONTINUE(auto lane_path,
                       ForwardExtendLanePathWithMinimumHeadingDiff(
                           psmm,
                           BuildLanePathFromDataOrReportIssue(
                               mapping::LanePathData(point), psmm),
                           required_min_length, /*allow_virtual=*/true));
    lane_paths.push_back(std::move(lane_path));
  }
  if (lane_paths.empty()) {
    return absl::NotFoundError(absl::StrCat(
        "Can not find lane path around pose [", pos.x(), ",", pos.y(), "]."));
  }
  return lane_paths;
}

Vec2d ArclengthToPos(const PlannerSemanticMapManager& psmm,
                     const mapping::LanePath& lane_path, double s) {
  const auto lane_point = lane_path.ArclengthToLanePoint(s);
  return ComputeLanePointPos(psmm, lane_point);
}

double ArclengthToLerpTheta(const PlannerSemanticMapManager& psmm,
                            const mapping::LanePath& lane_path, double s) {
  return LaneIndexPointToLerpTheta(psmm, lane_path,
                                   lane_path.ArclengthToLaneIndexPoint(s));
}

double LaneIndexPointToLerpTheta(
    const PlannerSemanticMapManager& psmm, const mapping::LanePath& lane_path,
    const mapping::LanePath::LaneIndexPoint& lane_index_point) {
  const int lane_index = lane_index_point.first;
  const double lane_fraction = lane_index_point.second;
  QCHECK_GE(lane_index, 0);
  QCHECK_LT(lane_index, lane_path.size());
  QCHECK_GE(lane_fraction, 0.0);
  QCHECK_LE(lane_fraction, 1.0);

  mapping::LanePoint lane_point(lane_path.lane_ids()[lane_index],
                                lane_fraction);
  double tangent;
  if (lane_index == lane_path.size() - 1) {
    // No successor lane.
    tangent = ComputeLanePointLerpTheta(psmm, lane_point);
  } else {
    const auto succ_lane_id = lane_path.lane_ids()[lane_index + 1];
    tangent = ComputeLanePointLerpThetaWithSuccessorLane(psmm, succ_lane_id,
                                                         lane_point);
  }

  return tangent;
}

std::vector<const mapping::LaneInfo*> GetLanesInfoBreakIfNotFound(
    const PlannerSemanticMapManager& psmm, const mapping::LanePath& lane_path) {
  std::vector<const mapping::LaneInfo*> lanes_info;
  lanes_info.reserve(lane_path.size());
  for (const auto& lane_id : lane_path.lane_ids()) {
    SMM_ASSIGN_LANE_OR_BREAK(lane_info, psmm, lane_id);
    lanes_info.push_back(&lane_info);
  }
  return lanes_info;
}

std::vector<const mapping::LaneInfo*> GetLanesInfoContinueIfNotFound(
    const PlannerSemanticMapManager& psmm, const mapping::LanePath& lane_path) {
  std::vector<const mapping::LaneInfo*> lanes_info;
  lanes_info.reserve(lane_path.size());
  for (const auto& lane_id : lane_path.lane_ids()) {
    SMM_ASSIGN_LANE_OR_CONTINUE(lane_info, psmm, lane_id);
    lanes_info.push_back(&lane_info);
  }
  return lanes_info;
}

absl::StatusOr<mapping::LanePath> TrimTrailingNotFoundLanes(
    const PlannerSemanticMapManager& psmm, const mapping::LanePath& lane_path) {
  if (lane_path.IsEmpty()) {
    return absl::InvalidArgumentError("Input lane path is empty.");
  }

  if (const auto* last_lane_ptr =
          psmm.FindLaneInfoOrNull(lane_path.lane_ids().back());
      last_lane_ptr != nullptr) {
    return lane_path;
  }

  if (const auto* first_lane_ptr =
          psmm.FindLaneByIdOrNull(lane_path.lane_ids().front());
      first_lane_ptr == nullptr) {
    return absl::NotFoundError(absl::StrFormat(
        "Current lane path is not loaded entirely, the lane path is: %s",
        lane_path.DebugString()));
  }

  const auto iter = std::find_if_not(
      lane_path.lane_ids().begin(), lane_path.lane_ids().end(),
      [&psmm](mapping::ElementId lane_id) {
        const auto* lane_ptr = psmm.FindLaneInfoOrNull(lane_id);
        return lane_ptr != nullptr;
      });
  std::vector<mapping::ElementId> new_lane_ids(lane_path.lane_ids().begin(),
                                               iter);
  return BuildLanePathFromData(
      mapping::LanePathData(lane_path.start_fraction(),
                            /*end_fraction=*/1.0, std::move(new_lane_ids)),
      psmm);
}

mapping::LanePath ForwardExtendLanePath(const PlannerSemanticMapManager& psmm,
                                        const mapping::LanePath& raw_lane_path,
                                        double extend_len) {
  if (extend_len <= 0.0) {
    return raw_lane_path;
  }

  mapping::LanePoint start_lp = raw_lane_path.back();
  mapping::LanePath forward_path =
      BuildLanePathFromDataOrReportIssue(mapping::LanePathData(start_lp), psmm);
  while (extend_len > 0.0) {
    const mapping::LaneInfo* lane_info_ptr = nullptr;
    SMM_ASSIGN_LANE_OR_BREAK(lane_info, psmm, start_lp.lane_id());
    lane_info_ptr = &lane_info;
    QCHECK_NOTNULL(lane_info_ptr);
    if (start_lp.fraction() == 1.0) {
      if (lane_info_ptr->outgoing_lanes().empty()) break;
      const auto out_lane_id = lane_info_ptr->outgoing_lanes().front();
      start_lp = mapping::LanePoint(out_lane_id, 0.0);
    } else {
      const double len = lane_info_ptr->length();
      if (len * (1.0 - start_lp.fraction()) > extend_len) {
        const double fraction = extend_len / len + start_lp.fraction();
        const auto tmp_lane_path = BuildLanePathFromDataOrReportIssue(
            mapping::LanePathData(start_lp.fraction(), fraction,
                                  {lane_info_ptr->id}),
            psmm);
        forward_path = ConnectLanePathOrReportIssue(
            forward_path, tmp_lane_path, psmm, /*fail_return_this=*/true);
        break;
      } else {
        extend_len -= len * (1.0 - start_lp.fraction());
        const auto tmp_lane_path = BuildLanePathFromDataOrReportIssue(
            mapping::LanePathData(start_lp.fraction(), 1.0,
                                  {lane_info_ptr->id}),
            psmm);
        forward_path = ConnectLanePathOrReportIssue(
            forward_path, tmp_lane_path, psmm, /*fail_return_this=*/true);
        start_lp = mapping::LanePoint(start_lp.lane_id(), 1.0);
      }
    }
  }

  return ConnectLanePathOrReportIssue(raw_lane_path, forward_path, psmm,
                                      /*fail_return_this=*/true);
}

std::vector<mapping::LanePath> CollectAllLanePathFromStartLane(
    const PlannerSemanticMapManager& psmm, const mapping::LanePath& start_lane,
    double max_lane_length) {
  if (start_lane.length() >= max_lane_length) {
    return {start_lane};
  }

  std::vector<mapping::LanePath> results;
  std::queue<mapping::LanePath> search_queue;
  search_queue.push(start_lane);
  while (!search_queue.empty()) {
    auto lane_path = search_queue.front();
    search_queue.pop();
    SMM_ASSIGN_LANE_OR_CONTINUE(lane_info, psmm, lane_path.lane_ids().back());

    if (lane_info.outgoing_lanes().empty()) {
      results.emplace_back(std::move(lane_path));
      continue;
    }

    for (const auto& out_lane_id : lane_info.outgoing_lanes()) {
      SMM_ASSIGN_LANE_OR_CONTINUE(out_lane_info, psmm, out_lane_id);
      std::vector<mapping::ElementId> new_lane_ids = lane_path.lane_ids();
      new_lane_ids.push_back(out_lane_id);
      if (lane_path.length() + out_lane_info.length() >= max_lane_length) {
        const double end_fraction = std::clamp(
            (max_lane_length - lane_path.length()) / out_lane_info.length(),
            0.0, 1.0);
        results.emplace_back(psmm.semantic_map_manager(),
                             std::move(new_lane_ids),
                             lane_path.front().fraction(), end_fraction);
      } else {
        search_queue.emplace(
            psmm.semantic_map_manager(), std::move(new_lane_ids),
            lane_path.front().fraction(), /*end_fraction=*/1.0);
      }
    }
  }

  return results;
}

absl::StatusOr<std::vector<Vec2d>> SampleLanePathByStep(
    const PlannerSemanticMapManager& psmm, const mapping::LanePath& lane_path,
    double step) {
  if (lane_path.IsEmpty()) {
    return absl::InvalidArgumentError("Empty lane path.");
  }
  if (step <= 0.0) {
    return absl::InvalidArgumentError(absl::StrFormat("Invalid step %f", step));
  }

  const double lane_len = lane_path.length();
  const int n = CeilToInt(lane_len / step) + 1;
  std::vector<Vec2d> sample_points;
  sample_points.reserve(n);

  double sample_s = 0.0;
  for (; sample_s <= lane_len; sample_s += step) {
    sample_points.emplace_back(
        ComputeLanePointPos(psmm, lane_path.ArclengthToLanePoint(sample_s)));
  }

  return sample_points;
}

// TODO(changqing): move to driving map topo util.
std::vector<mapping::LanePath> BuildLanePathsFromDrivingMapTopo(
    const DrivingMapTopo& dm, const PlannerSemanticMapManager& psmm) {
  FUNC_QTRACE();
  std::vector<std::vector<mapping::ElementId>> all_lane_seqs;
  for (const auto& start_id : dm.starting_lane_ids()) {
    std::vector<std::vector<mapping::ElementId>> lane_seqs;
    std::vector<mapping::ElementId> so_far_seq;
    so_far_seq.push_back(start_id);
    CollectWithCurrentSeq(dm, start_id, &so_far_seq, &lane_seqs);
    std::move(lane_seqs.begin(), lane_seqs.end(),
              std::back_inserter(all_lane_seqs));
  }

  std::vector<mapping::LanePath> lane_paths;
  lane_paths.reserve(all_lane_seqs.size());

  for (const auto& lane_seq : all_lane_seqs) {
    const auto* front_lane = dm.GetLaneById(lane_seq.front());
    if (front_lane == nullptr) continue;
    const mapping::LanePathData lane_path_data(front_lane->start_fraction,
                                               /*end_fraction=*/1.0, lane_seq);
    auto lane_path_or = BuildLanePathFromData(lane_path_data, psmm);
    if (lane_path_or.ok()) {
      lane_paths.push_back(std::move(*lane_path_or));
    }
  }
  return lane_paths;
}

std::vector<std::vector<mapping::ElementId>>
FindLanePathSequencesFromStartIdInDrivingMapTopo(const DrivingMapTopo& dm,
                                                 mapping::ElementId start_id) {
  std::vector<std::vector<mapping::ElementId>> lane_seqs;
  std::vector<mapping::ElementId> so_far_seq;
  so_far_seq.push_back(start_id);
  CollectWithCurrentSeq(dm, start_id, &so_far_seq, &lane_seqs);
  return lane_seqs;
}

absl::StatusOr<mapping::LanePath> BuildLanePathFromLaneIdSeqInDrivingMap(
    const DrivingMapTopo& dm, absl::Span<const mapping::ElementId> seq,
    const PlannerSemanticMapManager& psmm, double desire_length,
    double min_length) {
  constexpr double kLengthEpsilon = 0.1;  // m.
  if (seq.empty()) {
    return absl::InternalError("empty sequence.");
  }
  const auto* start_lane = dm.GetLaneById(seq.front());
  if (start_lane == nullptr) {
    return absl::NotFoundError("start lane id not in dm");
  }
  double remain_length = desire_length;
  // Deal with start lane path.
  SMM_ASSIGN_LANE_OR_RETURN(
      start_lane_info, psmm, seq.front(),
      absl::NotFoundError("Can not find start lane in psmm"));
  const auto start_lane_len = start_lane_info.length();
  const auto start_lane_remain_length =
      start_lane_len * (1.0 - start_lane->start_fraction);
  if (start_lane_remain_length > remain_length) {
    // Start lane length enough.
    const auto end_fraction = std::clamp(
        (start_lane_len * start_lane->start_fraction + remain_length) /
            start_lane_len,
        0.0, 1.0);
    mapping::LanePathData lp_data(start_lane->start_fraction, end_fraction,
                                  {seq.front()});
    return BuildLanePathFromData(lp_data, psmm);
  }
  // Build and connect lane paths iteratively.
  // raw start lane path.
  mapping::LanePath start_lane_path(
      psmm.semantic_map_manager(),
      mapping::LanePoint(seq.front(), start_lane->start_fraction));
  mapping::LanePoint start_lp = start_lane_path.back();
  remain_length -= start_lane_path.length();
  ASSIGN_OR_RETURN(
      auto forward_path,
      BuildLanePathFromData(mapping::LanePathData(start_lp), psmm));
  int seq_idx = 0;  // First seq has been resolved.
  while (remain_length > kLengthEpsilon) {
    SMM_ASSIGN_LANE_OR_BREAK(lane_info, psmm, start_lp.lane_id());
    const auto cur_lane_length = lane_info.length();
    const double fraction = std::min<double>(
        1.0, remain_length / cur_lane_length + start_lp.fraction());
    ASSIGN_OR_BREAK(
        const auto tmp_lane_path,
        BuildLanePathFromData(mapping::LanePathData(start_lp.fraction(),
                                                    fraction, {lane_info.id}),
                              psmm));
    ASSIGN_OR_BREAK(forward_path,
                    ConnectLanePath(forward_path, tmp_lane_path, psmm));
    remain_length -= tmp_lane_path.length();
    start_lp = forward_path.back();
    if (start_lp.fraction() == 1.0) {
      seq_idx++;
      if (seq_idx >= seq.size()) {
        break;
      }
      // Get the start point of next lane.
      start_lp = mapping::LanePoint(seq[seq_idx], /*fraction=*/0.0);
    }
  }

  // Check built lane path length = desire length.
  const auto built_length = desire_length - remain_length;
  if (built_length < min_length - kLengthEpsilon) {
    return absl::NotFoundError(
        "Can not build lane path with enough desire length from current "
        "sequence.");
  }
  return ConnectLanePath(start_lane_path, forward_path, psmm);
}

std::vector<mapping::LanePath>
BuildAllLanePathsFromStartIdInDrivingMapTopoWithDesireLength(
    const DrivingMapTopo& dm, const PlannerSemanticMapManager& psmm,
    mapping::ElementId start_id, double desire_length, double min_length) {
  const auto lane_seqs =
      FindLanePathSequencesFromStartIdInDrivingMapTopo(dm, start_id);
  auto lp_comp = [](const mapping::LanePath& lp1,
                    const mapping::LanePath& lp2) { return lp1 != lp2; };
  std::set<mapping::LanePath,
           bool (*)(const mapping::LanePath&, const mapping::LanePath&)>
      lane_paths(lp_comp);  // Might have duplicated results.
  for (const auto& seq : lane_seqs) {
    auto lp_or = BuildLanePathFromLaneIdSeqInDrivingMap(
        dm, absl::MakeSpan(seq), psmm, desire_length, min_length);
    if (lp_or.ok()) {
      lane_paths.insert(std::move(*lp_or));
    }
  }
  std::vector<mapping::LanePath> lps_vec;
  lps_vec.reserve(lane_paths.size());
  std::move(lane_paths.begin(), lane_paths.end(), std::back_inserter(lps_vec));
  return lps_vec;
}

}  // namespace qcraft::planner
