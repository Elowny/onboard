#include "onboard/planner/composite_lane_path.h"

#include <cmath>
#include <limits>
#include <ostream>

#include "onboard/lite/logging.h"
#include "onboard/maps/map_or_die_macros.h"
#include "onboard/maps/semantic_map_manager.h"

namespace qcraft {
namespace planner {

void CompositeLanePath::LaneIndexPoint::ToProto(
    CompositeLanePathProto::LaneIndexPointProto* proto) const {
  index().ToProto(proto->mutable_index());
  proto->set_lane_fraction(lane_fraction());
}

void CompositeLanePath::LaneIndexPoint::FromProto(
    const CompositeLanePathProto::LaneIndexPointProto& proto) {
  index_.FromProto(proto.index());
  lane_fraction_ = proto.lane_fraction();
}

void CompositeLanePath::Build() {
  lane_path_end_s_.clear();
  lane_path_end_s_.reserve(num_lane_paths() + 1);
  lane_path_end_s_.push_back(0.0);
  cumulative_lane_path_sizes_.clear();
  cumulative_lane_path_sizes_.reserve(num_lane_paths() + 1);
  cumulative_lane_path_sizes_.push_back(0);
  for (int i = 0; i < num_lane_paths(); ++i) {
    const double s =
        lane_path_end_s_.back() +
        (lane_paths_[i].length() - GetLanePathLengthBeforeTransition(i) -
         GetLanePathLengthAfterTransition(i));
    VLOG(3) << i << " " << lane_paths_[i].length() << " "
            << GetLanePathLengthBeforeTransition(i) << " "
            << GetLanePathLengthAfterTransition(i) << " " << s;
    lane_path_end_s_.push_back(s);
    cumulative_lane_path_sizes_.push_back(cumulative_lane_path_sizes_.back() +
                                          lane_paths_[i].size());
  }
  QCHECK_EQ(num_lane_paths() + 1, lane_path_end_s_.size());

  lane_path_transition_points_.clear();
  lane_path_transition_points_.reserve(num_lane_paths());
  for (int i = 0; i < num_lane_paths(); ++i) {
    LanePath& lane_path = lane_paths_[i];
    const LanePath::LaneIndexPoint entry = lane_path.ArclengthToLaneIndexPoint(
        GetLanePathLengthBeforeTransition(i));
    const LanePath::LaneIndexPoint exit = lane_path.ArclengthToLaneIndexPoint(
        lane_path.length() - GetLanePathLengthAfterTransition(i));
    lane_path_transition_points_.emplace_back(entry, exit);
  }

  lane_ids_.clear();
  lane_ids_.reserve(end() - begin());
  for (auto it = begin(); it != end(); ++it) {
    lane_ids_.push_back((*it).lane_id);
  }
  QCHECK_EQ(lane_ids_.size(), end() - begin());
}

void CompositeLanePath::Clear() {
  lane_paths_.clear();
  transitions_.clear();
  lane_path_end_s_.clear();
  lane_path_transition_points_.clear();
  lane_ids_.clear();
  cumulative_lane_path_sizes_.clear();
}

CompositeLanePath::CompositeIndex
CompositeLanePath::OverallIndexToCompositeIndex(int overall_lane_index) const {
  QCHECK_GE(cumulative_lane_path_sizes_.size(), 2);
  QCHECK_GE(overall_lane_index, 0);
  QCHECK_LT(overall_lane_index, size());
  const int lane_path_index =
      std::upper_bound(cumulative_lane_path_sizes_.begin(),
                       cumulative_lane_path_sizes_.end(), overall_lane_index) -
      cumulative_lane_path_sizes_.begin() - 1;
  QCHECK_GE(lane_path_index, 0);
  QCHECK_LT(lane_path_index, num_lane_paths());
  return {lane_path_index,
          overall_lane_index - cumulative_lane_path_sizes_[lane_path_index]};
}

int CompositeLanePath::CompositeIndexToOverallIndex(
    CompositeIndex composite_index) const {
  return cumulative_lane_path_sizes_[composite_index.lane_path_index] +
         composite_index.lane_index;
}

int CompositeLanePath::ArclengthToLanePathIndex(double s) const {
  QCHECK_GE(lane_path_end_s_.size(), 2);
  if (s >= length()) return num_lane_paths() - 1;
  if (s <= 0.0) return 0;
  const int index =
      std::upper_bound(lane_path_end_s_.begin(), lane_path_end_s_.end(), s) -
      lane_path_end_s_.begin() - 1;
  QCHECK_GE(index, 0);
  QCHECK_LT(index, num_lane_paths());
  return index;
}

CompositeLanePath::LanePathPoint CompositeLanePath::ArclengthToLanePathPoint(
    double s) const {
  QCHECK_GE(lane_path_end_s_.size(), 2);
  if (s >= length()) return {num_lane_paths() - 1, lane_paths_.back().length()};
  if (s <= 0.0) return {0, 0.0};
  const int lane_path_index = ArclengthToLanePathIndex(s);
  QCHECK_GE(s, lane_path_end_s_[lane_path_index]);
  QCHECK_LE(s, lane_path_end_s_[lane_path_index + 1]);
  return {lane_path_index,
          s - lane_path_end_s_[lane_path_index] +
              GetLanePathLengthBeforeTransition(lane_path_index)};
}

CompositeLanePath CompositeLanePath::AfterCompositeIndexAndLanePoint(
    const SemanticMapManager* semantic_map_manager,
    const CompositeIndex& composite_index,
    const mapping::LanePoint& lane_point) const {
  const auto lane_path_index = composite_index.lane_path_index;
  QCHECK_GE(lane_path_index, 0);
  QCHECK_LT(lane_path_index, lane_paths_.size());

  std::vector<mapping::LanePath> lane_paths(
      lane_paths_.begin() + lane_path_index, lane_paths_.end());
  std::vector<mapping::ElementId> new_lane_ids(
      lane_paths.front().lane_ids().begin() + composite_index.lane_index,
      lane_paths.front().lane_ids().end());
  lane_paths.front() = mapping::LanePath(
      semantic_map_manager, std::move(new_lane_ids), lane_point.fraction(),
      lane_paths.front().end_fraction());
  std::vector<TransitionInfo> transitions(
      transitions_.begin() + lane_path_index, transitions_.end());
  // TODO(xiang): Should not use CompositeLanePath ctor with SemanticMap, but
  // assign directly
  return CompositeLanePath(std::move(lane_paths), std::move(transitions));
}

CompositeLanePath::LaneIndexPoint CompositeLanePath::ArclengthToLaneIndexPoint(
    double s) const {
  const LanePathPoint lane_path_point = ArclengthToLanePathPoint(s);
  const LanePath::LaneIndexPoint lane_index_point =
      lane_paths_[lane_path_point.index()].ArclengthToLaneIndexPoint(
          lane_path_point.s());
  return {{lane_path_point.index(), lane_index_point.first},
          lane_index_point.second};
}

double CompositeLanePath::LanePathIndexToArclength(int lane_path_index) const {
  QCHECK_LT(lane_path_index, num_lane_paths());
  return lane_path_end_s_[lane_path_index];  // Lane start arclength.
}

double CompositeLanePath::LanePathPointToArclength(
    LanePathPoint lane_path_point) const {
  const int lane_path_index = lane_path_point.index();
  const double s_in_lane_path = lane_path_point.s();
  QCHECK_GE(lane_path_index, 0);
  QCHECK_LT(lane_path_index, num_lane_paths());
  return lane_path_end_s_[lane_path_index] +
         (s_in_lane_path - GetLanePathLengthBeforeTransition(lane_path_index));
}

double CompositeLanePath::LaneIndexPointToArclength(
    const LaneIndexPoint& lane_index_point) const {
  const CompositeIndex& composite_index = lane_index_point.index();
  const double lane_fraction = lane_index_point.lane_fraction();
  QCHECK_GE(composite_index.lane_path_index, 0);
  QCHECK_LT(composite_index.lane_path_index, num_lane_paths());
  QCHECK_GE(lane_fraction, 0.0);
  QCHECK_LE(lane_fraction, 1.0);
  const double s_in_lane_path =
      lane_paths_[composite_index.lane_path_index].LaneIndexPointToArclength(
          composite_index.lane_index, lane_fraction);
  return LanePathPointToArclength(
      {composite_index.lane_path_index, s_in_lane_path});
}

mapping::LanePoint CompositeLanePath::ArclengthToLanePoint(double s) const {
  const auto lane_path_point = ArclengthToLanePathPoint(s);
  return lane_paths_[lane_path_point.index()].ArclengthToLanePoint(
      lane_path_point.s());
}

double CompositeLanePath::FirstOccurrenceOfLanePointToArclength(
    LanePoint point) const {
  return LaneIndexPointToArclength(
      FirstOccurrenceOfLanePointToLaneIndexPoint(point));
}

CompositeLanePath::LanePathPoint
CompositeLanePath::FirstOccurrenceOfLanePointToLanePathPoint(
    LanePoint point) const {
  for (int i = 0; i < num_lane_paths(); ++i) {
    const LanePath& lane_path = lane_paths_[i];
    LanePath::LaneIndexPoint lane_index_point;
    if (!lane_path.ContainsLanePoint(point, &lane_index_point,
                                     /*forward_search=*/true))
      continue;
    const double s = lane_path.LaneIndexPointToArclength(lane_index_point);
    QCHECK(!std::isinf(s));
    if (s < GetLanePathLengthBeforeTransition(i)) continue;
    if (s > lane_path.length() - GetLanePathLengthAfterTransition(i)) continue;
    return {i, s};
  }
  QLOG(FATAL) << "Lane point " << point.DebugString()
              << " is not on composite lane path " << DebugString();
  return {-1, std::numeric_limits<double>::infinity()};
}

CompositeLanePath::LanePathPoint
CompositeLanePath::LastOccurrenceOfLanePointToLanePathPoint(
    LanePoint point) const {
  for (int i = num_lane_paths() - 1; i >= 0; --i) {
    const LanePath& lane_path = lane_paths_[i];
    LanePath::LaneIndexPoint lane_index_point;
    if (!lane_path.ContainsLanePoint(point, &lane_index_point,
                                     /*forward_search=*/false))
      continue;
    const double s = lane_path.LaneIndexPointToArclength(lane_index_point);
    QCHECK(!std::isinf(s));
    if (s < GetLanePathLengthBeforeTransition(i)) continue;
    if (s > lane_path.length() - GetLanePathLengthAfterTransition(i)) continue;
    return {i, s};
  }
  QLOG(FATAL) << "Lane point " << point.DebugString()
              << " is not on composite lane path " << DebugString();
  return {-1, std::numeric_limits<double>::infinity()};
}

CompositeLanePath::LaneIndexPoint
CompositeLanePath::FirstOccurrenceOfLanePointToLaneIndexPoint(
    LanePoint point) const {
  LaneIndexPoint lip;
  if (ContainsLanePoint(point, &lip, /*forward_search=*/true)) return lip;
  QLOG(FATAL) << "Lane point " << point.DebugString()
              << " is not on composite lane path " << DebugString();
  return {composite_index_end(), std::numeric_limits<double>::infinity()};
}

CompositeLanePath::LaneIndexPoint
CompositeLanePath::LastOccurrenceOfLanePointToLaneIndexPoint(
    LanePoint point) const {
  LaneIndexPoint lip;
  if (ContainsLanePoint(point, &lip, /*forward_search=*/false)) return lip;
  QLOG(FATAL) << "Lane point " << point.DebugString()
              << " is not on composite lane path " << DebugString();
  return {composite_index_end(), std::numeric_limits<double>::infinity()};
}

bool CompositeLanePath::ContainsLanePoint(
    LanePoint point, LaneIndexPoint* first_encounter_of_lane_index_point,
    bool forward_search) const {
  const int size = num_lane_paths();
  if (size == 0) return false;
  const int search_start = forward_search ? 0 : size - 1;
  const int search_end = forward_search ? size : -1;
  const int search_inc = forward_search ? 1 : -1;
  for (int i = search_start; i != search_end; i += search_inc) {
    const LanePath& lane_path = lane_paths_[i];
    LanePath::LaneIndexPoint lip;
    if (!lane_path.ContainsLanePoint(point, &lip, forward_search)) continue;
    const double s = lane_path.LaneIndexPointToArclength(lip);
    QCHECK(!std::isinf(s));
    if (s < GetLanePathLengthBeforeTransition(i)) continue;
    if (s > lane_path.length() - GetLanePathLengthAfterTransition(i)) continue;
    if (first_encounter_of_lane_index_point != nullptr) {
      *first_encounter_of_lane_index_point = {{i, lip.first}, lip.second};
    }
    return true;
  }
  return false;
}

CompositeLanePath CompositeLanePath::BeforeFirstOccurrenceOfLanePoint(
    LanePoint point) const {
  return BeforeLanePathPoint(FirstOccurrenceOfLanePointToLanePathPoint(point));
}

CompositeLanePath CompositeLanePath::BeforeLastOccurrenceOfLanePoint(
    LanePoint point) const {
  return BeforeLanePathPoint(LastOccurrenceOfLanePointToLanePathPoint(point));
}

CompositeLanePath CompositeLanePath::AfterFirstOccurrenceOfLanePoint(
    LanePoint point) const {
  return AfterLanePathPoint(FirstOccurrenceOfLanePointToLanePathPoint(point));
}

CompositeLanePath CompositeLanePath::BeforeArclength(double s) const {
  return BeforeLanePathPoint(ArclengthToLanePathPoint(s));
}

CompositeLanePath CompositeLanePath::AfterArclength(double s) const {
  return AfterLanePathPoint(ArclengthToLanePathPoint(s));
}

CompositeLanePath CompositeLanePath::BeforeLanePathPoint(
    LanePathPoint lane_path_point) const {
  const int lane_path_index = lane_path_point.index();
  QCHECK_GE(lane_path_index, 0);
  QCHECK_LT(lane_path_index, num_lane_paths());
  std::vector<LanePath> lane_paths = lane_paths_;
  lane_paths.erase(lane_paths.begin() + lane_path_index + 1, lane_paths.end());
  lane_paths.back() = lane_paths.back().BeforeArclength(lane_path_point.s());
  std::vector<TransitionInfo> transitions = transitions_;
  transitions.erase(transitions.begin() + lane_path_index, transitions.end());
  return CompositeLanePath(std::move(lane_paths), std::move(transitions));
}

CompositeLanePath CompositeLanePath::AfterLanePathPoint(
    LanePathPoint lane_path_point) const {
  const int lane_path_index = lane_path_point.index();
  QCHECK_GE(lane_path_index, 0);
  QCHECK_LT(lane_path_index, num_lane_paths());
  std::vector<LanePath> lane_paths = lane_paths_;
  lane_paths.erase(lane_paths.begin(), lane_paths.begin() + lane_path_index);
  lane_paths.front() = lane_paths.front().AfterArclength(lane_path_point.s());
  std::vector<TransitionInfo> transitions = transitions_;
  transitions.erase(transitions.begin(), transitions.begin() + lane_path_index);
  return CompositeLanePath(std::move(lane_paths), std::move(transitions));
}

CompositeLanePath CompositeLanePath::BeforeLaneIndexPoint(
    LaneIndexPoint lane_index_point) const {
  return BeforeLanePathPoint(
      ArclengthToLanePathPoint(LaneIndexPointToArclength(lane_index_point)));
}

CompositeLanePath CompositeLanePath::Compose(
    const CompositeLanePath& subsequent_path,
    const TransitionInfo& transition) const {
  std::vector<LanePath> lane_paths = lane_paths_;
  lane_paths.insert(lane_paths.end(), subsequent_path.lane_paths().begin(),
                    subsequent_path.lane_paths().end());
  std::vector<TransitionInfo> transitions = transitions_;
  transitions.push_back(transition);
  transitions.insert(transitions.end(), subsequent_path.transitions().begin(),
                     subsequent_path.transitions().end());
  return CompositeLanePath(std::move(lane_paths), std::move(transitions));
}

void CompositeLanePath::ToProto(CompositeLanePathProto* proto) const {
  proto->Clear();
  for (const auto& lane_path : lane_paths_) {
    lane_path.ToProto(proto->add_lane_paths());
  }
  for (const auto& transition : transitions_) {
    auto* transition_proto = proto->add_transitions();
    transition_proto->set_overlap_length(transition.overlap_length);
    transition_proto->set_transition_point_fraction(
        transition.transition_point_fraction);
    transition_proto->set_lc_left(transition.lc_left);

    transition_proto->set_optimized(transition.optimized);
    transition_proto->set_lc_section_id(transition.lc_section_id.value());
  }
}

template <typename SmmType>
void RouteSectionSequenceToClp(const SmmType& semantic_map_manager,
                               RouteSectionSequenceProto* section_seq_proto,
                               const CompositeLanePath* clp) {
  section_seq_proto->Clear();
  if (clp->size() == 0) {
    return;
  }
  const double start_fraction = clp->front().fraction();
  const double end_fraction = clp->back().fraction();
  section_seq_proto->mutable_section_id()->Reserve(clp->size());
  auto last_section_id = mapping::kInvalidSectionId.value();
  for (const auto& lane_path : clp->lane_paths()) {
    for (const auto& lane_id : lane_path.lane_ids()) {
      SMM_ASSIGN_LANE_OR_CONTINUE(lane, semantic_map_manager, lane_id);
      const auto* lane_ptr = &lane;
      if (lane_ptr->Proto().section_id() != last_section_id) {
        last_section_id = lane_ptr->Proto().section_id();
        section_seq_proto->add_section_id(last_section_id);
      }
    }
  }
  section_seq_proto->set_start_fraction(start_fraction);
  section_seq_proto->set_end_fraction(end_fraction);
}

void CompositeLanePath::ToRouteSectionSequenceProto(
    const SemanticMapManager& semantic_map_manager,
    RouteSectionSequenceProto* section_seq_proto) const {
  RouteSectionSequenceToClp(semantic_map_manager, section_seq_proto, this);
}

void CompositeLanePath::ToRouteSectionSequenceProto(
    const mapping::v2::SemanticMapManager& semantic_map_manager,
    RouteSectionSequenceProto* section_seq_proto) const {
  RouteSectionSequenceToClp(semantic_map_manager, section_seq_proto, this);
}

CompositeLanePath::CompositeIndex CompositeLanePath::IncrementCompositeIndex(
    CompositeIndex composite_index, int inc) const {
  if (inc < 0) return DecrementCompositeIndex(composite_index, -inc);
  int exit_index =
      LanePathExitIndexPoint(composite_index.lane_path_index).first;
  while (composite_index.lane_index + inc > exit_index) {
    inc -= exit_index - composite_index.lane_index + 1;
    ++composite_index.lane_path_index;
    if (composite_index.lane_path_index == num_lane_paths()) {
      // end() reached.
      composite_index.lane_index = 0;
      break;
    }
    composite_index.lane_index =
        LanePathEntryIndexPoint(composite_index.lane_path_index).first;
    exit_index = LanePathExitIndexPoint(composite_index.lane_path_index).first;
  }
  composite_index.lane_index += inc;
  return composite_index;
}

CompositeLanePath::CompositeIndex CompositeLanePath::DecrementCompositeIndex(
    CompositeIndex composite_index, int dec) const {
  if (dec < 0) return IncrementCompositeIndex(composite_index, -dec);
  int entry_index =
      LanePathEntryIndexPoint(composite_index.lane_path_index).first;
  while (composite_index.lane_index - dec < entry_index) {
    dec -= composite_index.lane_index - entry_index + 1;
    --composite_index.lane_path_index;
    if (composite_index.lane_path_index < 0) {
      // begin() exceeded.
      composite_index.lane_path_index = num_lane_paths();
      composite_index.lane_index = 0;
      break;
    }
    composite_index.lane_index =
        LanePathExitIndexPoint(composite_index.lane_path_index).first;
    entry_index =
        LanePathEntryIndexPoint(composite_index.lane_path_index).first;
  }
  composite_index.lane_index -= dec;
  return composite_index;
}

// Signed distance from index0 to index1 (positive if index1 > index0).
int CompositeLanePath::CompositeIndexDist(CompositeIndex index0,
                                          CompositeIndex index1) const {
  if (index1 < index0) return -CompositeIndexDist(index1, index0);
  int inc = 0;
  while (index0.lane_path_index < index1.lane_path_index) {
    inc += LanePathExitIndexPoint(index0.lane_path_index).first -
           index0.lane_index + 1;
    ++index0.lane_path_index;
    index0.lane_index =
        index0.lane_path_index < num_lane_paths()
            ? LanePathEntryIndexPoint(index0.lane_path_index).first
            : 0;
  }
  inc += index1.lane_index - index0.lane_index;
  return inc;
}

mapping::LanePath::LaneSegment
CompositeLanePath::lane_segment_in_composite_lane_path(
    CompositeIndex composite_index) const {
  DCHECK_GE(composite_index.lane_path_index, 0);
  DCHECK_LT(composite_index.lane_path_index, num_lane_paths());
  const auto entry = LanePathEntryIndexPoint(composite_index.lane_path_index);
  const auto exit = LanePathExitIndexPoint(composite_index.lane_path_index);
  DCHECK_GE(composite_index.lane_index, entry.first);
  DCHECK_LE(composite_index.lane_index, exit.first);
  const auto& lane_path = lane_paths()[composite_index.lane_path_index];
  return mapping::LanePath::LaneSegment(
      CompositeIndexToOverallIndex(composite_index),
      lane_path.lane_id(composite_index.lane_index),
      composite_index.lane_index == entry.first ? entry.second : 0.0,
      composite_index.lane_index == exit.first ? exit.second : 1.0,
      lane_path_end_s_[composite_index.lane_path_index] +
          lane_path.lane_segment(composite_index.lane_index).start_s,
      lane_path_end_s_[composite_index.lane_path_index] +
          lane_path.lane_segment(composite_index.lane_index).end_s);
}

std::string CompositeLanePath::DebugString() const {
  CompositeLanePathProto proto;
  ToProto(&proto);
  return proto.DebugString();
}

std::string CompositeLanePath::ShortDebugString() const {
  CompositeLanePathProto proto;
  ToProto(&proto);
  return proto.ShortDebugString();
}

CompositeLanePathProto AfterCompositeIndexAndLanePoint(
    const CompositeLanePathProto& composite_lane_path,
    const CompositeLanePathProto::CompositeIndexProto& composite_index,
    const mapping::LanePointProto& lane_point) {
  CompositeLanePathProto proto;
  const auto& lane_paths = composite_lane_path.lane_paths();
  const auto lane_path_index = composite_index.lane_path_index();
  QCHECK_GE(lane_path_index, 0);
  QCHECK_LT(lane_path_index, lane_paths.size());

  *proto.mutable_lane_paths() =
      google::protobuf::RepeatedPtrField<mapping::LanePathProto>(
          lane_paths.cbegin() + lane_path_index, lane_paths.cend());
  auto front = proto.mutable_lane_paths()->begin();
  front->mutable_lane_ids()->erase(
      front->lane_ids().begin(),
      front->lane_ids().begin() + composite_index.lane_index());
  front->set_start_fraction(lane_point.fraction());
  *proto.mutable_transitions() = google::protobuf::RepeatedPtrField<
      CompositeLanePathProto::TransitionProto>(
      composite_lane_path.transitions().cbegin() + lane_path_index,
      composite_lane_path.transitions().cend());
  return proto;
}

}  // namespace planner
}  // namespace qcraft
