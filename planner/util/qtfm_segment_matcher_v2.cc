#include "onboard/planner/util/qtfm_segment_matcher_v2.h"

#include <float.h>

#include <cmath>
#include <limits>
#include <ostream>
#include <tuple>

// IWYU pragma: no_include "Eigen/Core"

#include "absl/strings/str_format.h"

#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/math/geometry/circle2d.h"
#include "onboard/math/geometry/grid_frame_util.h"
#include "onboard/math/geometry/intersection_util.h"
#include "onboard/math/util.h"
#include "onboard/planner/util/qtfm_segment_matcher_util.h"
#include "onboard/vis/common/color.h"

namespace qcraft::planner {

namespace {

std::optional<Vec2d> LineIntersection(const Segment2d& l0,
                                      const Segment2d& l1) {
  Vec2d res;
  bool is_found = FindIntersectionBetweenLinesWithTangents(
      l0.start(), l0.unit_direction(), l1.start(), l1.unit_direction(), &res);
  if (is_found) {
    return res;
  } else {
    return {};
  }
}

// Get part of the impact region of one segment.
std::optional<std::vector<Vec2d>> GetTrapezoid(const Segment2d& back_div,
                                               const Segment2d& front_div,
                                               const Segment2d& segment,
                                               double cutoff_distance) {
  // Find the intersection if exist.
  std::optional<Vec2d> intersection = LineIntersection(back_div, front_div);
  if (intersection) {
    if (std::abs(segment.ProductOntoUnit(*intersection)) > cutoff_distance) {
      intersection.reset();
    }
  }

  // Vertices.
  Segment2d left_line = segment;
  left_line.Shift(segment.unit_direction().Perp() * cutoff_distance);
  Segment2d right_line = segment;
  right_line.Shift(-segment.unit_direction().Perp() * cutoff_distance);

  const std::optional<Vec2d> left_front =
      LineIntersection(left_line, front_div);
  const std::optional<Vec2d> left_back = LineIntersection(left_line, back_div);
  const std::optional<Vec2d> right_front =
      LineIntersection(right_line, front_div);
  const std::optional<Vec2d> right_back =
      LineIntersection(right_line, back_div);

  if (!left_front || !left_back || !right_front || !right_back) {
    // Corner case.
    return {};
  }

  auto is_far = [&segment, &cutoff_distance](const Vec2d& point) {
    constexpr int kGain = 9;
    return segment.DistanceSquareTo(point) > kGain * Sqr(cutoff_distance);
  };
  if (is_far(*left_front) || is_far(*left_back) || is_far(*right_front) ||
      is_far(*right_back)) {
    // Corner case.
    return {};
  }

  std::vector<Vec2d> ccw_points;
  constexpr int kMaxNumVertices = 4;
  ccw_points.reserve(kMaxNumVertices);

  if (segment.ProjectOntoUnit(*left_front) <
      segment.ProjectOntoUnit(*left_back)) {
    if (!intersection) {
      return {};
    }
    ccw_points.push_back(*intersection);
  } else {
    ccw_points.push_back(*left_front);
    ccw_points.push_back(*left_back);
  }

  if (segment.ProjectOntoUnit(*right_front) <
      segment.ProjectOntoUnit(*right_back)) {
    if (!intersection) {
      return {};
    }
    ccw_points.push_back(*intersection);
  } else {
    ccw_points.push_back(*right_back);
    ccw_points.push_back(*right_front);
  }

  if (ccw_points.size() > 2) {
    return ccw_points;
  } else {
    return {};
  }
}

}  // namespace

QtfmSegmentMatcherV2::QtfmSegmentMatcherV2(
    Config config, std::vector<Segment2d> raw_segments,
    std::vector<int> raw_start_segment_ids)
    : config_(config),
      raw_segments_(std::move(raw_segments)),
      raw_start_segment_ids_(std::move(raw_start_segment_ids)) {
  QCHECK(config_.IsValid());
  QCHECK_LT(config_.qtfm_depth, SegmentMap::kMaxDepth)
      << "config.qtfm_depth is too big.";

  // Transfer no-divisible segments to divisble ones.
  std::vector<int> start_segment_ids;
  MakeSegmentsDivisible(raw_segments_, raw_start_segment_ids_, &segments_,
                        &start_segment_ids, &segment_id_to_raw_id_);

  // Start segment table.
  is_start_segment_.resize(segments_.size(), 0);
  for (const auto& start_id : start_segment_ids) {
    QCHECK_LT(start_id, segments_.size());
    QCHECK_GE(start_id, 0);
    is_start_segment_[start_id] = 1;
  }

  if (!is_start_segment_.empty()) {
    QCHECK(is_start_segment_[0]);
  }

  // Pre-process segments.
  division_line_with_prev_.reserve(segments_.size());
  for (int i = 0; i < segments_.size(); ++i) {
    if (is_start_segment_[i]) {
      division_line_with_prev_.emplace_back();
    } else {
      const Segment2d& prev_seg = segments_[i - 1];
      const Segment2d& cur_seg = segments_[i];

      QCHECK(IsDivisible(prev_seg, cur_seg))
          << prev_seg.DebugStringFullPrecision() << " "
          << cur_seg.DebugStringFullPrecision() << " "
          << (cur_seg.start() - prev_seg.end()).DebugStringFullPrecision()
          << " " << i << " " << DebugStringFullPrecision();
      division_line_with_prev_.push_back(GetDivisionLine(prev_seg, cur_seg));
    }
  }

  // Initialize grid.
  const double grid_unit = (1 << (config_.qtfm_depth - 1)) * config_.resolution;
  const double x0 = FloorWithBase(config_.x_min, grid_unit);
  const double y0 = FloorWithBase(config_.y_min, grid_unit);
  grid_frame_.emplace(grid_unit, x0, y0);

  x_dim_ =
      static_cast<int>(std::ceil((config_.x_max - x0) / grid_frame_->unit()));
  y_dim_ =
      static_cast<int>(std::ceil((config_.y_max - y0) / grid_frame_->unit()));

  QCHECK_GT(x_dim_, 0) << DebugStringFullPrecision();
  QCHECK_GT(y_dim_, 0) << DebugStringFullPrecision();

  grid_.resize(x_dim_ * y_dim_, -1);

  single_object_grid_cache_.resize(x_dim_ * y_dim_, -1);

  std::vector<std::pair<int, int>> result_cache;

  struct InitialCandidateInfo {
    int xid = 0;
    int yid = 0;
    std::vector<int> candidates;
  };

  std::vector<InitialCandidateInfo> initial_candidates_cache;
  for (int seg_id = 0; seg_id < segments_.size(); ++seg_id) {
    CalculateGridsAffectedBySegment(seg_id, &result_cache);

    for (const auto& indices : result_cache) {
      const int grid_vector_index =
          GridIndexToVectorIndex(indices.first, indices.second);
      DCHECK_GE(grid_vector_index, 0);

      int segm_vec_index = grid_[grid_vector_index];
      if (segm_vec_index < 0) {
        // Make sure field map created at given grid.
        segm_vec_index = static_cast<int>(segment_field_maps_.size());
        grid_[grid_vector_index] = segm_vec_index;
        segment_field_maps_.emplace_back(
            SegmentMap::Config{.origin_x = grid_frame_->XIdToX(indices.first),
                               .origin_y = grid_frame_->YIdToY(indices.second),
                               .root_box_width = grid_frame_->unit(),
                               .depth = config_.qtfm_depth});
        initial_candidates_cache.push_back(InitialCandidateInfo{
            .xid = indices.first, .yid = indices.second, .candidates = {}});
      }
      initial_candidates_cache[segm_vec_index].candidates.push_back(seg_id);
    }
  }

  for (int i = 0; i < segment_field_maps_.size(); ++i) {
    InitialCandidateInfo& info = initial_candidates_cache[i];
    std::stable_sort(info.candidates.begin(), info.candidates.end());
    IdField field = CreateIdFieldFromSortedVector(info.candidates);
    field.Simplify(AABox2d{
        Vec2d{grid_frame_->XIdToX(info.xid) + 0.5 * grid_frame_->unit(),
              grid_frame_->YIdToY(info.yid) + 0.5 * grid_frame_->unit()},
        grid_frame_->unit(), grid_frame_->unit()});
    segment_field_maps_[i].AddField(SegmentMap::Indices{0, 0, 0}, field);
  }

  for (auto& segment_field_map : segment_field_maps_) {
    DivideAllFieldBFS(kCutoffComplexity, &segment_field_map);
  }
}

namespace {
std::string GetFullString(double x) {
  return absl::StrFormat("%.*e", DBL_DIG, x);
}

std::string GetFullString(int x) { return std::to_string(x); }

std::string GetFullString(const Segment2d& seg) {
  return seg.DebugStringFullPrecision();
}

template <typename T>
std::string GetFullString(const std::vector<T>& list) {
  std::string result = "{";

  for (int i = 0; i < list.size() - 1; ++i) {
    result += GetFullString(list[i]) + ",";
  }

  if (!list.empty()) {
    result += GetFullString(list.back());
  }

  result += "}";
  return result;
}

}  // namespace

std::string QtfmSegmentMatcherV2::Config::DebugStringFullPrecision() const {
  std::string result = "{";
  result += ".x_min=" + GetFullString(x_min) + ",";
  result += ".x_max=" + GetFullString(x_max) + ",";
  result += ".y_min=" + GetFullString(y_min) + ",";
  result += ".y_max=" + GetFullString(y_max) + ",";

  result += ".resolution=" + GetFullString(resolution) + ",";
  result += ".qtfm_depth=" + GetFullString(qtfm_depth) + ",";
  result += ".cutoff_num_range=" + GetFullString(cutoff_num_range) + ",";
  result +=
      ".cutoff_max_num_candidate=" + GetFullString(cutoff_max_num_candidate) +
      ",";
  result += ".cutoff_distance=" + GetFullString(cutoff_distance) + "}";
  return result;
}

std::string QtfmSegmentMatcherV2::DebugStringFullPrecision() const {
  std::string result;
  result += "config=" + config_.DebugStringFullPrecision() + ",";
  result += "raw_segments=" + GetFullString<Segment2d>(raw_segments_) + ",";
  result +=
      "raw_start_segment_ids=" + GetFullString<int>(raw_start_segment_ids_);
  result += "segments=" + GetFullString<Segment2d>(segments_) + ",";
  result += "is_start_segment=" + GetFullString<Bool>(is_start_segment_);
  return result;
}

QtfmSegmentMatcherV2::QtfmSegmentMatcherV2(const QtfmSegmentMatcherV2& other) {
  // Has nothing to release, just use operator=.
  this->operator=(other);
}

QtfmSegmentMatcherV2& QtfmSegmentMatcherV2::operator=(
    const QtfmSegmentMatcherV2& other) {
  config_ = other.config_;
  segments_ = other.segments_;
  is_start_segment_ = other.is_start_segment_;
  segment_id_to_raw_id_ = other.segment_id_to_raw_id_;

  division_line_with_prev_ = other.division_line_with_prev_;
  all_ranges_ = other.all_ranges_;
  x_dim_ = other.x_dim_;
  y_dim_ = other.y_dim_;
  grid_frame_ = other.grid_frame_;
  grid_ = other.grid_;
  segment_field_maps_ = other.segment_field_maps_;
  center_distance_sqr_cache_ = other.center_distance_sqr_cache_;
  single_object_grid_cache_ = other.single_object_grid_cache_;

  for (auto& field_map : segment_field_maps_) {
    for (IdField& field : field_map) {
      field.matcher_ptr = this;
    }
  }
  return *this;
}

void QtfmSegmentMatcherV2::StitchField(const AABox2d& box, IdField* field) {
  DCHECK(field != nullptr);
  DCHECK(IsFieldValid(*field));
  if (field->range_id_end <= field->range_id_begin) {
    // Empty field won't be changed.
    return;
  }

  const int new_rid_begin = static_cast<int>(all_ranges_.size());
  int new_rid_end = new_rid_begin;

  Range current = all_ranges_[field->range_id_begin];
  for (int rid = field->range_id_begin + 1; rid < field->range_id_end; ++rid) {
    DCHECK(current.first < current.second);
    // Don't use const reference.
    const Range now = all_ranges_[rid];
    bool can_stitch_with_current =
        now.first == current.second && !is_start_segment_[now.first];

    if (can_stitch_with_current) {
      // Check order on current side.
      can_stitch_with_current &=
          current.second ==
              current.first +
                  1 ||  // If current is singleton, no check is needed.
          qtfm_util::NoReverseRegion(box,
                                     division_line_with_prev_[now.first - 1],
                                     division_line_with_prev_[now.first]);
    }
    if (can_stitch_with_current) {
      // Check order on now side.
      can_stitch_with_current &=
          now.second ==
              now.first + 1 ||  // If now is singleton, no check is needed.
          qtfm_util::NoReverseRegion(box, division_line_with_prev_[now.first],
                                     division_line_with_prev_[now.first + 1]);
    }

    if (can_stitch_with_current) {
      current.second = now.second;
    } else {
      // Will nullify all const reference.
      all_ranges_.push_back(current);
      current = now;
      new_rid_end++;
    }
  }
  all_ranges_.push_back(current);
  new_rid_end++;

  field->range_id_begin = new_rid_begin;
  field->range_id_end = new_rid_end;
}

void QtfmSegmentMatcherV2::PurgeField(const AABox2d& box, IdField* field) {
  DCHECK(field != nullptr);
  DCHECK(IsFieldValid(*field));
  if (field->range_id_end <= field->range_id_begin) {
    // Empty field won't be changed.
    return;
  }

  // Get the threshold of purge.
  center_distance_sqr_cache_.clear();
  center_distance_sqr_cache_.reserve(field->range_id_end -
                                     field->range_id_begin);
  double min_center_dist_sqr = std::numeric_limits<double>::infinity();
  for (int rid = field->range_id_begin; rid < field->range_id_end; ++rid) {
    const int nearest_seg_id_in_range =
        GetNearestSegmentIndexFromRange(box.center(), all_ranges_[rid]);
    const double center_dist_sqr =
        segments_[nearest_seg_id_in_range].DistanceSquareTo(box.center());
    min_center_dist_sqr = std::min(min_center_dist_sqr, center_dist_sqr);
    center_distance_sqr_cache_.push_back(center_dist_sqr);
  }

  const double radius = box.width() * M_SQRT1_2;
  double max_center_dist_to_be_kept =
      std::sqrt(min_center_dist_sqr) + 2.0 * radius;
  max_center_dist_to_be_kept =
      std::min(config_.cutoff_distance + radius, max_center_dist_to_be_kept);
  const double max_center_dist_to_be_kept_sqr = Sqr(max_center_dist_to_be_kept);

  // Insert those whose center dist leq max_center_dist_to_be_kept_sqr.
  const int new_rid_begin = static_cast<int>(all_ranges_.size());
  int new_rid_end = new_rid_begin;

  for (int rid = field->range_id_begin; rid < field->range_id_end; ++rid) {
    if (center_distance_sqr_cache_[rid - field->range_id_begin] <=
        max_center_dist_to_be_kept_sqr) {
      all_ranges_.push_back(all_ranges_[rid]);
      new_rid_end++;
    }
  }
  field->range_id_begin = new_rid_begin;
  field->range_id_end = new_rid_end;
}

void QtfmSegmentMatcherV2::ClampField(const AABox2d& box, IdField* field) {
  DCHECK(field != nullptr);
  DCHECK(IsFieldValid(*field));
  if (field->range_id_end <= field->range_id_begin) {
    // Empty field won't be changed.
    return;
  }
  const std::array<Vec2d, 4> box_points = box.GetAllCorners();

  for (int rid = field->range_id_begin; rid < field->range_id_end; ++rid) {
    UpClampRange(box_points, &all_ranges_[rid]);
    DownClampRange(box_points, &all_ranges_[rid]);
  }
}

void QtfmSegmentMatcherV2::DownClampRange(
    const std::array<Vec2d, 4>& box_points, Range* range) const {
  DCHECK(range != nullptr);
  DCHECK_GT(range->second, range->first);
  DCHECK_GE(range->first, 0);
  DCHECK_LE(range->second, segments_.size());
  if (range->first == range->second - 1) {
    // No clamping singleton.
    return;
  }

  // Find the first div line that is to the n-side of all box vertices.
  auto is_dlp_n_side = [&](int seg_id) {
    const Segment2d& div_line = division_line_with_prev_[seg_id];
    return div_line.ProductOntoUnit(box_points[0]) >= 0.0 &&
           div_line.ProductOntoUnit(box_points[1]) >= 0.0 &&
           div_line.ProductOntoUnit(box_points[2]) >= 0.0 &&
           div_line.ProductOntoUnit(box_points[3]) >= 0.0;
  };

  // left <= right;
  int left = range->first + 1;
  int right = range->second - 1;

  if (!is_dlp_n_side(right)) {
    // No clamp.
    return;
  }
  if (left == right || is_dlp_n_side(left)) {
    // Clamped to singleton.
    range->second = range->first + 1;
    return;
  }
  // Always: !dlpn(left), dlpn(right).

  while (true) {
    if (left >= right - 1) {
      range->second = right;
      return;
    }
    const int mid = (left + right) / 2;
    if (is_dlp_n_side(mid)) {
      right = mid;
    } else {
      left = mid;
    }
  }
  range->second = right;
}

void QtfmSegmentMatcherV2::UpClampRange(const std::array<Vec2d, 4>& box_points,
                                        Range* range) const {
  DCHECK(range != nullptr);
  DCHECK_GT(range->second, range->first);
  DCHECK_GE(range->first, 0);
  DCHECK_LE(range->second, segments_.size());
  if (range->first == range->second - 1) {
    // No clamping singleton.
    return;
  }

  // Find the first div line that is to the n-side of all box vertices.
  auto is_dlp_p_side = [&](int seg_id) {
    const Segment2d& div_line = division_line_with_prev_[seg_id];
    return div_line.ProductOntoUnit(box_points[0]) < 0.0 &&
           div_line.ProductOntoUnit(box_points[1]) < 0.0 &&
           div_line.ProductOntoUnit(box_points[2]) < 0.0 &&
           div_line.ProductOntoUnit(box_points[3]) < 0.0;
  };

  // left <= right;
  int left = range->first + 1;
  int right = range->second - 1;

  if (!is_dlp_p_side(left)) {
    // No clamp.
    return;
  }
  if (left == right || is_dlp_p_side(right)) {
    // Clamped to singleton.
    range->first = range->second - 1;
    return;
  }
  // Always: dlpp(left), !dlpp(right).

  while (true) {
    if (left >= right - 1) {
      range->first = left;
      return;
    }
    const int mid = (left + right) / 2;
    if (is_dlp_p_side(mid)) {
      left = mid;
    } else {
      right = mid;
    }
  }
  range->first = left;
}

void QtfmSegmentMatcherV2::SuperposeField(const IdField& other,
                                          IdField* field) {
  DCHECK(field != nullptr);
  DCHECK(IsFieldValid(*field));
  DCHECK(IsFieldValid(other));
  if (other.IsVoid()) {
    return;
  }

  const int new_rid_begin = static_cast<int>(all_ranges_.size());
  const int new_rid_end = new_rid_begin + other.size() + field->size();

  for (int rid = other.range_id_begin; rid < other.range_id_end; ++rid) {
    all_ranges_.push_back(all_ranges_[rid]);
  }
  for (int rid = field->range_id_begin; rid < field->range_id_end; ++rid) {
    all_ranges_.push_back(all_ranges_[rid]);
  }

  field->range_id_begin = new_rid_begin;
  field->range_id_end = new_rid_end;
}

bool QtfmSegmentMatcherV2::IsFieldSimpleEnough(const IdField& field) const {
  DCHECK(IsFieldValid(field));

  if (field.IsVoid()) {
    return true;
  }

  if (field.range_id_end - field.range_id_begin > config_.cutoff_num_range) {
    return false;
  }

  for (int rid = field.range_id_begin; rid < field.range_id_end; ++rid) {
    if (all_ranges_[rid].second - all_ranges_[rid].first >
        config_.cutoff_max_num_candidate) {
      return false;
    }
  }

  return true;
}

QtfmSegmentMatcherV2::IdField
QtfmSegmentMatcherV2::CreateIdFieldFromSortedVector(
    const std::vector<int>& sorted_segment_ids) {
  IdField res;
  res.range_id_begin = static_cast<int>(all_ranges_.size());
  res.range_id_end = res.range_id_begin + sorted_segment_ids.size();
  res.matcher_ptr = this;

  // all singletons.
  for (const int seg_id : sorted_segment_ids) {
    all_ranges_.emplace_back(seg_id, seg_id + 1);
  }
  res.is_simple_enough = IsFieldSimpleEnough(res);

  return res;
}

void QtfmSegmentMatcherV2::CalculateGridsAffectedBySegment(
    int segment_id, std::vector<std::pair<int, int>>* result) {
  DCHECK_NOTNULL(result);

  // The way of adding element.
  result->clear();
  auto append_single_set = [&](const YMonotonicGridSet2di& set) {
    for (int yid = set.yid_begin(); yid < set.yid_end(); ++yid) {
      const auto& x_range = set.xid_begin_end()[yid - set.yid_begin()];
      for (int xid = x_range.first; xid < x_range.second; ++xid) {
        const int grid_vector_index = GridIndexToVectorIndex(xid, yid);
        if (grid_vector_index < 0) {
          continue;
        }
        if (single_object_grid_cache_[grid_vector_index] == segment_id) {
          continue;
        }
        single_object_grid_cache_[grid_vector_index] = segment_id;
        result->emplace_back(xid, yid);
      }
    }
  };

  // The voronoi diagram of single segment:
  // 1, start circle.
  // 2, end circle.
  // 3 intermediate trapezoid.
  const Segment2d& segment = segments_[segment_id];
  const Vec2d& start = segment.start();
  const Vec2d& end = segment.end();
  const Vec2d left = config_.cutoff_distance * segment.unit_direction().Perp();

  std::optional<Segment2d> back_div, front_div;
  const bool has_prev = segment_id > 0 && !is_start_segment_[segment_id];
  const bool has_next =
      segment_id < segments_.size() - 1 && !is_start_segment_[segment_id + 1];

  if (has_prev) {
    back_div = division_line_with_prev_[segment_id];
  } else {
    back_div.emplace(start, start + left);
  }
  if (has_next) {
    front_div = division_line_with_prev_[segment_id + 1];
  } else {
    front_div.emplace(end, end + left);
  }
  std::optional<std::vector<Vec2d>> trapezoid =
      GetTrapezoid(*back_div, *front_div, segment, config_.cutoff_distance);

  if (!trapezoid) {
    // Backup plan.
    YMonotonicGridSet2di start_grids = CalculateGridsTouchingCircle(
        *grid_frame_, Circle2d(start, config_.cutoff_distance));
    append_single_set(start_grids);
    YMonotonicGridSet2di end_grids = CalculateGridsTouchingCircle(
        *grid_frame_, Circle2d(end, config_.cutoff_distance));
    append_single_set(end_grids);
    YMonotonicGridSet2di mid_grids = CalculateGridsTouchingConvexPolygonPoints(
        *grid_frame_,
        std::vector<Vec2d>{end + left, start + left, start - left, end - left});
    append_single_set(mid_grids);
  } else {
    YMonotonicGridSet2di mid_grids =
        CalculateGridsTouchingConvexPolygonPoints(*grid_frame_, *trapezoid);
    append_single_set(mid_grids);

    if (!has_next) {
      YMonotonicGridSet2di end_grids = CalculateGridsTouchingCircle(
          *grid_frame_, Circle2d(end, config_.cutoff_distance));
      append_single_set(end_grids);
    }
    if (!has_prev) {
      YMonotonicGridSet2di start_grids = CalculateGridsTouchingCircle(
          *grid_frame_, Circle2d(start, config_.cutoff_distance));
      append_single_set(start_grids);
    }
  }
}

bool QtfmSegmentMatcherV2::GetNearestSegmentIndex(double x, double y,
                                                  int* index) const {
  DCHECK(index != nullptr);
  DCHECK(grid_frame_);

  const int xid = grid_frame_->XToGridXId<int>(x);
  const int yid = grid_frame_->YToGridYId<int>(y);

  const int grid_vector_index = GridIndexToVectorIndex(xid, yid);

  if (grid_vector_index < 0) {
    // Out of grid range
    *index = -1;
    return false;
  }

  DCHECK(grid_vector_index < grid_.size());
  const int segm_vec_index = grid_[grid_vector_index];
  if (segm_vec_index < 0) {
    // Grid of interest is void.
    *index = -1;
    return false;
  }

  const Vec2d point(x, y);
  const IdField* field =
      segment_field_maps_[segm_vec_index].GetFieldAtPos(point);

  if (field == nullptr || field->IsVoid()) {
    // Quad tree box of interest is void.
    *index = -1;
    return false;
  }

  if (field->size() == 1) {
    const int seg_id = GetNearestSegmentIndexFromRange(
        point, all_ranges_[field->range_id_begin]);
    *index = segment_id_to_raw_id_[seg_id];
    return true;
  }

  int chosen_one = -1;
  double min_distance_sqr = std::numeric_limits<double>::infinity();
  for (int rid = field->range_id_begin; rid < field->range_id_end; ++rid) {
    const int seg_id = GetNearestSegmentIndexFromRange(point, all_ranges_[rid]);
    const double dist_sqr = segments_[seg_id].DistanceSquareTo(point);
    if (dist_sqr < min_distance_sqr) {
      min_distance_sqr = dist_sqr;
      chosen_one = seg_id;
    }
  }
  if (chosen_one >= 0) {
    *index = segment_id_to_raw_id_[chosen_one];
    return true;
  } else {
    *index = -1;
    return false;
  }
}

void QtfmSegmentMatcherV2::DrawFieldMap(vis::Canvas* canvas) const {
  constexpr int kLineWidth = 3;
  constexpr double kBoxInnerMargin = 0.1;
  if (canvas == nullptr) {
    return;
  }

  canvas->SetGroundZero(1);

  auto draw_one_box = [&canvas](double x0, double y0, double width,
                                const vis::Color& color) {
    const double x1 = x0 + width - kBoxInnerMargin;
    const double y1 = y0 + width - kBoxInnerMargin;
    x0 += kBoxInnerMargin;
    y0 += kBoxInnerMargin;
    canvas->DrawLine(Vec3d{x0, y0, 0.0}, Vec3d{x0, y1, 0.0}, color, kLineWidth);
    canvas->DrawLine(Vec3d{x0, y0, 0.0}, Vec3d{x1, y0, 0.0}, color, kLineWidth);
    canvas->DrawLine(Vec3d{x1, y1, 0.0}, Vec3d{x0, y1, 0.0}, color, kLineWidth);
    canvas->DrawLine(Vec3d{x1, y1, 0.0}, Vec3d{x1, y0, 0.0}, color, kLineWidth);
  };

  auto color_from_range_count = [&](int range_count) {
    const double ratio =
        std::clamp((1.0 * range_count) / config_.cutoff_num_range, 0.0, 1.0);
    return vis::Color{ratio, 0.0, 0.0};
  };

  for (const auto& map : segment_field_maps_) {
    for (auto iter = map.cbegin(); iter != map.cend(); ++iter) {
      int xid, yid, level;
      std::tie(level, xid, yid) = iter.GetIndices();
      const double box_width =
          (1 << (map.Depth() - 1 - level)) * config_.resolution;
      const double x0 = map.config().origin_x + xid * box_width;
      const double y0 = map.config().origin_y + yid * box_width;

      draw_one_box(x0, y0, box_width, color_from_range_count(iter->size()));
    }
  }
}

void QtfmSegmentMatcherV2::DrawSegments(vis::Canvas* canvas) const {
  constexpr double kZ = 0.3;
  constexpr int kLineWidth = 3;

  if (canvas == nullptr) {
    return;
  }
  canvas->SetGroundZero(1);

  const std::array<vis::Color, 3> color_pool = {vis::Color{0.2, 0.2, 1.0},
                                                vis::Color{0.2, 1.0, 1.0},
                                                vis::Color{0.4, 1.0, 0.4}};

  for (int i = 0; i < segments_.size(); ++i) {
    const auto& segment = segments_[i];
    const vis::Color& color = color_pool[i % 3];
    canvas->DrawLine(Vec3d{segment.start().x(), segment.start().y(), kZ},
                     Vec3d{segment.end().x(), segment.end().y(), kZ}, color,
                     kLineWidth);
  }
}

bool QtfmSegmentMatcherV2::IsDivisible(const Segment2d& prev_seg,
                                       const Segment2d& cur_seg) {
  // Same direction.
  const bool on_same_line =
      std::abs(prev_seg.unit_direction().CrossProd(cur_seg.unit_direction())) <=
          kMaxSameDirCrossProdError &&
      std::abs(prev_seg.unit_direction().CrossProd(
          cur_seg.start() - prev_seg.end())) <= kMaxSameDirCrossProdError;
  if (on_same_line) {
    return true;
  }

  // Same length
  const bool has_same_length_and_connected =
      std::abs(prev_seg.length() - cur_seg.length()) <= kLengthEpsilon &&
      cur_seg.start().DistanceTo(prev_seg.end()) <= kLengthEpsilon;
  if (has_same_length_and_connected) {
    return true;
  }

  return false;
}

Segment2d QtfmSegmentMatcherV2::GetDivisionLine(const Segment2d& prev_seg,
                                                const Segment2d& cur_seg) {
  Vec2d div_dir =
      cur_seg.unit_direction().Perp() + prev_seg.unit_direction().Perp();

  if (div_dir.Length() < kLengthEpsilon) {
    div_dir = cur_seg.unit_direction();
  }
  const Vec2d joint = 0.5 * (prev_seg.end() + cur_seg.start());
  return Segment2d(joint, joint + div_dir);
}

void QtfmSegmentMatcherV2::MakeSegmentsDivisible(
    const std::vector<Segment2d>& raw_segments,
    const std::vector<int>& raw_start_segment_ids,
    std::vector<Segment2d>* divisible_segments,
    std::vector<int>* divisible_start_segment_ids,
    std::vector<int>* raw_segment_index_of_divisible_segments) {
  QCHECK_NOTNULL(divisible_segments);
  QCHECK_NOTNULL(divisible_start_segment_ids);
  QCHECK_NOTNULL(raw_segment_index_of_divisible_segments);
  divisible_segments->clear();
  divisible_start_segment_ids->clear();
  raw_segment_index_of_divisible_segments->clear();

  std::vector<Bool> is_start_raw_segment;
  is_start_raw_segment.resize(raw_segments.size(), 0);
  // TODO(huaiyuan): do not depend on input's specification of connection
  // status. Simply remove the input later.
  for (const int start_id : raw_start_segment_ids) {
    is_start_raw_segment[start_id] = 1;
  }
  // Mark those not connected segments start segment.
  int num_start_ids = 0;
  for (int i = 0; i < raw_segments.size(); ++i) {
    if (i == 0 || raw_segments[i].start().DistanceTo(
                      raw_segments[i - 1].end()) > 0.5 * kLengthEpsilon) {
      is_start_raw_segment[i] = 1;
      num_start_ids += 1;
    }
  }

  divisible_start_segment_ids->reserve(num_start_ids);
  divisible_segments->reserve(raw_segments.size() * 3);
  raw_segment_index_of_divisible_segments->reserve(raw_segments.size() * 3);

  for (int i = 0; i < raw_segments.size(); ++i) {
    if (is_start_raw_segment[i]) {
      divisible_start_segment_ids->push_back(divisible_segments->size());
    }

    double up_break_ratio = 0.5;
    double down_break_ratio = 0.5;
    const double length = raw_segments[i].length();
    QCHECK_GE(length, kMinSegmentLength)
        << raw_segments[i].DebugStringFullPrecision();

    if (i > 0 && !is_start_raw_segment[i] &&
        raw_segments[i - 1].length() < length) {
      // Has a prev segment shorter than current.
      down_break_ratio *= raw_segments[i - 1].length() / length;
    }

    if (i < raw_segments.size() - 1 && !is_start_raw_segment[i + 1] &&
        raw_segments[i + 1].length() < length) {
      // Has a next segment shorter than current.
      up_break_ratio *= raw_segments[i + 1].length() / length;
    }

    const Segment2d& segment = raw_segments[i];
    const Vec2d s2e = segment.end() - segment.start();

    // All edge length >= 0.5 * kMinSegmentLength.
    Segment2d seg0 = segment;
    seg0.Scale(down_break_ratio);
    divisible_segments->push_back(seg0);
    raw_segment_index_of_divisible_segments->push_back(i);

    Segment2d seg1 = segment;
    seg1.Scale(1.0 - down_break_ratio - up_break_ratio);
    seg1.Shift(down_break_ratio * s2e);
    if (seg1.length() > 0.5 * kMinSegmentLength) {
      divisible_segments->push_back(seg1);
      raw_segment_index_of_divisible_segments->push_back(i);
    }

    Segment2d seg2 = segment;
    seg2.Scale(up_break_ratio);
    seg2.Shift((1.0 - up_break_ratio) * s2e);
    divisible_segments->push_back(seg2);
    raw_segment_index_of_divisible_segments->push_back(i);
  }
}

}  // namespace qcraft::planner
