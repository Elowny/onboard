#ifndef ONBOARD_PLANNER_UTIL_QTFM_SEGMENT_MATCHER_V2_H_
#define ONBOARD_PLANNER_UTIL_QTFM_SEGMENT_MATCHER_V2_H_

#include <sys/types.h>

#include <algorithm>
#include <array>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "glog/logging.h"

#include "onboard/math/geometry/aabox2d.h"
#include "onboard/math/geometry/grid_frame.h"
#include "onboard/math/geometry/segment2d.h"
#include "onboard/math/vec.h"
#include "onboard/planner/util/quad_tree_field_map.h"
#include "onboard/vis/canvas/canvas.h"

namespace qcraft::planner {

// Segment matching problem is often used in
// motion planning tasks. Shortly speaking, for a list of 2d segments
// [s1,s2,s3,s4...sn], given a random 2d point p,
// find the index of the segment whose .DistanceTo(p)
// is the smallest/one of the smallests.
// Such task is known as segment matching. And the agent that can
// accelerate the query for a pre-defined/fixed segment list is known
// as a segment matcher.

// For qtfm segment matcher, user should be aware that a cutoff_distance is
// required to limit the spacial impact of each segment. Its formal definition
// is: If the distance of segment `s` to querry point `p` is greater equal to
// cutoff_distance, `s` will be skipped/excluded in the smallest .DistanceTo(p)
// competition.

// Some slides to help understand the algorithm:
// https://docs.google.com/presentation/d/1gPnm5NxZPd7iIZ5fnBBcv2EhyhKcpeJj4PMXFlRfw1c/edit?usp=sharing
class QtfmSegmentMatcherV2 {
 public:
  static constexpr double kMinSegmentLength = 1e-5;
  static constexpr double kLengthEpsilon = 1e-6;
  static constexpr double kMaxSameDirCrossProdError = 1e-6;
  struct Config {
    // TODO(huaiyuan): Simplify the config by removing the following
    // parameters and use segments' bounding box.
    // Region of interest.
    // Queries outside the domain is not guaranteed to be correct.
    double x_min = 0.0;
    double x_max = 0.0;
    double y_min = 0.0;
    double y_max = 0.0;

    // Implementation related parameters:
    // The matcher's structure is a sparse grid map of
    // QuadTreeFieldMap<NearestSegmentIdField>
    //
    //   ___________________________
    //  |      |      |      |      |
    //  | qtfm | qtfm |      | qtfm |
    //  |______|______|______|______|
    //  |      |      |      |      |
    //  | qtfm |      | qtfm | qtfm |
    //  |______|______|______|______|
    //  |      |      |      |      |
    //  |      | qtfm |      |      |
    //  |______|______|______|______|
    //  |      |      |      |      |
    //  |      |      | qtfm |      |
    //  |______|______|______|______|
    //
    //  |<-gu->|
    //
    // gu means grid unit.
    // The origin of the container-grid will be a vertex
    // of GridFrame(gu, 0.0, 0.0);

    // The smallest box size of each qtfm.
    double resolution = 0.0;
    // The depth of each qtfm.
    int qtfm_depth = 1;

    // When a field's range count <= cutoff_num_range,
    // and the max number of single range candidates <=
    // cutoff_max_num_candidate. The field will be deemed as simple enough,
    // such that it don't need to be further divided.
    int cutoff_num_range = 1;
    int cutoff_max_num_candidate = 16;

    // The matcher result might be wrong when the point's
    // nearest segment distance is greater than cutoff_distance.
    double cutoff_distance = 0.0;

    bool IsValid() const {
      return x_max > x_min && y_max > y_min && resolution > 0.0 &&
             qtfm_depth > 0 && cutoff_num_range > 0 &&
             cutoff_max_num_candidate > 0 && cutoff_distance > 0.0;
    }

    std::string DebugStringFullPrecision() const;
  };

  // For example, there are 9 segments:
  // 0 1 2 3 4   5 6 7   8.
  // ->->->->->  ->->->  ->
  // Where 0~4 forms a polyline.
  // 5,6,7 forms a polyline.
  // 8 is a single segment that connects to no one.
  // The input of start_segment_ids should be {0, 5, 8}.
  QtfmSegmentMatcherV2(Config config, std::vector<Segment2d> raw_segments,
                       std::vector<int> raw_start_segment_ids);

  bool GetNearestSegmentIndex(double x, double y, int* index) const;

  void DrawFieldMap(vis::Canvas* canvas) const;

  void DrawSegments(vis::Canvas* canvas) const;

  QtfmSegmentMatcherV2(const QtfmSegmentMatcherV2& other);
  QtfmSegmentMatcherV2& operator=(const QtfmSegmentMatcherV2& other);

  const Config& config() const { return config_; }

  std::string DebugStringFullPrecision() const;

 private:
  // Quad-tree field map complexity manipulator
  // for BFS process.
  static constexpr double kCutoffComplexity = 2.0;
  static constexpr double kCutoff = 1.0;
  static constexpr double kKeepDivide = 3.0;
  using Range = std::pair<int, int>;
  using Bool = u_char;

  struct IdField {
    // What all it needs to describe a field.
    // list of ranges

    // id in all_ranges_.
    // ranges are sorted by segment id.

    int range_id_begin;
    int range_id_end;

    // A range must satisefy:
    // -|-|-|-|-|-
    // if exist, each consecutive div line pair must be ordered
    // such that NoReverseRegion(prev, cur) == true;

    QtfmSegmentMatcherV2* matcher_ptr = nullptr;
    bool is_simple_enough = false;

    double Complexity() const {
      return is_simple_enough ? kCutoff : kKeepDivide;
    }

    // field superposed with another one.
    constexpr void SuperPose(const IdField& other) {
      matcher_ptr->SuperposeField(other, this);
      is_simple_enough = matcher_ptr->IsFieldSimpleEnough(*this);
    }

    // The following constexpr was observed to be having
    // "make the program run faster" effect.
    // I am not sure why and how.
    // Decided to leave the consexpr
    // expression here for now since it is harmless.
    // A field can be simplified when the domain is restricted.
    constexpr void Simplify(const AABox2d& box) {
      matcher_ptr->StitchField(box, this);
      matcher_ptr->PurgeField(box, this);
      matcher_ptr->ClampField(box, this);
      is_simple_enough = matcher_ptr->IsFieldSimpleEnough(*this);
    }

    // Being true means the field is empty.
    constexpr bool IsVoid() const { return size() == 0; }

    constexpr int size() const {
      return std::max(0, range_id_end - range_id_begin);
    }
  };

  bool IsFieldValid(const IdField& field) const {
    return field.range_id_begin >= 0 &&
           field.range_id_end <= all_ranges_.size();
  }

  std::string FieldDebugString(const IdField& field) const {
    if (field.IsVoid()) {
      return "";
    }
    std::string res = "ranges:";
    for (int rid = field.range_id_begin; rid < field.range_id_end; ++rid) {
      res += std::to_string(all_ranges_[rid].first);
      res += "~";
      res += std::to_string(all_ranges_[rid].second);
      res += ",";
    }
    return res;
  }

  // Neighbor ranges can be stiched to make larger range, when in box.
  void StitchField(const AABox2d& box, IdField* field);

  // If a range's nearest distance to box is larger than
  // some other range's longest distance, this one will be removed.
  void PurgeField(const AABox2d& box, IdField* field);

  // For each single range, clamp it by
  // binary searching the upper division line
  // and bottom division line
  void ClampField(const AABox2d& box, IdField* field);

  void DownClampRange(const std::array<Vec2d, 4>& box_points,
                      Range* range) const;

  void UpClampRange(const std::array<Vec2d, 4>& box_points, Range* range) const;

  // The range is valid.
  constexpr int GetNearestSegmentIndexFromRange(const Vec2d& point,
                                                const Range& range) const;

  void SuperposeField(const IdField& other, IdField* field);

  bool IsFieldSimpleEnough(const IdField& field) const;

  IdField CreateIdFieldFromSortedVector(
      const std::vector<int>& sorted_segment_ids);

  constexpr int GridIndexToVectorIndex(int xid, int yid) const {
    if (xid < 0 || xid >= x_dim_ || yid < 0 || yid >= y_dim_) {
      return -1;
    }
    return xid * y_dim_ + yid;
  }

  // xid, yid
  // Not clearing single_object_grid_cache_.
  // Make sure each segment_id be called only once.
  void CalculateGridsAffectedBySegment(
      int segment_id, std::vector<std::pair<int, int>>* result);

  // Whether the two segments' voronoi diagram can be described by half-plane.
  // In other word, whether the two segments can be splited by a division line,
  // such that:
  // On one side all points are nearer to segment 1 and on the other side all
  // points are nearer to segment 2.
  static bool IsDivisible(const Segment2d& prev_seg, const Segment2d& cur_seg);

  // If the 2 segments can be splited by a division line, such that
  // on one side all points are nearer to segment 1
  // and on the other side all points are nearer to segment 2,
  // Find the division line.
  static Segment2d GetDivisionLine(const Segment2d& prev_seg,
                                   const Segment2d& cur_seg);

  static void MakeSegmentsDivisible(
      const std::vector<Segment2d>& raw_segments,
      const std::vector<int>& raw_start_segment_ids,
      std::vector<Segment2d>* divisible_segments,
      std::vector<int>* divisible_start_segment_ids,
      std::vector<int>* raw_segment_index_of_divisible_segments);

  using SegmentMap = QuadTreeFieldMap<IdField>;

  Config config_;
  std::vector<Segment2d> raw_segments_;
  std::vector<int> raw_start_segment_ids_;

  std::vector<Segment2d> segments_;
  std::vector<Bool> is_start_segment_;
  std::vector<int> segment_id_to_raw_id_;

  // Not valid when is_start_segment_[id] > 0.
  std::vector<Segment2d> division_line_with_prev_;

  // begin segment id, end segment id.
  std::vector<std::pair<int, int>> all_ranges_;

  int x_dim_ = 0;
  int y_dim_ = 0;
  std::optional<GridFrame2d> grid_frame_;

  // index of segment field map.
  std::vector<int> grid_;
  std::vector<SegmentMap> segment_field_maps_;

  std::vector<double> center_distance_sqr_cache_;
  std::vector<int> single_object_grid_cache_;
};

constexpr int QtfmSegmentMatcherV2::GetNearestSegmentIndexFromRange(
    const Vec2d& point, const Range& range) const {
  // Binary search or linear interpolation.
  DCHECK(range.second > range.first);        // NOLINT
  DCHECK(range.first >= 0);                  // NOLINT
  DCHECK(range.second <= segments_.size());  // NOLINT

  if (range.first == range.second - 1) {
    return range.first;
  }

  // left <= right;
  int left = range.first + 1;
  int right = range.second - 1;

  if (division_line_with_prev_[right].ProductOntoUnit(point) < 0.0) {
    return right;
  }
  if (left == right ||
      division_line_with_prev_[left].ProductOntoUnit(point) >= 0.0) {
    return left - 1;
  }
  // Always: dlp[left] <0, dlp[right] >= 0.

  while (true) {
    if (left >= right - 1) {
      return left;
    }

    const int mid = (left + right) / 2;
    if (division_line_with_prev_[mid].ProductOntoUnit(point) >= 0.0) {
      right = mid;
    } else {
      left = mid;
    }
  }
  return left;
}

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_UTIL_QTFM_SEGMENT_MATCHER_V2_H_
