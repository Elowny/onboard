#ifndef ONBOARD_PLANNER_UTIL_QUAD_TREE_FIELD_MAP_H_
#define ONBOARD_PLANNER_UTIL_QUAD_TREE_FIELD_MAP_H_

#include <deque>
#include <tuple>
#include <utility>

#include "onboard/math/geometry/grid_frame.h"
#include "onboard/planner/util/quad_tree.h"

namespace qcraft::planner {

// The class defined here is the base class of FieldType
// that can't be compiled by C++. The comment here is for reference
// purpose. One may also uncomment them to facilitate auto completion.

/*
class FieldType {
public:
  // field superposed with another one.
  void SuperPose(const FieldType& other);

  // A field can be simplified when the domain is restricted.
  void Simplify(const AABox2d& box);

  // Being true means the field is empty.
  bool IsVoid() const;
};
*/

template <typename T, int Capacity>
struct BoundedVector {
  int num_elements = 0;
  std::array<T, Capacity> elements;

  constexpr void push_back(T element) {
    QCHECK_LT(num_elements, Capacity);
    elements[num_elements] = std::move(element);
    num_elements += 1;
  }
};

template <class FieldType>
class QuadTreeFieldMap {
 public:
  static constexpr int kMaxDepth = 20;

  struct Config {
    double origin_x = 0.0;
    double origin_y = 0.0;
    double root_box_width = 0.0;
    int depth = 0;
  };

  // indice is level, xid, yid of the map_.
  struct Indices {
    int level = 0;
    int xid = 0;
    int yid = 0;
  };

  using FourIndices = BoundedVector<Indices, 4>;
  using iterator = typename QuadTree<FieldType>::iterator;
  using const_iterator = typename QuadTree<FieldType>::const_iterator;

  explicit QuadTreeFieldMap(Config config)
      : config_(std::move(config)),
        tree_(config_.depth),
        unit_box_width_(
            tree_.RootBoxWidthToUnitBoxWidth(config_.root_box_width)) {
    CHECK_LE(config_.depth, kMaxDepth);
  }

  constexpr const FieldType* GetFieldAtPos(const Vec2d& position) const;

  // Add a field to the map.
  // indices is level, xid, yid of the map_.
  // Return true when field is successfully added.
  // Return false when container overflow.
  bool AddField(const Indices& indices, FieldType field);

  // Return the const pointer to the field when present at given indices,
  // or nullptr when it is absent, void or out of range.
  const FieldType* GetField(const Indices& indices) const;

  // The remaining capacity of slots to storage field.
  int RemainingCapacity() const {
    return static_cast<int>(tree_.max_num_elements() - tree_.size());
  }

  // The quad tree depth.
  int Depth() const { return config_.depth; }

  constexpr size_t size() const { return tree_.size(); }

  // Pre-condition:
  // 1, RemainingCapacity()>=4.
  // 2, indices.level+1 < Depth().
  // Sub-divide the field at given indices, without changing the
  // effect of the field.
  // Return a vector of newly added child indices.
  // The returned vector can be empty when no child addition is
  // needed.
  FourIndices QuarterField(const Indices& indices);

  iterator begin() { return tree_.begin(); }

  iterator end() { return tree_.end(); }

  const_iterator cbegin() const { return tree_.cbegin(); }

  const_iterator cend() const { return tree_.cend(); }

  const Config& config() const { return config_; }

 private:
  Config config_;
  QuadTree<FieldType> tree_;

  // TODO(huaiyuan) : all classes should use unit box width.
  double unit_box_width_ = 0.0;
};

template <class FieldType>
constexpr const FieldType* QuadTreeFieldMap<FieldType>::GetFieldAtPos(
    const Vec2d& position) const {
  const FieldType* p_field = tree_.GetElementByPosition(
      position.x() - config_.origin_x, position.y() - config_.origin_y,
      config_.root_box_width);
  if (p_field == nullptr) {
    return nullptr;
  }
  return p_field;
}

template <class FieldType>
bool QuadTreeFieldMap<FieldType>::AddField(const Indices& indices,
                                           FieldType field) {
  if (!tree_.IsInside(indices.level, indices.xid, indices.yid)) {
    return true;
  }
  if (field.IsVoid()) {
    return true;
  }

  FieldType* p_existing_field =
      tree_.GetMutableElement(indices.level, indices.xid, indices.yid);
  if (p_existing_field == nullptr) {
    return tree_.AddElement(indices.level, indices.xid, indices.yid,
                            std::move(field));
  } else {
    p_existing_field->SuperPose(field);
    if (p_existing_field->IsVoid()) {
      tree_.RemoveElement(indices.level, indices.xid, indices.yid);
    }
    return true;
  }
}

template <class FieldType>
const FieldType* QuadTreeFieldMap<FieldType>::GetField(
    const Indices& indices) const {
  if (!tree_.IsInside(indices.level, indices.xid, indices.yid)) {
    return nullptr;
  }
  const FieldType* p_field =
      tree_.GetElement(indices.level, indices.xid, indices.yid);
  if (p_field == nullptr) {
    return nullptr;
  }
  if (p_field->IsVoid()) {
    return nullptr;
  }
  return p_field;
}

template <class FieldType>
typename QuadTreeFieldMap<FieldType>::FourIndices
QuadTreeFieldMap<FieldType>::QuarterField(const Indices& indices) {
  DCHECK_GE(RemainingCapacity(), 4);
  DCHECK_LT(indices.level + 1, tree_.depth());

  FourIndices added_chilren;
  const FieldType* p_field = GetField(indices);
  if (p_field == nullptr) {
    return added_chilren;
  }

  const int next_level = indices.level + 1;
  const int nxid = indices.xid << 1;
  const int nyid = indices.yid << 1;

  const std::array<std::pair<int, int>, 4> children_rids = {
      std::pair<int, int>{0, 0}, std::pair<int, int>{0, 1},
      std::pair<int, int>{1, 0}, std::pair<int, int>{1, 1}};
  // When adding new element, p_field will be a nullified pointer.
  FieldType parent = *p_field;
  for (const auto& child_rid : children_rids) {
    Indices child_indices{.level = next_level,
                          .xid = nxid + child_rid.first,
                          .yid = nyid + child_rid.second};
    FieldType child_field = parent;
    AABox2d child_box = tree_.BoxAt(child_indices.level, child_indices.xid,
                                    child_indices.yid, unit_box_width_);
    child_box.Shift(Vec2d{config_.origin_x, config_.origin_y});
    child_field.Simplify(child_box);

    if (!child_field.IsVoid()) {
      QCHECK(AddField(child_indices, std::move(child_field)));
      added_chilren.push_back(std::move(child_indices));
    }
  }
  tree_.RemoveElement(indices.level, indices.xid, indices.yid);
  return added_chilren;
}

template <typename FieldType>
void DivideAllFieldBFS(double cutoff_complexity,
                       QuadTreeFieldMap<FieldType>* map) {
  typedef QuadTreeFieldMap<FieldType> MapType;
  typedef typename MapType::Indices Indices;

  std::deque<Indices> cache;
  // Add all initial fields
  for (auto iter = map->begin(); iter != map->end(); ++iter) {
    const std::tuple<int, int, int> indices = iter.GetIndices();
    cache.push_back(Indices{.level = std::get<0>(indices),
                            .xid = std::get<1>(indices),
                            .yid = std::get<2>(indices)});
  }

  while (!cache.empty() && map->RemainingCapacity() >= 4) {
    const Indices& now = cache.front();
    const FieldType* p_field = map->GetField(now);

    if (now.level + 1 < map->Depth() && p_field != nullptr &&
        p_field->Complexity() > cutoff_complexity) {
      typename MapType::FourIndices added_children = map->QuarterField(now);
      for (int i = 0; i < added_children.num_elements; ++i) {
        cache.push_back(std::move(added_children.elements[i]));
      }
    }
    cache.pop_front();
  }
}

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_UTIL_QUAD_TREE_FIELD_MAP_H_
