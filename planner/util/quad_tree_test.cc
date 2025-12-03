#include "onboard/planner/util/quad_tree.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type

#include <stdlib.h>
#include <sys/types.h>

#include <cmath>
#include <ostream>
#include <unordered_map>
#include <utility>

#include "gtest/gtest.h"

namespace qcraft::planner {
namespace {

TEST(QuadTreeTest, QuadTreeAsMapTest) {
  const int depth = 7;
  const int n = 1000000;
  QuadTree<double> tree(depth);

  struct HashIntPair {
    size_t operator()(const std::pair<int, int>& p) const {
      const size_t hash1 = std::hash<int>{}(p.first);
      const size_t hash2 = std::hash<int>{}(p.second);
      return hash1 ^ hash2;
    }
  };
  std::vector<std::unordered_map<std::pair<int, int>, double, HashIntPair>>
      elements_at_level(depth);

  uint seed = 0;

  std::vector<int> power_of_two(depth + 1, 1);
  for (int i = 1; i < depth + 1; ++i) {
    power_of_two[i] = 2 * power_of_two[i - 1];
  }

  const int log_point = n / 10;
  for (int i = 0; i < n; ++i) {
    LOG_IF(INFO, /*i>219900 && */ i % log_point == 0) << i << "/" << n;

    const int level = rand_r(&seed) % depth;
    const int xid = rand_r(&seed) % power_of_two[level];
    const int yid = rand_r(&seed) % power_of_two[level];

    QCHECK(tree.IsInside(level, xid, yid));

    const auto key = std::make_pair(xid, yid);
    if (elements_at_level[level].count(key) > 0) {
      // CHECK existence and equality.
      const double* ele = tree.GetElement(level, xid, yid);
      QCHECK(ele != nullptr);
      EXPECT_EQ(*ele, elements_at_level[level][key]);

      // Delete it or not delete it.
      if (rand_r(&seed) % 2 == 0) {
        elements_at_level[level].erase(key);
        tree.RemoveElement(level, xid, yid);
      }
    } else {
      const double* ele = tree.GetElement(level, xid, yid);
      EXPECT_EQ(ele, nullptr);

      // Add it.
      const double val = rand_r(&seed) % 1000;
      elements_at_level[level][key] = val;
      tree.AddElement(level, xid, yid, val);
    }
  }
}

TEST(QuadTreeTest, QuadTreeGeometryTest) {
  constexpr int kDepth = 6;

  uint seed = 0;
  QuadTree<std::tuple<int, int, int>> indices_tree(kDepth);
  int num_elements_in_tree = 0;

  // Post some random elements.
  for (int level = 0; level < kDepth; level++) {
    const int num_boxes = std::pow(2, level);
    for (int xid = 0; xid < num_boxes; ++xid) {
      for (int yid = 0; yid < num_boxes; ++yid) {
        if (rand_r(&seed) % 2 == 0 && level != 3) {
          indices_tree.AddElement(level, xid, yid,
                                  std::make_tuple(level, xid, yid));
          num_elements_in_tree++;
        }
      }
    }
  }

  // Randomly delete 1/3 elements.
  for (int level = 0; level < kDepth; level++) {
    const int num_boxes = std::pow(2, level);
    for (int xid = 0; xid < num_boxes; ++xid) {
      for (int yid = 0; yid < num_boxes; ++yid) {
        if (rand_r(&seed) % 3 == 0 &&
            indices_tree.GetElement(level, xid, yid) != nullptr) {
          indices_tree.RemoveElement(level, xid, yid);
          num_elements_in_tree--;
        }
      }
    }
  }

  // CHECK iterator connectivity.
  int num_elements_by_iterating = 0;
  for (auto iter = indices_tree.begin(); iter != indices_tree.end(); ++iter) {
    num_elements_by_iterating += 1;
  }
  EXPECT_EQ(num_elements_by_iterating, num_elements_in_tree);
  EXPECT_EQ(num_elements_by_iterating, indices_tree.size());

  // Check
  const int n = 1000000;
  const int div = 10000;
  const int log_point = n / 10;

  for (int i = 0; i < n; ++i) {
    LOG_IF(INFO, i % log_point == 0) << i << "/" << n;

    const double x = rand_r(&seed) % div * 1.0 / div;
    const double y = rand_r(&seed) % div * 1.0 / div;
    QuadTree<std::tuple<int, int, int>>::ConstElementReferenceArray result =
        indices_tree.GetElementsByPosition(x, y);

    for (int i = 0; i < result.num_elements; ++i) {
      const auto& ele = *result.elements[i];
      const int level = std::get<0>(ele);
      const int num_boxes = 1 << level;
      EXPECT_EQ(static_cast<int>(x * num_boxes), std::get<1>(ele));
      EXPECT_EQ(static_cast<int>(y * num_boxes), std::get<2>(ele));
    }

    const auto leaf = indices_tree.GetElementByPosition(x, y);
    if (leaf != nullptr) {
      const int level = std::get<0>(*leaf);
      const int num_boxes = 1 << level;
      EXPECT_EQ(static_cast<int>(x * num_boxes), std::get<1>(*leaf));
      EXPECT_EQ(static_cast<int>(y * num_boxes), std::get<2>(*leaf));
    }
  }
}

}  // namespace
}  // namespace qcraft::planner
