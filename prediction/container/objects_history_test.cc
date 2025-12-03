#include "onboard/prediction/container/objects_history.h"

// IWYU pragma: no_include <boost/move/utility_core.hpp>  // for move

#include "gtest/gtest.h"
#include "onboard/math/coordinate_converter.h"

namespace qcraft {
namespace prediction {

TEST(ObjectsHistoryTest, Test_ObjectHistory_Push) {
  ObjectsHistory objects_history(3);
  CoordinateConverter loc_converter;
  EXPECT_EQ(objects_history["0"].empty(), true);
  EXPECT_EQ(objects_history["0"].Push(
                1.0, PredictionObject(ObjectProto(), loc_converter)),
            true);
  EXPECT_EQ(objects_history["0"].Push(
                1.0, PredictionObject(ObjectProto(), loc_converter)),
            false);
}

}  // namespace prediction
}  // namespace qcraft
