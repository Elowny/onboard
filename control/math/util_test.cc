#include "onboard/control/math/util.h"

#include "gtest/gtest.h"

namespace qcraft {
namespace control {
namespace {

TEST(ComputeVectorMeanTest, CircularBuffer) {
  boost::circular_buffer<double> cb_double{5};
  EXPECT_DOUBLE_EQ(ComputeMeanOfCircularBuffer(cb_double), 0.0);
  cb_double.push_back(0.0);
  cb_double.push_back(1.0);
  cb_double.push_back(2.0);
  cb_double.push_back(3.0);
  cb_double.push_back(4.0);
  cb_double.push_back(5.0);
  EXPECT_DOUBLE_EQ(ComputeMeanOfCircularBuffer(cb_double), 3.0);

  boost::circular_buffer<int> cb_int{5};
  EXPECT_EQ(ComputeMeanOfCircularBuffer(cb_int), 0);
  cb_int.push_back(0);
  cb_int.push_back(1);
  cb_int.push_back(2);
  cb_int.push_back(3);
  cb_int.push_back(4);
  cb_int.push_back(5);
  EXPECT_EQ(ComputeMeanOfCircularBuffer(cb_int), 3);
}

}  // namespace
}  // namespace control
}  // namespace qcraft
