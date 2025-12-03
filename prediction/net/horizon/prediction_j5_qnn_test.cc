#include "onboard/prediction/net/horizon/prediction_j5_qnn.h"

#include "gtest/gtest.h"

#include "onboard/lite/logging.h"
#include "onboard/nets/proto/net_param.pb.h"
#include "onboard/params/param_finder.h"
#include "onboard/prediction/test_util/net_input_generator.h"

namespace qcraft {
namespace prediction {
namespace {
#if (defined(Q_CPU_ONLY) && defined(__X86_64__)) || defined(__J5__)
TEST(PredictionJ5QNNTest, RunActNetJ5HBDNN) {
  NetParam actnet_j5_param;
  QCHECK_OK(GetProtoParamById("Q2206", "act_net_j5_param", &actnet_j5_param));
  std::unique_ptr<prediction::PredictionJ5QNN> net(
      new prediction::PredictionJ5QNN(actnet_j5_param));

  EXPECT_EQ(net->GetInputMap().size(),
            actnet_j5_param.input_tensor_list_size());
  for (int i = 0; i < actnet_j5_param.input_tensor_list_size(); ++i) {
    const auto& tensor_info = actnet_j5_param.input_tensor_list(i);
    std::vector<int> valid_shape;
    valid_shape.reserve(4);
    for (auto it : tensor_info.shape()) {
      valid_shape.push_back(it);
    }
    const auto aligned_shape_param =
        ValidShape2AlignedShape(valid_shape, qnn::DataType::kInt8);
    const auto input_tensor = net->GetInputByName(tensor_info.name());
    // Skip batch dim.
    for (int j = 1; j < aligned_shape_param.size(); ++j) {
      EXPECT_EQ(aligned_shape_param[j], input_tensor->shape(j));
    }
  }

  EXPECT_EQ(net->GetOutputMap().size(),
            actnet_j5_param.output_tensor_list_size());
  for (int i = 0; i < actnet_j5_param.output_tensor_list_size(); ++i) {
    const auto& tensor_info = actnet_j5_param.output_tensor_list(i);
    std::vector<int> valid_shape;
    valid_shape.reserve(4);
    for (auto it : tensor_info.shape()) {
      valid_shape.push_back(it);
    }
    const auto aligned_shape_param =
        ValidShape2AlignedShape(valid_shape, qnn::DataType::kInt32);
    const auto output_tensor = net->GetOutputByName(tensor_info.name());
    // Skip batch dim.
    for (int j = 1; j < aligned_shape_param.size(); ++j) {
      EXPECT_EQ(aligned_shape_param[j], output_tensor->shape(j));
    }
  }

  const auto batch_inputs =
      GenUniformDistributedInputByNetParam(actnet_j5_param);
  for (int i = 0; i < batch_inputs.size(); ++i) {
    const auto& name = actnet_j5_param.input_tensor_list(i).name();
    char* data_ptr = net->GetInputByName(name)->GetDataPtr<char>();
    std::memcpy(data_ptr, batch_inputs[i].data(),
                batch_inputs[i].size() * sizeof(char));
  }
  const int max_batch_size = actnet_j5_param.max_batch_size();
  const auto run_res = net->Run(max_batch_size);
  EXPECT_EQ(run_res.size(), actnet_j5_param.output_tensor_list_size());

  // Reset input tensor.
  net->ResetInputs();
  for (int i = 0; i < batch_inputs.size(); ++i) {
    const auto& name = actnet_j5_param.input_tensor_list(i).name();
    float* data_ptr = net->GetInputByName(name)->GetDataPtr<float>();
    EXPECT_EQ(data_ptr[i], 0.0f);
  }
}
#else
TEST(PredictionJ5QNNTest, RunActNetJ5ONNX) {
  NetParam actnet_j5_param;
  QCHECK_OK(GetProtoParamById("Q2206", "act_net_j5_param", &actnet_j5_param));
  actnet_j5_param.set_device_type(NetParam_DeviceType_CPU);
  actnet_j5_param.set_device_id(NetParam_DeviceID_DEVICE_0);
  actnet_j5_param.set_backend_type(NetParam_BackendType_ONNX);
  std::unique_ptr<prediction::PredictionJ5QNN> net(
      new prediction::PredictionJ5QNN(actnet_j5_param));

  EXPECT_EQ(net->GetInputMap().size(),
            actnet_j5_param.input_tensor_list_size());
  for (int i = 0; i < actnet_j5_param.input_tensor_list_size(); ++i) {
    const auto& tensor_info = actnet_j5_param.input_tensor_list(i);
    const auto input_tensor = net->GetInputByName(tensor_info.name());
    // Skip batch dim.
    for (int j = 1; j < tensor_info.shape_size(); ++j) {
      EXPECT_EQ(tensor_info.shape(j), input_tensor->shape(j));
    }
  }

  EXPECT_EQ(net->GetOutputMap().size(),
            actnet_j5_param.output_tensor_list_size());
  for (int i = 0; i < actnet_j5_param.output_tensor_list_size(); ++i) {
    const auto& tensor_info = actnet_j5_param.output_tensor_list(i);
    const auto input_tensor = net->GetOutputByName(tensor_info.name());
    // Skip batch dim.
    for (int j = 1; j < tensor_info.shape_size(); ++j) {
      EXPECT_EQ(tensor_info.shape(j), input_tensor->shape(j));
    }
  }

  const auto batch_inputs =
      GenUniformDistributedInputByNetParam(actnet_j5_param);
  for (int i = 0; i < batch_inputs.size(); ++i) {
    const auto& name = actnet_j5_param.input_tensor_list(i).name();
    float* data_ptr = net->GetInputByName(name)->GetDataPtr<float>();
    std::memcpy(data_ptr, batch_inputs[i].data(),
                batch_inputs[i].size() * sizeof(float));
  }
  const int max_batch_size = actnet_j5_param.max_batch_size();
  const auto run_res = net->Run(max_batch_size);
  EXPECT_EQ(run_res.size(), actnet_j5_param.output_tensor_list_size());

  // Reset input tensor.
  net->ResetInputs();
  for (int i = 0; i < batch_inputs.size(); ++i) {
    const auto& name = actnet_j5_param.input_tensor_list(i).name();
    float* data_ptr = net->GetInputByName(name)->GetDataPtr<float>();
    EXPECT_EQ(data_ptr[i], 0.0f);
  }
}
#endif
// see https://developer.horizon.ai/forumDetail/118364000835765837
TEST(PredictionJ5QNNTest, ValidShape2AlignedShapeTest) {
  // sample 1
  std::vector<int> valid_shape = {1, 19, 19, 425};
  std::vector<int> aligned_shape_gt = {1, 19, 19, 448};
  std::vector<int> aligned_shape =
      ValidShape2AlignedShape(valid_shape, qnn::DataType::kInt32);
  CHECK_EQ(aligned_shape.size(), aligned_shape_gt.size());
  for (int i = 0; i < aligned_shape.size(); ++i) {
    EXPECT_EQ(aligned_shape[i], aligned_shape_gt[i]);
  }
  // sample 2
  valid_shape = {1, 19, 19, 425};
  aligned_shape_gt = {1, 19, 19, 512};
  aligned_shape = ValidShape2AlignedShape(valid_shape, qnn::DataType::kInt8);
  CHECK_EQ(aligned_shape.size(), aligned_shape_gt.size());
  for (int i = 0; i < aligned_shape.size(); ++i) {
    EXPECT_EQ(aligned_shape[i], aligned_shape_gt[i]);
  }
  // sample 3
  valid_shape = {1, 19, 19, 425};
  aligned_shape_gt = {1, 19, 19, 432};
  aligned_shape = ValidShape2AlignedShape(valid_shape, qnn::DataType::kInt64);
  CHECK_EQ(aligned_shape.size(), aligned_shape_gt.size());
  for (int i = 0; i < aligned_shape.size(); ++i) {
    EXPECT_EQ(aligned_shape[i], aligned_shape_gt[i]);
  }
  // sample 4 actnetj5 input
  valid_shape = {-1, 20, 1, 160};
  aligned_shape_gt = {-1, 20, 1, 256};
  aligned_shape = ValidShape2AlignedShape(valid_shape, qnn::DataType::kInt8);
  CHECK_EQ(aligned_shape.size(), aligned_shape_gt.size());
  for (int i = 0; i < aligned_shape.size(); ++i) {
    EXPECT_EQ(aligned_shape[i], aligned_shape_gt[i]);
  }
  // sample 5 actnetj5 input
  valid_shape = {-1, 3, 1, 1};
  aligned_shape_gt = {-1, 3, 1, 16};
  aligned_shape = ValidShape2AlignedShape(valid_shape, qnn::DataType::kInt8);
  CHECK_EQ(aligned_shape.size(), aligned_shape_gt.size());
  for (int i = 0; i < aligned_shape.size(); ++i) {
    EXPECT_EQ(aligned_shape[i], aligned_shape_gt[i]);
  }
  // sample 6 actnetj5 output
  valid_shape = {-1, 401, 1, 3};
  aligned_shape_gt = {-1, 401, 1, 4};
  aligned_shape = ValidShape2AlignedShape(valid_shape, qnn::DataType::kInt32);
  CHECK_EQ(aligned_shape.size(), aligned_shape_gt.size());
  for (int i = 0; i < aligned_shape.size(); ++i) {
    EXPECT_EQ(aligned_shape[i], aligned_shape_gt[i]);
  }
  // sample 7 actnetj5 output
  valid_shape = {-1, 1, 1, 1};
  aligned_shape_gt = {-1, 1, 1, 4};
  aligned_shape = ValidShape2AlignedShape(valid_shape, qnn::DataType::kInt32);
  CHECK_EQ(aligned_shape.size(), aligned_shape_gt.size());
  for (int i = 0; i < aligned_shape.size(); ++i) {
    EXPECT_EQ(aligned_shape[i], aligned_shape_gt[i]);
  }
  // sample 8
  valid_shape = {-1, 401, 1, 4};
  aligned_shape_gt = {-1, 401, 1, 4};
  aligned_shape = ValidShape2AlignedShape(valid_shape, qnn::DataType::kInt32);
  CHECK_EQ(aligned_shape.size(), aligned_shape_gt.size());
  for (int i = 0; i < aligned_shape.size(); ++i) {
    EXPECT_EQ(aligned_shape[i], aligned_shape_gt[i]);
  }
}
}  // namespace
}  // namespace prediction
}  // namespace qcraft
