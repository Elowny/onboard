#include "onboard/prediction/net/horizon/horizon_tensor_wrapper.h"

#include <cmath>    // for round
#include <ostream>  // for operator<<

#include "gtest/gtest.h"  // for Message, TestPartResult, ASSERT_EQ, Test, AssertionResult, TestInfo, ASSER...

#include "onboard/lite/logging.h"  // for BufferedLoggerWrapper, QLOG, QLOG_WITH_MODULE_INFO
#include "onboard/nets/qnn/qnn/common.h"  // for DataFormat, DataFormat::kNCHW, DataType, DataType::kInt32, DataType::kInt8
#include "onboard/nets/qnn/qnn/tensor.h"  // for QTensor, Tensor

namespace qcraft {
namespace prediction {

#if (defined(Q_CPU_ONLY) && defined(__X86_64__)) || defined(__J5__)
TEST(HorizonTensorWrapper, CreateTensorInt8OnJ5Test) {
  hbDNNTensor input_tensor;
  hbSysAllocMem(&input_tensor.sysMem[0], sizeof(char) * 2 * 3 * 4);
  std::shared_ptr<qnn::Tensor> tensor = qnn::Tensor::CreateJ5(
      {1, 2, 3, 4}, qnn::DataType::kInt8, qnn::DataFormat::kNCHW, input_tensor);
  ASSERT_TRUE(tensor != nullptr);
  float scale[2] = {1.0f, 2.0f};
  tensor->SetScale(scale, 2);

  HorizonTensorWrapper tensor_wrapper = HorizonTensorWrapper(tensor);
  tensor_wrapper.zero_memory();

  ASSERT_EQ(tensor_wrapper.shape(0), 1);
  ASSERT_EQ(tensor_wrapper.shape(1), 2);
  ASSERT_EQ(tensor_wrapper.shape(2), 3);
  ASSERT_EQ(tensor_wrapper.shape(3), 4);
  ASSERT_EQ(tensor_wrapper.stride(0), 2 * 3 * 4);
  ASSERT_EQ(tensor_wrapper.stride(1), 3 * 4);
  ASSERT_EQ(tensor_wrapper.stride(2), 4);
  ASSERT_EQ(tensor_wrapper.stride(3), 1);
  ASSERT_EQ(tensor_wrapper.GetDataPtr<char>(), tensor->HostMutableData<char>());
  // Note(yinbao): Qat and set  only implemented for int8 on J5
  const float step = 10.0f;
  for (int n = 0; n < tensor->Batch(); n++) {
    for (int c = 0; c < tensor->Channel(); c++) {
      for (int h = 0; h < tensor->Height(); h++) {
        for (int w = 0; w < tensor->Width(); w++) {
          const int idx =
              n * tensor->Channel() * tensor->Height() * tensor->Width() +
              c * tensor->Height() * tensor->Width() + h * tensor->Width() + w;
          tensor_wrapper.index_put_qat(n, c, h, w, step * idx);
        }
      }
    }
  }
  const char* data_ptr = tensor_wrapper.GetDataPtr<char>();
  const char* data_ptr_1 = tensor->HostData<char>();
  for (int i = 0; i < tensor_wrapper.byte_size() / sizeof(char); ++i) {
    ASSERT_EQ(data_ptr_1[i], data_ptr[i]);
  }
  // set zeros
  tensor_wrapper.zero_memory();
  for (int i = 0; i < tensor_wrapper.byte_size() / sizeof(char); ++i) {
    ASSERT_EQ(data_ptr[i], 0);
  }
  QLOG(INFO) << "HorizonTensorWrapper::CreateInt8TensorOnJ5Test end.";
}
TEST(HorizonTensorWrapper, CreateTensorInt32OnJ5Test) {
  hbDNNTensor input_tensor;
  hbSysAllocMem(&input_tensor.sysMem[0], sizeof(int) * 2 * 3 * 4);
  std::shared_ptr<qnn::Tensor> tensor =
      qnn::Tensor::CreateJ5({1, 2, 3, 4}, qnn::DataType::kInt32,
                            qnn::DataFormat::kNCHW, input_tensor);
  ASSERT_TRUE(tensor != nullptr);
  float scale[2] = {1.0f, 2.0f};
  tensor->SetScale(scale, 2);

  HorizonTensorWrapper tensor_wrapper = HorizonTensorWrapper(tensor);
  tensor_wrapper.zero_memory();

  ASSERT_EQ(tensor_wrapper.shape(0), 1);
  ASSERT_EQ(tensor_wrapper.shape(1), 2);
  ASSERT_EQ(tensor_wrapper.shape(2), 3);
  ASSERT_EQ(tensor_wrapper.shape(3), 4);
  ASSERT_EQ(tensor_wrapper.stride(0), 2 * 3 * 4);
  ASSERT_EQ(tensor_wrapper.stride(1), 3 * 4);
  ASSERT_EQ(tensor_wrapper.stride(2), 4);
  ASSERT_EQ(tensor_wrapper.stride(3), 1);
  ASSERT_EQ(tensor_wrapper.GetDataPtr<int>(), tensor->HostMutableData<int>());
  // Note(yinbao): Deqat only implemented for int32 on J5.
  int* data_ptr = tensor_wrapper.GetDataPtr<int>();
  const int* data_ptr_1 = tensor->HostData<int>();
  const float step = 10.0f;

  // Hack: set data to ptr.
  for (int n = 0; n < tensor->Batch(); n++) {
    for (int c = 0; c < tensor->Channel(); c++) {
      for (int h = 0; h < tensor->Height(); h++) {
        for (int w = 0; w < tensor->Width(); w++) {
          const int idx =
              n * tensor->Channel() * tensor->Height() * tensor->Width() +
              c * tensor->Height() * tensor->Width() + h * tensor->Width() + w;
          *(data_ptr + idx) = std::clamp((idx * step) / tensor->GetScale()[c],
                                         kMinINT32Value, kMaxINT32Value);
        }
      }
    }
  }

  for (int i = 0; i < tensor_wrapper.byte_size() / sizeof(int); ++i) {
    ASSERT_EQ(data_ptr_1[i], data_ptr[i]);
  }

  // Deqat: Only int32 implementation for J5
  for (int n = 0; n < tensor->Batch(); n++) {
    for (int c = 0; c < tensor->Channel(); c++) {
      for (int h = 0; h < tensor->Height(); h++) {
        for (int w = 0; w < tensor->Width(); w++) {
          const int idx =
              n * tensor->Channel() * tensor->Height() * tensor->Width() +
              c * tensor->Height() * tensor->Width() + h * tensor->Width() + w;
          ASSERT_EQ(tensor_wrapper.index_get_deqat(n, c, h, w),
                    std::round(std::clamp(idx * step / tensor->GetScale()[c],
                                          kMinINT32Value, kMaxINT32Value)) *
                        tensor->GetScale()[c]);
        }
      }
    }
  }
  // set zeros
  tensor_wrapper.zero_memory();
  for (int i = 0; i < tensor_wrapper.byte_size() / sizeof(int); ++i) {
    ASSERT_EQ(data_ptr[i], 0);
  }
  QLOG(INFO) << "HorizonTensorWrapper::CreateInt32TensorOnJ5Test end.";
}
#else
TEST(HorizonTensorWrapper, CreateInt8TensorTest) {
  std::shared_ptr<qnn::Tensor> tensor = qnn::Tensor::CreateOnHost(
      {1, 2, 3, 4}, qnn::DataType::kInt8, qnn::DataFormat::kNCHW);
  ASSERT_TRUE(tensor != nullptr);
  float scale[2] = {1.0f, 2.0f};
  tensor->SetScale(scale, 2);

  HorizonTensorWrapper tensor_wrapper = HorizonTensorWrapper(tensor);

  ASSERT_EQ(tensor_wrapper.shape(0), 1);
  ASSERT_EQ(tensor_wrapper.shape(1), 2);
  ASSERT_EQ(tensor_wrapper.shape(2), 3);
  ASSERT_EQ(tensor_wrapper.shape(3), 4);
  ASSERT_EQ(tensor_wrapper.stride(0), 2 * 3 * 4);
  ASSERT_EQ(tensor_wrapper.stride(1), 3 * 4);
  ASSERT_EQ(tensor_wrapper.stride(2), 4);
  ASSERT_EQ(tensor_wrapper.stride(3), 1);
  ASSERT_EQ(tensor_wrapper.GetDataPtr<char>(), tensor->HostMutableData<char>());
  // qat and set data
  const float step = 10.0f;
  for (int n = 0; n < tensor->Batch(); n++) {
    for (int c = 0; c < tensor->Channel(); c++) {
      for (int h = 0; h < tensor->Height(); h++) {
        for (int w = 0; w < tensor->Width(); w++) {
          const int idx =
              n * tensor->Channel() * tensor->Height() * tensor->Width() +
              c * tensor->Height() * tensor->Width() + h * tensor->Width() + w;
          tensor_wrapper.index_put_qat(n, c, h, w, step * idx);
        }
      }
    }
  }
  const char* data_ptr = tensor_wrapper.GetDataPtr<char>();
  const char* data_ptr_1 = tensor->HostData<char>();
  for (int i = 0; i < tensor_wrapper.byte_size() / sizeof(char); ++i) {
    ASSERT_EQ(data_ptr_1[i], data_ptr[i]);
  }
  // deqat
  for (int n = 0; n < tensor->Batch(); n++) {
    for (int c = 0; c < tensor->Channel(); c++) {
      for (int h = 0; h < tensor->Height(); h++) {
        for (int w = 0; w < tensor->Width(); w++) {
          const int idx =
              n * tensor->Channel() * tensor->Height() * tensor->Width() +
              c * tensor->Height() * tensor->Width() + h * tensor->Width() + w;
          ASSERT_EQ(tensor_wrapper.index_get_deqat(n, c, h, w),
                    std::clamp(static_cast<int>(std::round(
                                   idx * step / tensor->GetScale()[c])),
                               kMinINT8Value, kMaxINT8Value) *
                        tensor->GetScale()[c]);
        }
      }
    }
  }
  // set zeros
  tensor_wrapper.zero_memory();
  for (int i = 0; i < tensor_wrapper.byte_size() / sizeof(char); ++i) {
    ASSERT_EQ(data_ptr[i], 0);
  }
  QLOG(INFO) << "HorizonTensorWrapper::CreateInt8TensorTest end.";
}

TEST(HorizonTensorWrapper, CreateInt32TensorTest) {
  std::shared_ptr<qnn::Tensor> tensor = qnn::Tensor::CreateOnHost(
      {1, 2, 3, 4}, qnn::DataType::kInt32, qnn::DataFormat::kNCHW);
  ASSERT_TRUE(tensor != nullptr);
  float scale[2] = {0.1f, 0.2f};
  tensor->SetScale(scale, 2);

  HorizonTensorWrapper tensor_wrapper = HorizonTensorWrapper(tensor);

  ASSERT_EQ(tensor_wrapper.shape(0), 1);
  ASSERT_EQ(tensor_wrapper.shape(1), 2);
  ASSERT_EQ(tensor_wrapper.shape(2), 3);
  ASSERT_EQ(tensor_wrapper.shape(3), 4);
  ASSERT_EQ(tensor_wrapper.stride(0), 2 * 3 * 4);
  ASSERT_EQ(tensor_wrapper.stride(1), 3 * 4);
  ASSERT_EQ(tensor_wrapper.stride(2), 4);
  ASSERT_EQ(tensor_wrapper.stride(3), 1);
  ASSERT_EQ(tensor_wrapper.GetDataPtr<int>(), tensor->HostMutableData<int>());
  // qat and set data
  const float step = 10.0f;
  for (int n = 0; n < tensor->Batch(); n++) {
    for (int c = 0; c < tensor->Channel(); c++) {
      for (int h = 0; h < tensor->Height(); h++) {
        for (int w = 0; w < tensor->Width(); w++) {
          const int idx =
              n * tensor->Channel() * tensor->Height() * tensor->Width() +
              c * tensor->Height() * tensor->Width() + h * tensor->Width() + w;
          tensor_wrapper.index_put_qat(n, c, h, w, step * idx);
        }
      }
    }
  }
  const int* data_ptr = tensor_wrapper.GetDataPtr<int>();
  const int* data_ptr_1 = tensor->HostData<int>();
  for (int i = 0; i < tensor_wrapper.byte_size() / sizeof(int); ++i) {
    ASSERT_EQ(data_ptr_1[i], data_ptr[i]);
  }
  // deqat
  for (int n = 0; n < tensor->Batch(); n++) {
    for (int c = 0; c < tensor->Channel(); c++) {
      for (int h = 0; h < tensor->Height(); h++) {
        for (int w = 0; w < tensor->Width(); w++) {
          const int idx =
              n * tensor->Channel() * tensor->Height() * tensor->Width() +
              c * tensor->Height() * tensor->Width() + h * tensor->Width() + w;
          ASSERT_EQ(tensor_wrapper.index_get_deqat(n, c, h, w),
                    std::clamp(static_cast<int>(std::round(
                                   idx * step / tensor->GetScale()[c])),
                               kMinINT32Value, kMaxINT32Value) *
                        tensor->GetScale()[c]);
        }
      }
    }
  }
  // set zeros
  tensor_wrapper.zero_memory();
  for (int i = 0; i < tensor_wrapper.byte_size() / sizeof(int); ++i) {
    ASSERT_EQ(data_ptr[i], 0);
  }
  QLOG(INFO) << "HorizonTensorWrapper::CreateInt32TensorTest end.";
}

TEST(HorizonTensorWrapper, CreateFP32TensorTest) {
  std::shared_ptr<qnn::Tensor> tensor = qnn::Tensor::CreateOnHost(
      {1, 2, 3, 4}, qnn::DataType::kFP32, qnn::DataFormat::kNCHW);
  ASSERT_TRUE(tensor != nullptr);
  float scale[2] = {1.0f, 2.0f};
  tensor->SetScale(scale, 2);

  HorizonTensorWrapper tensor_wrapper = HorizonTensorWrapper(tensor);

  ASSERT_EQ(tensor_wrapper.shape(0), 1);
  ASSERT_EQ(tensor_wrapper.shape(1), 2);
  ASSERT_EQ(tensor_wrapper.shape(2), 3);
  ASSERT_EQ(tensor_wrapper.shape(3), 4);
  ASSERT_EQ(tensor_wrapper.stride(0), 2 * 3 * 4);
  ASSERT_EQ(tensor_wrapper.stride(1), 3 * 4);
  ASSERT_EQ(tensor_wrapper.stride(2), 4);
  ASSERT_EQ(tensor_wrapper.stride(3), 1);
  ASSERT_EQ(tensor_wrapper.GetDataPtr<float>(),
            tensor->HostMutableData<float>());
  // qat and set data
  const float step = 1e-2;
  for (int n = 0; n < tensor->Batch(); n++) {
    for (int c = 0; c < tensor->Channel(); c++) {
      for (int h = 0; h < tensor->Height(); h++) {
        for (int w = 0; w < tensor->Width(); w++) {
          const int idx =
              n * tensor->Channel() * tensor->Height() * tensor->Width() +
              c * tensor->Height() * tensor->Width() + h * tensor->Width() + w;
          tensor_wrapper.index_put_qat(n, c, h, w, step * idx);
        }
      }
    }
  }
  const float* data_ptr = tensor_wrapper.GetDataPtr<float>();
  const float* data_ptr_1 = tensor->HostData<float>();
  for (int i = 0; i < tensor_wrapper.byte_size() / sizeof(float); ++i) {
    ASSERT_EQ(data_ptr_1[i], data_ptr[i]);
  }
  // deqat
  for (int n = 0; n < tensor->Batch(); n++) {
    for (int c = 0; c < tensor->Channel(); c++) {
      for (int h = 0; h < tensor->Height(); h++) {
        for (int w = 0; w < tensor->Width(); w++) {
          const int idx =
              n * tensor->Channel() * tensor->Height() * tensor->Width() +
              c * tensor->Height() * tensor->Width() + h * tensor->Width() + w;
          ASSERT_EQ(tensor_wrapper.index_get_deqat(n, c, h, w), idx * step);
        }
      }
    }
  }
  // set zeros
  tensor_wrapper.zero_memory();
  for (int i = 0; i < tensor_wrapper.byte_size() / sizeof(int); ++i) {
    ASSERT_EQ(data_ptr[i], 0);
  }
  QLOG(INFO) << "HorizonTensorWrapper::CreateFp32TensorTest end.";
}

TEST(HorizonTensorWrapper, CreateFP32TensorNoScaleTest) {
  std::shared_ptr<qnn::Tensor> tensor = qnn::Tensor::CreateOnHost(
      {1, 2, 3, 4}, qnn::DataType::kFP32, qnn::DataFormat::kNCHW);
  ASSERT_TRUE(tensor != nullptr);

  HorizonTensorWrapper tensor_wrapper = HorizonTensorWrapper(tensor);

  ASSERT_EQ(tensor_wrapper.shape(0), 1);
  ASSERT_EQ(tensor_wrapper.shape(1), 2);
  ASSERT_EQ(tensor_wrapper.shape(2), 3);
  ASSERT_EQ(tensor_wrapper.shape(3), 4);
  ASSERT_EQ(tensor_wrapper.stride(0), 2 * 3 * 4);
  ASSERT_EQ(tensor_wrapper.stride(1), 3 * 4);
  ASSERT_EQ(tensor_wrapper.stride(2), 4);
  ASSERT_EQ(tensor_wrapper.stride(3), 1);
  ASSERT_EQ(tensor_wrapper.GetDataPtr<float>(),
            tensor->HostMutableData<float>());
  // qat and set data
  const float step = 1e-2;
  for (int n = 0; n < tensor->Batch(); n++) {
    for (int c = 0; c < tensor->Channel(); c++) {
      for (int h = 0; h < tensor->Height(); h++) {
        for (int w = 0; w < tensor->Width(); w++) {
          const int idx =
              n * tensor->Channel() * tensor->Height() * tensor->Width() +
              c * tensor->Height() * tensor->Width() + h * tensor->Width() + w;
          tensor_wrapper.index_put_qat(n, c, h, w, step * idx);
        }
      }
    }
  }
  const float* data_ptr = tensor_wrapper.GetDataPtr<float>();
  const float* data_ptr_1 = tensor->HostData<float>();
  for (int i = 0; i < tensor_wrapper.byte_size() / sizeof(float); ++i) {
    ASSERT_EQ(data_ptr_1[i], data_ptr[i]);
  }
  // deqat
  for (int n = 0; n < tensor->Batch(); n++) {
    for (int c = 0; c < tensor->Channel(); c++) {
      for (int h = 0; h < tensor->Height(); h++) {
        for (int w = 0; w < tensor->Width(); w++) {
          const int idx =
              n * tensor->Channel() * tensor->Height() * tensor->Width() +
              c * tensor->Height() * tensor->Width() + h * tensor->Width() + w;
          ASSERT_EQ(tensor_wrapper.index_get_deqat(n, c, h, w), idx * step);
        }
      }
    }
  }
  // set zeros
  tensor_wrapper.zero_memory();
  for (int i = 0; i < tensor_wrapper.byte_size() / sizeof(float); ++i) {
    ASSERT_EQ(data_ptr[i], 0.0f);
  }
  QLOG(INFO) << "HorizonTensorWrapper::CreateFP32TensorNoScaleTest end.";
}
#endif

#if (defined(__ARM_NEON) && defined(__J5__))
TEST(HorizonTensorWrapper, NeonTest) {
  std::shared_ptr<qnn::Tensor> tensor_1 = qnn::Tensor::CreateOnHost(
      {1, 4, 3, 5}, qnn::DataType::kInt8, qnn::DataFormat::kNCHW);
  ASSERT_TRUE(tensor_1 != nullptr);
  HorizonTensorWrapper tensor_wrapper_1 = HorizonTensorWrapper(tensor_1);

  // qat and set data
  for (int n = 0; n < tensor_1->Batch(); n++) {
    for (int c = 0; c < tensor_1->Channel(); c++) {
      for (int h = 0; h < tensor_1->Height(); h++) {
        for (int w = 0; w < tensor_1->Width(); w++) {
          tensor_wrapper_1.index_put_qat(n, c, h, w, c);
        }
      }
    }
  }

  // index_put_qat_neon64
  std::shared_ptr<qnn::Tensor> tensor_2 = qnn::Tensor::CreateOnHost(
      {1, 4, 3, 5}, qnn::DataType::kInt8, qnn::DataFormat::kNCHW);
  ASSERT_TRUE(tensor_2 != nullptr);
  HorizonTensorWrapper tensor_wrapper_2 = HorizonTensorWrapper(tensor_2);

  // qat and set data
  for (int n = 0; n < tensor_2->Batch(); n++) {
    for (int h = 0; h < tensor_2->Height(); h++) {
      for (int w = 0; w < tensor_2->Width(); w++) {
        float values1[2] = {0.0f, 1.0f};
        float values2[2] = {2.0f, 3.0f};
        tensor_wrapper_2.index_put_qat_neon64(n, 0, h, w, values1);
        tensor_wrapper_2.index_put_qat_neon64(n, 2, h, w, values2);
      }
    }
  }

  // index_put_qat_neon128
  std::shared_ptr<qnn::Tensor> tensor_3 = qnn::Tensor::CreateOnHost(
      {1, 4, 3, 5}, qnn::DataType::kInt8, qnn::DataFormat::kNCHW);
  ASSERT_TRUE(tensor_3 != nullptr);
  HorizonTensorWrapper tensor_wrapper_3 = HorizonTensorWrapper(tensor_3);

  // qat and set data
  for (int n = 0; n < tensor_3->Batch(); n++) {
    for (int h = 0; h < tensor_3->Height(); h++) {
      for (int w = 0; w < tensor_3->Width(); w++) {
        float values[4] = {0.0f, 1.0f, 2.0f, 3.0f};
        tensor_wrapper_3.index_put_qat_neon128(n, 0, h, w, values);
      }
    }
  }

  const float* data_ptr_1 = tensor_wrapper_1.GetDataPtr<float>();
  const float* data_ptr_2 = tensor_wrapper_2.GetDataPtr<float>();
  const float* data_ptr_3 = tensor_wrapper_3.GetDataPtr<float>();
  for (int i = 0; i < tensor_wrapper_1.byte_size() / sizeof(char); ++i) {
    ASSERT_EQ(data_ptr_1[i], data_ptr_2[i]);
    ASSERT_EQ(data_ptr_1[i], data_ptr_3[i]);
  }
  QLOG(INFO) << "HorizonTensorWrapper::NeonTest end.";
}
#endif

}  // namespace prediction
}  // namespace qcraft
