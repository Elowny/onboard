#include "onboard/prediction/net/horizon/horizon_tensor_wrapper.h"

#include <cmath>  // for fabs
#include <ostream>  // for char_traits, basic_ostream::operator<<, operator<<, basic_ostream

#include "onboard/lite/logging.h"  // for BufferedLoggerWrapper, QLOG, QLOG_WITH_MODULE_INFO

namespace qcraft {
namespace prediction {
namespace {
constexpr float kFloatEpsilon = 1e-6f;
}

HorizonTensorWrapper::HorizonTensorWrapper(
    const std::shared_ptr<qnn::Tensor>& tensor) {
  data_ptr_ = tensor->HostMutableData<void>();
  shape_.reserve(tensor->Shape().size());
  for (const auto it : tensor->Shape()) {
    shape_.push_back(it);
  }
  int size = 1;
  stride_.resize(shape_.size());
  for (int i = shape_.size() - 1; i >= 0; --i) {
    stride_[i] = size;
    size *= shape_[i];
  }

#if (defined(Q_CPU_ONLY) && defined(__X86_64__)) || defined(__J5__)
  byte_size_ = tensor->HbdnnMutableData().sysMem[0].memSize;
#else
  byte_size_ = tensor->ByteSize();
#endif

  data_type_ = tensor->GetDataType();
  CHECK(data_type_ == qnn::DataType::kInt8 ||
        data_type_ == qnn::DataType::kInt32 ||
        data_type_ == qnn::DataType::kFP32);
  if (tensor->ScaleSize() == 0) {
    // default scale size is equal to channel.
    QLOG(INFO) << "Warning: Use 1.0 as default scale value. Channel size: "
               << tensor->Channel();
    scale_inv_ = std::vector<float>(tensor->Channel(), 1.0f);
    scale_ = std::vector<float>(tensor->Channel(), 1.0f);
  } else {
    // Only consider qat at channel dim.
    CHECK(tensor->ScaleSize() == tensor->Channel());
    scale_inv_.resize(tensor->ScaleSize());
    scale_.resize(tensor->ScaleSize());
    const float* scale_ptr = tensor->GetScale();
    for (int i = 0; i < tensor->ScaleSize(); ++i) {
      const float scale = scale_ptr[i];
      CHECK(std::fabs(scale_ptr[i]) >= kFloatEpsilon);
      scale_inv_[i] = 1.0f / scale;
      scale_[i] = scale;
    }
  }
}

HorizonTensorWrapper::~HorizonTensorWrapper() {}

}  // namespace prediction
}  // namespace qcraft
