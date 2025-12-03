#ifndef ONBOARD_PREDICTION_NET_HORIZON_HORIZON_TENSOR_WRAPPER_H
#define ONBOARD_PREDICTION_NET_HORIZON_HORIZON_TENSOR_WRAPPER_H

#include <algorithm>  // for clamp
#include <cmath>
#include <cstring>  // for memset, size_t
#include <memory>   // for shared_ptr
#include <vector>   // for vector

#ifdef __ARM_NEON
#include <arm_neon.h>
#endif

#include "glog/logging.h"  // for CheckNotNull, CHECK_NOTNULL

#include "onboard/nets/qnn/qnn/common.h"  // for DataType
#include "onboard/nets/qnn/qnn/tensor.h"  // for Tensor

namespace qcraft {
namespace prediction {

constexpr int kMinINT8Value = -128;
constexpr int kMaxINT8Value = 127;
constexpr int kMinINT32Value = -2147483648;
constexpr int kMaxINT32Value = 2147483647;

class HorizonTensorWrapper final {
 public:
  explicit HorizonTensorWrapper(const std::shared_ptr<qnn::Tensor>& tensor);
  ~HorizonTensorWrapper();
  __attribute__((always_inline)) inline void zero_memory() {
    CHECK_NOTNULL(data_ptr_);
    std::memset(data_ptr_, 0, byte_size_);
  }

#if (defined(Q_CPU_ONLY) && defined(__X86_64__)) || defined(__J5__)
  // Do qat in channel dim. b n w h, inputs are always int8 in J5 and x86 sim
  // hbdnn.
  void index_put_qat(int batch, int channel, int h, int w, float value) {
    const int index =
        batch * stride_[0] + channel * stride_[1] + h * stride_[2] + w;
    *(reinterpret_cast<char*>(data_ptr_) + index) = static_cast<char>(
        std::clamp(static_cast<int>(std::round(value * scale_inv_[channel])),
                   kMinINT8Value, kMaxINT8Value));
  }

#ifdef __ARM_NEON
  __attribute__((always_inline)) inline void index_put_qat_neon64(
      int batch, int channel, int h, int w, const float* value) {
    const int index0 =
        batch * stride_[0] + channel * stride_[1] + h * stride_[2] + w;
    auto _kMinINT8 = vdup_n_f32(kMinINT8Value);
    auto _kMaxINT8 = vdup_n_f32(kMaxINT8Value);
    const int index1 = index0 + stride_[1];
    float32x2_t _v = vmul_f32(vld1_f32(value), vld1_f32(&scale_inv_[channel]));
    float32x2_t _clamp = vmax_f32(_kMinINT8, vmin_f32(_kMaxINT8, _v));
    *(reinterpret_cast<char*>(data_ptr_) + index0) = vget_lane_f32(_clamp, 0);
    *(reinterpret_cast<char*>(data_ptr_) + index1) = vget_lane_f32(_clamp, 1);
  }

  __attribute__((always_inline)) inline void index_put_qat_neon128(
      int batch, int channel, int h, int w, const float* value) {
    const int index0 =
        batch * stride_[0] + channel * stride_[1] + h * stride_[2] + w;
    auto _kMinINT8 = vdupq_n_f32(kMinINT8Value);
    auto _kMaxINT8 = vdupq_n_f32(kMaxINT8Value);
    const int index2 = index0 + (stride_[1] << 1);
    const int index1 = index0 + stride_[1];
    float32x4_t _v =
        vmulq_f32(vld1q_f32(value), vld1q_f32(&scale_inv_[channel]));
    float32x4_t _clamp = vmaxq_f32(_kMinINT8, vminq_f32(_kMaxINT8, _v));
    const int index3 = index2 + stride_[1];
    *(reinterpret_cast<char*>(data_ptr_) + index0) = vgetq_lane_f32(_clamp, 0);
    *(reinterpret_cast<char*>(data_ptr_) + index1) = vgetq_lane_f32(_clamp, 1);
    *(reinterpret_cast<char*>(data_ptr_) + index2) = vgetq_lane_f32(_clamp, 2);
    *(reinterpret_cast<char*>(data_ptr_) + index3) = vgetq_lane_f32(_clamp, 3);
  }
#endif
#else
  // Do qat in channel dim. b n w h
  void index_put_qat(int batch, int channel, int h, int w, float value) {
    const int index = batch * stride_[0] + channel * stride_[1] +
                      h * stride_[2] + w * stride_[3];
    switch (data_type_) {
      case qnn::DataType::kInt8:
        *(reinterpret_cast<char*>(data_ptr_) + index) =
            static_cast<char>(std::clamp(
                static_cast<int>(std::round(value * scale_inv_[channel])),
                kMinINT8Value, kMaxINT8Value));
        break;
      case qnn::DataType::kInt32:
        *(reinterpret_cast<int*>(data_ptr_) + index) = std::clamp(
            static_cast<int>(std::round(value * scale_inv_[channel])),
            kMinINT32Value, kMaxINT32Value);
        break;
      case qnn::DataType::kFP32:
      default:
        *(reinterpret_cast<float*>(data_ptr_) + index) =
            value * scale_inv_[channel];
        break;
    }
  }
#endif

#if (defined(Q_CPU_ONLY) && defined(__X86_64__)) || defined(__J5__)
  // Do deqat in channel dim. b n w h, outputs are always int32 in J5 and x86
  // sim hbdnn.
  float index_get_deqat(int batch, int channel, int h, int w) const {
    const int index =
        batch * stride_[0] + channel * stride_[1] + h * stride_[2] + w;
    return *(reinterpret_cast<int*>(data_ptr_) + index) * scale_[channel];
  }
#else
  // Do deqat in channel dim. b n w h
  float index_get_deqat(int batch, int channel, int h, int w) const {
    const int index = batch * stride_[0] + channel * stride_[1] +
                      h * stride_[2] + w * stride_[3];
    switch (data_type_) {
      case qnn::DataType::kInt32:
        return *(reinterpret_cast<int*>(data_ptr_) + index) * scale_[channel];
      case qnn::DataType::kInt8:
        return *(reinterpret_cast<char*>(data_ptr_) + index) * scale_[channel];
      case qnn::DataType::kFP32:
      default:
        return *(reinterpret_cast<float*>(data_ptr_) + index) * scale_[channel];
    }
  }
#endif

  int shape(int dim) { return shape_[dim]; }
  int stride(int dim) { return stride_[dim]; }

  template <typename T>
  T* GetDataPtr() {
    return reinterpret_cast<T*>(data_ptr_);
  }

  size_t byte_size() { return byte_size_; }
  size_t size() {
    switch (data_type_) {
      case qnn::DataType::kInt32:
        return byte_size_ / sizeof(int);
      case qnn::DataType::kInt8:
        return byte_size_ / sizeof(char);
      case qnn::DataType::kFP32:
      default:
        return byte_size_ / sizeof(float);
    }
  }

 private:
  void* data_ptr_;
  std::vector<int> shape_;
  std::vector<int> stride_;

  size_t byte_size_ = 0;
  qnn::DataType data_type_;

  // scale for qat
  std::vector<float> scale_inv_;
  std::vector<float> scale_;
};

}  // namespace prediction
}  // namespace qcraft
#endif  // ONBOARD_PREDICTION_NET_HORIZON_HORIZON_TENSOR_WRAPPER_H
