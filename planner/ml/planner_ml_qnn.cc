#include "onboard/planner/ml/planner_ml_qnn.h"

#include <numeric>
#include <utility>

#include "onboard/global/trace.h"
#include "onboard/lite/logging.h"
#ifndef Q_CPU_ONLY
#include <cuda_runtime.h>

#include <functional>

#include "onboard/nets/qcv/cuda_device_guard.h"
#endif

namespace qcraft {
namespace planner {
namespace ml {
qnn::NetworkConfig SetNetConfig(const std::string& model_path,
                                const NetParam& net_param, bool use_cuda) {
  qnn::NetworkConfig config;
  config.model.paths.push_back(model_path);
  config.device_id = static_cast<int>(net_param.device_id());
  config.model.model_type = qnn::ModelType::kOnnx;
  config.model.max_batch = net_param.max_batch_size();
  if (use_cuda && net_param.device_type() == NetParam_DeviceType_GPU) {
    config.backend = qnn::BackendType::kTensorrt;
    config.device = qnn::DeviceType::kCuda;
  } else {
    config.backend = qnn::BackendType::kOnnx;
    config.device = qnn::DeviceType::kCpu;
  }
  if (net_param.precision_type() == NetParam_PrecisionType_FP16) {
    config.precision = qnn::PrecisionType::kFP16;
  } else if (net_param.precision_type() == NetParam_PrecisionType_INT8) {
    config.precision = qnn::PrecisionType::kInt8;
  }
  const int input_size = net_param.input_tensor_list_size();
  const int output_size = net_param.output_tensor_list_size();
  config.model.input_infos.reserve(input_size);
  const auto& input_tensors = net_param.input_tensor_list();
  bool dynamic = input_tensors[0].shape(0) == -1;
  for (int i = 0; i < input_size; ++i) {
    qnn::ModelIOInfo input_info = {.name = input_tensors[i].name(),
                                   .data_type = qnn::DataType::kFP32,
                                   .dynamic = dynamic};
    if (dynamic) {
      std::vector<int> min_shape(input_tensors[i].shape().begin(),
                                 input_tensors[i].shape().end());
      min_shape[0] = 1;
      std::vector<int> opt_shape(min_shape);
      opt_shape[0] = config.model.max_batch;
      std::vector<int> max_shape(min_shape);
      max_shape[0] = config.model.max_batch;
      input_info.min_shape = std::move(min_shape);
      input_info.opt_shape = std::move(opt_shape);
      input_info.max_shape = std::move(max_shape);
    } else {
      std::vector<int> shape(input_tensors[i].shape().begin(),
                             input_tensors[i].shape().end());
      input_info.shape = std::move(shape);
    }
    config.model.input_infos.push_back(std::move(input_info));
  }

  const auto& output_tensors = net_param.output_tensor_list();
  config.model.output_infos.reserve(output_size);
  for (int i = 0; i < output_size; ++i) {
    qnn::ModelIOInfo output_info = {.name = output_tensors[i].name(),
                                    .data_type = qnn::DataType::kFP32,
                                    .dynamic = dynamic};
    if (dynamic) {
      std::vector<int> min_shape(output_tensors[i].shape().begin(),
                                 output_tensors[i].shape().end());
      min_shape[0] = 1;
      std::vector<int> opt_shape(min_shape);
      opt_shape[0] = config.model.max_batch / 2;
      std::vector<int> max_shape(min_shape);
      max_shape[0] = config.model.max_batch;
      output_info.min_shape = std::move(min_shape);
      output_info.opt_shape = std::move(opt_shape);
      output_info.max_shape = std::move(max_shape);
    } else {
      std::vector<int> shape(output_tensors[i].shape().begin(),
                             output_tensors[i].shape().end());

      output_info.shape = std::move(shape);
    }
    config.model.output_infos.push_back(std::move(output_info));
  }
  return config;
}
}  // namespace ml

ml::PlannerMLQNN::PlannerMLQNN(const std::string& model_path,
                               const NetParam& net_param) {
#ifndef Q_CPU_ONLY
  is_cuda_ = net_param.device_type() == NetParam_DeviceType_GPU;
#endif
  config_ = SetNetConfig(model_path, net_param, is_cuda_);
#ifndef Q_CPU_ONLY
  qcv::cuda::CudaDeviceGuard guard(config_.device_id);
  cudaSetDeviceFlags(cudaDeviceScheduleBlockingSync);
#endif

  net_ = qnn::QNN::Create(config_);
  QCHECK(net_ != nullptr);

  qnn::SessionConfig sess_config;
  session_ = net_->CreateSession(sess_config);
  QCHECK(session_ != nullptr);
}

ml::PlannerMLQNN::~PlannerMLQNN() {
  if (net_ != nullptr && session_ != nullptr) {
    net_->ReleaseSession(session_);
    net_->Destroy();
  } else if (net_ != nullptr && session_ == nullptr) {
    net_->Destroy();
  }
}

std::map<std::string, std::vector<float>> ml::PlannerMLQNN::Run(
    const std::vector<std::vector<float>>& batch, int num_objs) {
#ifndef Q_CPU_ONLY
  qcv::cuda::CudaDeviceGuard guard(config_.device_id);
#endif
  {
    SCOPED_QTRACE("PlannerMLQNN::SetInputs");
    SetInputs(batch);
  }

  {
    SCOPED_QTRACE("PlannerMLQNN::Infer");
    net_->RunSession(session_);
  }

  {
    SCOPED_QTRACE("PlannerMLQNN::ExtractOutputs");
    return ExtractOutputs(num_objs);
  }
}

void ml::PlannerMLQNN::SetInputs(const std::vector<std::vector<float>>& batch) {
  for (int i = 0; i < config_.model.input_infos.size(); ++i) {
    std::shared_ptr<qnn::Tensor> input_tensor =
        net_->GetSessionInput(session_, i);
    float* input_ptr = input_tensor->HostMutableData<float>();
    std::memcpy(input_ptr, batch[i].data(), batch[i].size() * sizeof(float));
#ifndef Q_CPU_ONLY
    if (is_cuda_) {
      input_tensor->HostToDevice();
    }
#endif
  }
}

std::map<std::string, std::vector<float>> ml::PlannerMLQNN::ExtractOutputs(
    int num_objs) {
  std::map<std::string, std::vector<float>> output_map;
  const auto& output_infos = config_.model.output_infos;
  const int output_size = output_infos.size();
#ifndef Q_CPU_ONLY
  if (is_cuda_) {
    for (int i = 0; i < output_size; ++i) {
      net_->GetSessionOutput(session_, i)->DeviceToHost();
    }
  }
#endif
  for (int i = 0; i < output_size; ++i) {
    const float* cur_ptr =
        net_->GetSessionOutput(session_, i)->HostData<float>();
    const int data_size = std::accumulate(output_infos[i].opt_shape.begin() + 1,
                                          output_infos[i].opt_shape.end(),
                                          num_objs, std::multiplies<int>());
    std::vector<float> out_vec(cur_ptr, cur_ptr + data_size);
    const std::string out_name = output_infos[i].name;
    output_map[out_name] = std::move(out_vec);
  }
  return output_map;
}

}  // namespace planner
}  // namespace qcraft
