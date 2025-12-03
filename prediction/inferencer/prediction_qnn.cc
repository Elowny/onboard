#include "onboard/prediction/inferencer/prediction_qnn.h"

#include <numeric>
#include <utility>

#include "onboard/global/trace.h"
#include "onboard/lite/logging.h"
#include "onboard/lite/qissue_trans.h"
#include "onboard/nets/qnn/qnn/status.h"

#ifndef Q_CPU_ONLY
#include <cuda_runtime.h>

#include <functional>

#include "onboard/nets/qcv/cuda_device_guard.h"
#endif

namespace qcraft {
namespace prediction {
namespace {
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
      opt_shape[0] = config.model.max_batch / 2;
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
    // hack for cutin net (lixin)
#ifdef Q_CPU_ONLY
    if (output_tensors[i].name() == "457") {
      continue;
    }
#endif
    if (net_param.device_type() == NetParam_DeviceType_CPU) {
      if (output_tensors[i].name() == "457") {
        continue;
      }
    }
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
}  // namespace

PredictionQNN::PredictionQNN(const std::string& model_path,
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

PredictionQNN::~PredictionQNN() {
  if (net_ != nullptr && session_ != nullptr) {
    net_->ReleaseSession(session_);
    net_->Destroy();
  } else if (net_ != nullptr && session_ == nullptr) {
    net_->Destroy();
  }
}

std::map<std::string, std::vector<float>> PredictionQNN::Run(
    const std::vector<std::vector<float>>& batch, int num_objs) {
#ifndef Q_CPU_ONLY
  qcv::cuda::CudaDeviceGuard guard(config_.device_id);
#endif
  {
    SCOPED_QTRACE("PredictionQNN::SetInputs");
    SetInputs(batch);
  }

  {
    SCOPED_QTRACE("PredictionQNN::Infer");
    std::vector<std::vector<int>> shape = {};
    // NOTE(yinbao): onnx Backend is not suppport dynamic batch now.
    if (is_cuda_) {
      shape.resize(batch.size());
      for (size_t i = 0; i < shape.size(); ++i) {
        shape[i] = config_.model.input_infos[i].opt_shape;
        shape[i][0] = num_objs;
      }
    }
    auto status = net_->RunSession(session_, shape);
    if (status != qnn::StatusCode::kQnnOk) {
      QISSUEX(QIssueSeverity::QIS_FATAL, QIssueType::QIT_EXCEPTION,
              QIssueSubType::QIST_PREDICTION,
              "QNN ERROR with status " + status.Description());
    }
  }

  {
    SCOPED_QTRACE("PredictionQNN::ExtractOutputs");
    return ExtractOutputs(num_objs);
  }
}

void PredictionQNN::SetInputs(const std::vector<std::vector<float>>& batch) {
  for (size_t i = 0; i < config_.model.input_infos.size(); ++i) {
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

std::map<std::string, std::vector<float>> PredictionQNN::ExtractOutputs(
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

}  // namespace prediction
}  // namespace qcraft
