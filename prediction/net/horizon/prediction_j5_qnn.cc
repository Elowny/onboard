#include "onboard/prediction/net/horizon/prediction_j5_qnn.h"

#include <ostream>
#include <vector>

#include "onboard/global/trace.h"  // for SCOPED_QTRACE, ScopedTrace
#include "onboard/lite/check.h"    // for QCHECK
#include "onboard/lite/logging.h"
#include "onboard/lite/qissue_trans.h"
#include "onboard/nets/qnn/qnn/status.h"
#include "onboard/nets/qnn/qnn/tensor.h"
#include "onboard/params/param_converter.h"
#include "onboard/proto/q_issue.pb.h"

namespace qcraft {
namespace prediction {

namespace {
constexpr int kAlignedUnit = 256;
const std::vector<int> kAlignedBytesRuler{0, 16, 32, 64, 128, 256};
int SizeofDatatype(qnn::DataType data_type) {
  int data_type_size = -1;
  switch (data_type) {
    case qnn::DataType::kInt64:
      data_type_size = 8;
      break;
    case qnn::DataType::kFP32:
    case qnn::DataType::kInt32:
      data_type_size = 4;
      break;
    case qnn::DataType::kFP16:
    case qnn::DataType::kInt16:
    case qnn::DataType::kUInt16:
      data_type_size = 2;
      break;
    case qnn::DataType::kInt8:
    case qnn::DataType::kUInt8:
      data_type_size = 1;
      break;
    default:
      QLOG(FATAL) << "Fatal: unknown data type " << static_cast<int>(data_type);
      break;
  }
  return data_type_size;
}
}  // namespace
// Convert valid shape to aligned shape.
// see https://developer.horizon.ai/forumDetail/118364000835765837
std::vector<int> ValidShape2AlignedShape(const std::vector<int>& valid_shape,
                                         qnn::DataType data_type) {
  QCHECK_EQ(valid_shape.size(), 4);
  std::vector<int> aligned_shape = valid_shape;
  int valid_bytes = valid_shape[3] * SizeofDatatype(data_type);
  int padding_bytes = valid_bytes % kAlignedUnit;
  for (int it : kAlignedBytesRuler) {
    if (it >= padding_bytes) {
      padding_bytes = it - padding_bytes;
      break;
    }
  }
  aligned_shape[3] = (valid_bytes + padding_bytes) / SizeofDatatype(data_type);
  return aligned_shape;
}

PredictionJ5QNN::PredictionJ5QNN(const NetParam& net_param_origin) {
#if (defined(Q_CPU_ONLY) && defined(__X86_64__)) || defined(__J5__)
  // Use hbdnn on J5 and x86 sim hbdnn.
  NetParam net_param = net_param_origin;
  QLOG(INFO) << "Note: Force to use hbdnn backend on J5 and x86 sim hbdnn!";
  net_param.set_device_type(NetParam_DeviceType::NetParam_DeviceType_BPU);
  net_param.set_backend_type(NetParam_BackendType::NetParam_BackendType_HBDNN);
#elif defined(Q_CPU_ONLY)
  NetParam net_param = net_param_origin;
  net_param.set_device_type(NetParam_DeviceType::NetParam_DeviceType_CPU);
  QCHECK(net_param.backend_type() !=
         NetParam_BackendType::NetParam_BackendType_TENSORRT);
#else
  const NetParam& net_param = net_param_origin;
#endif
  config_ = param_converter::SetNetConfig(net_param);

  if (config_.backend == qnn::BackendType::kTensorrt) {
    is_dynamic_batch_ = true;
    QLOG(INFO) << "Use dynamic batch for TENSORRT.";
  } else {
    // NOTE(yinbao): only TENSORRT support dynamic batch.
    QLOG(INFO) << "Do not use dynamic batch for ONNX/LIBTORCH/HBDNN.";
    for (int i = 0; i < config_.model.input_infos.size(); ++i) {
      config_.model.input_infos[i].dynamic = false;
      std::vector<int>& shape = config_.model.input_infos[i].shape;
      if (shape.size() == 0) {
        const auto& max_shape = config_.model.input_infos[i].max_shape;
        shape.assign(max_shape.begin(), max_shape.end());
      }
    }
    for (int i = 0; i < config_.model.output_infos.size(); ++i) {
      config_.model.output_infos[i].dynamic = false;
      config_.model.output_infos[i].is_cached = true;
      std::vector<int>& shape = config_.model.output_infos[i].shape;
      if (shape.size() == 0) {
        const auto& max_shape = config_.model.output_infos[i].max_shape;
        shape.assign(max_shape.begin(), max_shape.end());
      }
    }
  }

#if (defined(Q_CPU_ONLY) && defined(__X86_64__)) || defined(__J5__)
  // Convert valid shape to aligned shape
  QLOG(INFO)
      << "Note: Covert valid shape to aligned shape on J5 and x86 sim hbdnn!";
  for (auto& input_info : config_.model.input_infos) {
    input_info.shape =
        ValidShape2AlignedShape(input_info.shape, qnn::DataType::kInt8);
  }
  for (auto& output_info : config_.model.output_infos) {
    output_info.shape =
        ValidShape2AlignedShape(output_info.shape, qnn::DataType::kInt32);
  }
#endif

  net_ = qnn::QNN::Create(config_);
  QCHECK(net_ != nullptr);

  qnn::SessionConfig sess_config;
  session_ = net_->CreateSession(sess_config);
  QCHECK(session_ != nullptr);

  // For hbnn qat
  map_input_tensor_.reserve(net_param.input_tensor_list_size());
  for (int i = 0; i < net_param.input_tensor_list_size(); ++i) {
    const auto& input_tensor = net_param.input_tensor_list(i);
    map_input_tensor_[input_tensor.name()] =
        std::make_shared<HorizonTensorWrapper>(
            net_->GetSessionInput(session_, i));
    map_input_tensor_[input_tensor.name()]->zero_memory();
  }
  map_output_tensor_.reserve(net_param.output_tensor_list_size());
  for (int i = 0; i < net_param.output_tensor_list_size(); ++i) {
    const auto& output_tensor = net_param.output_tensor_list(i);
    map_output_tensor_[output_tensor.name()] =
        std::make_shared<HorizonTensorWrapper>(
            net_->GetSessionOutput(session_, i));
    map_output_tensor_[output_tensor.name()]->zero_memory();
  }
}

PredictionJ5QNN::~PredictionJ5QNN() {
  if (net_ != nullptr && session_ != nullptr) {
    net_->ReleaseSession(session_);
    net_->Destroy();
  } else if (net_ != nullptr && session_ == nullptr) {
    net_->Destroy();
  }
}

void PredictionJ5QNN::PreProcess() {
#ifndef Q_CPU_ONLY
  if (config_.backend == qnn::BackendType::kTensorrt) {
    for (int i = 0; i < config_.model.input_infos.size(); ++i) {
      net_->GetSessionInput(session_, i)->HostToDevice();
    }
  }
#endif
}

void PredictionJ5QNN::PostProcess() {
#ifndef Q_CPU_ONLY
  if (config_.backend == qnn::BackendType::kTensorrt) {
    for (int i = 0; i < config_.model.output_infos.size(); ++i) {
      net_->GetSessionOutput(session_, i)->DeviceToHost();
    }
  }
#endif
}

std::unordered_map<std::string, std::shared_ptr<HorizonTensorWrapper>>
PredictionJ5QNN::Run(int num_objs) {
  {
    SCOPED_QTRACE("PredictionJ5QNN::Qantize Inputs.");
    PreProcess();
  }
  {
    SCOPED_QTRACE_ARG1("PredictionJ5QNN::Infer", "device_ID",
                       config_.device_id);
    // NOTE(yinbao): hbnn and onnx Backend does not suppport dynamic batch now.
    std::vector<std::vector<int>> shape = {};
    // NOTE(yinbao): onnx Backend is not suppport dynamic batch now.
    if (is_dynamic_batch_) {
      shape.resize(config_.model.input_infos.size());
      for (int i = 0; i < shape.size(); ++i) {
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
    SCOPED_QTRACE("PredictionJ5QNN::Deqantize Outputs.");
    PostProcess();
  }
  return map_output_tensor_;
}

}  // namespace prediction
}  // namespace qcraft
