#ifndef ONBOARD_PLANNER_ML_CAPTAIN_NET_J5_QNN_H
#define ONBOARD_PLANNER_ML_CAPTAIN_NET_J5_QNN_H

#include <memory>         // for shared_ptr
#include <string>         // for string
#include <unordered_map>  // for unordered_map
#include <utility>        // for pair
#include <vector>

#include "onboard/nets/proto/net_param.pb.h"  // for NetParam
#include "onboard/nets/qnn/qnn/common.h"      // for NetworkConfig
#include "onboard/nets/qnn/qnn/qnn.h"         // for QNN, Session
#include "onboard/planner/ml/captain_net_j5/horizon_tensor_wrapper.h"  // for HorizonTensorWrapper

namespace qcraft::planner::ml {
std::vector<int> ValidShape2AlignedShape(const std::vector<int>& valid_shape,
                                         qnn::DataType data_type);
class PLANNERMLJ5QNN {
 public:
  explicit PLANNERMLJ5QNN(const NetParam& net_param);
  virtual ~PLANNERMLJ5QNN();

  // simplify code.
  std::unordered_map<std::string, std::shared_ptr<HorizonTensorWrapper>> Run(
      int num_objs);

  // maybe use torch tensor
  // Get input tensor data ptr.
  std::shared_ptr<HorizonTensorWrapper> GetInputByName(
      const std::string& name) {
    return map_input_tensor_.at(name);
  }
  // Get output tensor data ptr.
  std::shared_ptr<HorizonTensorWrapper> GetOutputByName(
      const std::string& name) {
    return map_output_tensor_.at(name);
  }
  std::unordered_map<std::string, std::shared_ptr<HorizonTensorWrapper>>
  GetInputMap() {
    return map_input_tensor_;
  }
  std::unordered_map<std::string, std::shared_ptr<HorizonTensorWrapper>>
  GetOutputMap() {
    return map_output_tensor_;
  }

  __attribute__((always_inline)) inline void ResetInputs() {
    for (auto& it : map_input_tensor_) {
      it.second->zero_memory();
    }
  }

 private:
  void PreProcess();
  void PostProcess();

 private:
  std::shared_ptr<qnn::QNN> net_;
  std::shared_ptr<qnn::Session> session_;
  qnn::NetworkConfig config_;
  std::unordered_map<std::string, std::shared_ptr<HorizonTensorWrapper>>
      map_input_tensor_;
  std::unordered_map<std::string, std::shared_ptr<HorizonTensorWrapper>>
      map_output_tensor_;
  bool is_dynamic_batch_ = false;
};

}  // namespace qcraft::planner::ml

#endif  // ONBOARD_PLANNER_ML_CAPTAIN_NET_J5_QNN_H
