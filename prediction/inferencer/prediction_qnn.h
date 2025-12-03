#ifndef ONBOARD_PREDICTION_INFERENCER_PREDICTION_QNN_H_
#define ONBOARD_PREDICTION_INFERENCER_PREDICTION_QNN_H_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "onboard/nets/proto/net_param.pb.h"
#include "onboard/nets/qnn/qnn/qnn.h"

namespace qcraft {
namespace prediction {

class PredictionQNN {
 public:
  explicit PredictionQNN(const std::string& model_path,
                         const NetParam& net_param);
  ~PredictionQNN();

  std::map<std::string, std::vector<float>> Run(
      const std::vector<std::vector<float>>& batch, int num_objs);

 private:
  void SetInputs(const std::vector<std::vector<float>>& batch);
  std::map<std::string, std::vector<float>> ExtractOutputs(int num_objs);

 private:
  std::shared_ptr<qnn::QNN> net_;
  std::shared_ptr<qnn::Session> session_;
  qnn::NetworkConfig config_;
  bool is_cuda_ = false;
};

}  // namespace prediction
}  // namespace qcraft

#endif  // ONBOARD_PREDICTION_INFERENCER_PREDICTION_QNN_H_
