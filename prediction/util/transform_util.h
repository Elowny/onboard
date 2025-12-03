
#ifndef ONBOARD_PREDICTION_UTIL_TRANSFORM_UTIL_H_
#define ONBOARD_PREDICTION_UTIL_TRANSFORM_UTIL_H_
#include "onboard/math/coordinate_converter.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/proto/positioning.pb.h"
namespace qcraft {
namespace prediction {
// Convert origin_pose defined in `origin` coordinate converter to `unified`
// coordinate converter. Note that we only need to convert smooth coordinate
// fields, although not all the smooth coordinate fields are converted.
PoseProto ConvertToUnifiedCoordinatePose(const PoseProto& origin_pose,
                                         const CoordinateConverter& origin,
                                         const CoordinateConverter& unified);

// Convert origin_object defined in `origin` coordinate converter to `unified`
// coordinate converter. Note that we only need to convert smooth coordinate
// fields.
ObjectProto ConvertToUnifiedCoordinatePerceptionObject(
    const ObjectProto& origin_object, const CoordinateConverter& origin,
    const CoordinateConverter& unified);

bool HasInvalidLocTransformInHistory(
    absl::Span<const LocalizationTransformProto* const> loc_transforms);

}  // namespace prediction
}  // namespace qcraft

#endif  // ONBOARD_PREDICTION_UTIL_TRANSFORM_UTIL_H_
