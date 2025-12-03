#include "onboard/planner/router/thirdparty/external_route_request.h"

#include <list>
#include <optional>
#include <ostream>
#include <string>
#include <string_view>

#include "absl/status/status.h"
#include "absl/strings/numbers.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/strings/str_split.h"
#include "absl/time/clock.h"
#include "google/protobuf/repeated_ptr_field.h"
#include "nlohmann/json.hpp"

#include "onboard/global/trace.h"
#include "onboard/lite/logging.h"
#include "onboard/math/util.h"
#include "onboard/planner/router/thirdparty/coordinate_transformer.h"
#include "onboard/planner/router/thirdparty/tencent_action_format.h"
#include "onboard/proto/route_navigation.pb.h"
#include "onboard/utils/http_client.h"
#include "onboard/utils/time_util.h"

namespace qcraft::planner::route {
namespace {
std::string FormatTencentStr(const Vec2d& pt) {
  return absl::StrFormat("%.7f,%.7f", r2d(pt.y()), r2d(pt.x()));
}

bool ParseTencentStr(std::string_view src, Vec2d* pt) {
  std::vector<std::string> latlng = absl::StrSplit(src, ",");
  bool b1 = absl::SimpleAtod(latlng[0], pt->data() + 1);
  bool b2 = absl::SimpleAtod(latlng[1], pt->data());

  pt->data()[1] = d2r(pt->data()[1]);
  pt->data()[0] = d2r(pt->data()[0]);
  return b1 && b2;
}

absl::StatusOr<std::vector<Vec2d>> ParseTencentCoordJsonArray(
    const nlohmann::json& j_arr) {
  if (!j_arr.is_array()) {
    return absl::InvalidArgumentError("Must be a json array.");
  }
  std::vector<Vec2d> result(j_arr.size());
  int i = 0;
  for (const auto& j_latlng : j_arr) {
    if (!ParseTencentStr(j_latlng.get<std::string>(), &result[i])) {
      return absl::InternalError(
          absl::StrCat("Parse data error ", j_latlng.dump()));
    }
    ++i;
  }
  return result;
}

void ConvertStep(const nlohmann::json& step, const std::vector<Vec2d>& points,
                 NaviSegmentProto* segment) {
  std::vector<int> polyline_idx =
      step.contains("polylineIdx") ? step["polylineIdx"].get<std::vector<int>>()
                                   : std::vector<int>();
  for (int idx : polyline_idx) {
    segment->add_coord_idx(idx);
  }
  segment->set_length(step.value("distance", 0));

  NaviActionProto* navi_action = segment->add_navi_actions();
  auto action_op = ConvertMainAction(step.value("actDesc", ""));
  if (action_op.has_value()) {
    navi_action->set_navi_action_type(*action_op);
  } else {
    QLOG(WARNING) << "Cannot convert main action. step:" << step;
  }

  auto sub_action_op = ConvertSubAction(step.value("accessorial_desc", ""));
  if (sub_action_op.has_value()) {
    navi_action->set_sub_action_type(*sub_action_op);
  }

  const auto& start_point = points[polyline_idx.back()];
  start_point.ToProto(navi_action->mutable_start_point());
  navi_action->set_section_id(-1);
}

}  // namespace

namespace internal {
ExternalRouteResult ConvertTencentJson(const CoordinateTransformer& cf,
                                       const nlohmann::json& content) {
  SCOPED_QTRACE("ThirdRoute::ConvertTencentJson");
  ExternalRouteResult result;
  NaviPreviewProto& navi_preview_proto = result.navi_preview;
  const auto& j_coordinates = content["coordinates"];
  auto coords_or = ParseTencentCoordJsonArray(j_coordinates);
  result.points = cf.Gcj02ToWgs84(*coords_or);
  navi_preview_proto.mutable_navi_segments()->Reserve(content["steps"].size());
  for (const auto& j_step : content["steps"]) {
    ConvertStep(j_step, result.points, navi_preview_proto.add_navi_segments());
  }
  return result;
}
}  // namespace internal

absl::StatusOr<ExternalRouteResult> TencentRoute(
    const CoordinateTransformer& cf, const Vec2d& origin,
    const Vec2d& destination, absl::Span<const Vec2d> waypoints,
    std::string_view url, const int road_type) {
  SCOPED_QTRACE("ThirdRoute::TencentRoute");
  qcraft::HttpClient http_client;
  std::list<std::string> headers;
  headers.push_back("Content-Type: application/json");
  nlohmann::json request;
  nlohmann::json resp;

  request["roadType"] = road_type;
  request["from"] = FormatTencentStr(origin);
  request["to"] = FormatTencentStr(destination);

  auto j_via_points = nlohmann::json::array();
  for (const auto& pt : waypoints) {
    j_via_points.push_back(FormatTencentStr(pt));
  }
  request["viaPoints"] = j_via_points;
  std::string result;
  result.reserve(8192);
  const auto begin_time = ToUnixDoubleSeconds(absl::Now());
  auto s =
      http_client.Post(std::string(url), headers, request, &result,
                       /*timeout = */ 3000, /*connection_timeout = */ 30000);
  if (!s.ok()) {
    return s;
  }
  const double get_results_time = ToUnixDoubleSeconds(absl::Now());
  QLOG(INFO) << "Get tencent route successfully, time cost: "
             << (get_results_time - begin_time) << " seconds.";
  resp = nlohmann::json::parse(result, /*cb=*/nullptr,
                               /*allow_exceptions =*/false,
                               /*ignore_comments */ true);
  if (resp.is_null() || resp.is_discarded()) {
    return absl::NotFoundError(result);
  }
  if (!resp.contains("code") || resp["code"] != 0 || !resp.contains("data") ||
      !resp["data"].contains("steps")) {
    return absl::InternalError(
        absl::StrCat("req:", request.dump(), ", resp:", resp.dump()));
  }
  auto results = internal::ConvertTencentJson(cf, resp["data"]);
  QLOG(INFO) << "Parse tencent route successfully, time cost: "
             << (ToUnixDoubleSeconds(absl::Now()) - get_results_time)
             << " seconds.";
  return results;
}

// Radians
absl::StatusOr<std::vector<Vec2d>> Wgs84ToGcj02ByQops(
    const std::vector<Vec2d>& points, std::string_view url) {
  qcraft::HttpClient http_client;
  std::list<std::string> headers;
  headers.push_back("Content-Type: application/json");
  nlohmann::json request;
  auto coordinates = nlohmann::json::array();
  nlohmann::json resp;
  for (const auto& pt : points) {
    coordinates.push_back(FormatTencentStr(pt));
  }
  request["coordinates"] = coordinates;
  auto s =
      http_client.Post(std::string(url), headers, request, &resp,
                       /*timeout = */ 3000, /*connection_timeout = */ 30000);

  if (!s.ok()) {
    return s;
  }
  if (!resp.contains("code") || resp["code"] != 0) {
    return absl::InternalError(resp.dump());
  }
  std::vector<Vec2d> result(points.size());
  int i = 0;
  for (const auto& j_latlng : resp["data"]) {
    if (!ParseTencentStr(j_latlng.get<std::string>(), &result[i])) {
      return absl::InternalError(
          absl::StrCat("Parse data error ", resp.dump()));
    }
    ++i;
  }
  return result;
}

}  // namespace qcraft::planner::route
