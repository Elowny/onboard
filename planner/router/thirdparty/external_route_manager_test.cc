#include "onboard/planner/router/thirdparty/external_route_manager.h"

#include <string>

#include "google/protobuf/repeated_ptr_field.h"

#include "gtest/gtest.h"

#include "common/proto/map_geometry.pb.h"

#include "onboard/base/macros.h"
#include "onboard/math/coordinate_converter.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/geometry/util.h"
#include "onboard/planner/router/geometry/gfc.h"
#include "onboard/proto/adasis.pb.h"
#include "onboard/proto/route.pb.h"
#include "onboard/utils/file_util.h"

namespace qcraft::planner::route {

namespace {

void FillSdRouteMatchStateFirstTime(
    const RoutingResultProto& routing_result_proto,
    SdRouteMatchState* sd_match_state) {
  auto& sd_route_match_state = *sd_match_state;
  sd_route_match_state.coords.clear();
  sd_route_match_state.segment_dists.clear();
  int points_size = routing_result_proto.path_points_from_current_size();
  if (points_size == 0) {
    return;
  }
  sd_route_match_state.coords.reserve(points_size);
  sd_route_match_state.cur_coord_index = 0;
  for (const auto& point : routing_result_proto.path_points_from_current()) {
    sd_route_match_state.coords.push_back(Vec2dFromProto(point));
  }
  auto cc =
      CoordinateConverter::FromMapOriginCoord(sd_route_match_state.coords[0]);
  sd_route_match_state.segment_dists.reserve(points_size);
  auto last_smooth_pt = cc.GlobalToSmooth(sd_route_match_state.coords[0]);
  for (const auto& pt : sd_route_match_state.coords) {
    auto smooth_pt = cc.GlobalToSmooth(pt);
    auto dist = smooth_pt.DistanceTo(last_smooth_pt);
    sd_route_match_state.segment_dists.push_back(dist);
    last_smooth_pt = smooth_pt;
  }
}

// NOLINTNEXTLINE
TEST(ExternalRouteManager, UpdateRouteTest_destination) {
  ExternalRouteManager route_manager;
  auto* rms = route_manager.mutable_route_manager_state();
  std::string str_routing_result = R"(update_id: 0
    routing_request {
      destinations {
        global_point {
          longitude: 2.1050558
          latitude: 0.5476658
        }
      }
      destinations {
        global_point {
          longitude: 2.1059578
          latitude: 0.5481989
          altitude: 0
        }
      }
      destinations {
        global_point {
          longitude: 2.1054642
          latitude: 0.548413
          altitude: 0
        }
      }
    }
    success: true
    path_points_from_current {
      x: 2.1047361
      y: 0.54830201745329243
    }
    path_points_from_current {
      x: 2.1046694633291594
      y: 0.5482952281225022
    }
    path_points_from_current {
      x: 2.1046272612678458
      y: 0.54829117895863755
    }
    path_points_from_current {
      x: 2.1045651449997678
      y: 0.54828475614699024
    }
    path_points_from_current {
      x: 2.1045592632401884
      y: 0.54828431981467718
    }
    path_points_from_current {
      x: 2.1045592632401884
      y: 0.54828431981467718
    }
    path_points_from_current {
      x: 2.1045572561115482
      y: 0.548284983039793
    }
    path_points_from_current {
      x: 2.1045305525739932
      y: 0.54828304572432318
    }
    path_points_from_current {
      x: 2.1045305525739932
      y: 0.54828304572432318
    }
    path_points_from_current {
      x: 2.1045227160456514
      y: 0.54828269665847285
    }
    path_points_from_current {
      x: 2.1045170786321679
      y: 0.54828281883152052
    }
    path_points_from_current {
      x: 2.1045130120150106
      y: 0.54828320280395593
    }
    path_points_from_current {
      x: 2.1045067986428734
      y: 0.548284581614065
    }
    path_points_from_current {
      x: 2.1045018942676754
      y: 0.54828650147624225
    }
    path_points_from_current {
      x: 2.1044996427929403
      y: 0.548287635940256
    }
    path_points_from_current {
      x: 2.1044835159506516
      y: 0.54829793338284283
    }
    path_points_from_current {
      x: 2.1044835159506516
      y: 0.54829793338284283
    }
    path_points_from_current {
      x: 2.1044755747025552
      y: 0.54830044665696565
    }
    path_points_from_current {
      x: 2.1044717349782007
      y: 0.54830100516232627
    }
    path_points_from_current {
      x: 2.1044692391573707
      y: 0.54830105752220382
    }
    path_points_from_current {
      x: 2.1044637937301043
      y: 0.54830055137672074
    }
    path_points_from_current {
      x: 2.10445787706394
      y: 0.54829877114088377
    }
    path_points_from_current {
      x: 2.1044539675264153
      y: 0.548296833825414
    }
    path_points_from_current {
      x: 2.1044501278020609
      y: 0.548294111111781
    }
    path_points_from_current {
      x: 2.1044463404375842
      y: 0.54829000958803875
    }
    path_points_from_current {
      x: 2.1044431639383454
      y: 0.54828454670748006
    }
    path_points_from_current {
      x: 2.1044421167407945
      y: 0.548281527287874
    }
    path_points_from_current {
      x: 2.1044409473701955
      y: 0.54827470305049875
    }
    path_points_from_current {
      x: 2.1044408426504404
      y: 0.54827243412247106
    }
    path_points_from_current {
      x: 2.10443827701644
      y: 0.54825897763393827
    }
    path_points_from_current {
      x: 2.1044361651680452
      y: 0.54825328786057681
    }
    path_points_from_current {
      x: 2.1044249601542475
      y: 0.54822728245472208
    }
    path_points_from_current {
      x: 2.104420387391607
      y: 0.54821747370432583
    }
    path_points_from_current {
      x: 2.1044161811481095
      y: 0.54820956736281434
    }
    path_points_from_current {
      x: 2.1044155702828715
      y: 0.548207106448569
    }
    path_points_from_current {
      x: 2.1044158844421368
      y: 0.54820534366602447
    }
    path_points_from_current {
      x: 2.1044104564681634
      y: 0.548196198140744
    }
    path_points_from_current {
      x: 2.1044049237744344
      y: 0.54818745404119151
    }
    path_points_from_current {
      x: 2.1044015552889781
      y: 0.54818265438574854
    }
    path_points_from_current {
      x: 2.1043886922123911
      y: 0.54816530581298373
    }
    path_points_from_current {
      x: 2.1043766494405522
      y: 0.54815130827238268
    }
    path_points_from_current {
      x: 2.1043574159121952
      y: 0.54813087046684184
    }
    path_points_from_current {
      x: 2.1043451287942609
      y: 0.54811741397830893
    }
    path_points_from_current {
      x: 2.1043374144389673
      y: 0.54810849534583128
    }
    path_points_from_current {
      x: 2.1043267853838223
      y: 0.548094654884863
    }
    path_points_from_current {
      x: 2.1043162086885556
      y: 0.54807866766891478
    }
    path_points_from_current {
      x: 2.1043104840086091
      y: 0.54806945233046411
    }
    path_points_from_current {
      x: 2.1043025602138048
      y: 0.54805398871329158
    }
    path_points_from_current {
      x: 2.1042956487099671
      y: 0.5480389439751393
    }
    path_points_from_current {
      x: 2.1042898193102655
      y: 0.54802377706393957
    }
    path_points_from_current {
      x: 2.104282122408264
      y: 0.54799831271015287
    }
    path_points_from_current {
      x: 2.1042361504357667
      y: 0.54782386705141606
    }
    path_points_from_current {
      x: 2.1042287327864457
      y: 0.54779667482167
    }
    path_points_from_current {
      x: 2.1042227812136964
      y: 0.54777281617079521
    }
    path_points_from_current {
      x: 2.104220093406648
      y: 0.54776367064551479
    }
    navi_preview {
      navi_segments {
        navi_actions {
          navi_action_type: TURN_RIGHT
          start_point {
            x: 2.1045592632401884
            y: 0.54828431981467718
          }
          section_id: -1
        }
        length: 969
        coord_idx: 0
        coord_idx: 4
      }
      navi_segments {
        navi_actions {
          navi_action_type: TURN_LEFT
          start_point {
            x: 2.1045305525739932
            y: 0.54828304572432318
          }
          section_id: -1
        }
        length: 157
        coord_idx: 5
        coord_idx: 7
      }
      navi_segments {
        navi_actions {
          navi_action_type: TURN_LEFT
          start_point {
            x: 2.1044835159506516
            y: 0.54829793338284283
          }
          section_id: -1
        }
        length: 283
        coord_idx: 8
        coord_idx: 15
      }
      navi_segments {
        navi_actions {
          navi_action_type: TURN_RIGHT
          start_point {
            x: 2.104220093406648
            y: 0.54776367064551479
          }
          section_id: -1
        }
        length: 3856
        coord_idx: 16
        coord_idx: 54
      }
      update_id: 0
    })";

  file_util::StringToProto(str_routing_result, &(rms->routing_result_proto));
  ASSERT_TRUE(rms->routing_result_proto.success());
  FillSdRouteMatchStateFirstTime(rms->routing_result_proto,
                                 &rms->sd_route_match_state);
  int k = 0;
  std::vector<Vec2d> points(
      rms->routing_result_proto.path_points_from_current().size());
  for (const auto& pt_proto :
       rms->routing_result_proto.path_points_from_current()) {
    points[k++].FromProto(pt_proto);
  }
  std::vector<double> dist_table(points.size());
  dist_table[0] = 0;
  for (int i = 1; i < static_cast<int>(points.size()); i++) {
    dist_table[i] =
        dist_table[i - 1] + HaversineDistance(points[i - 1], points[i]);
  }

  EXPECT_EQ(route_manager.coords().size(), dist_table.size());

  // UpdateRoute test
  for (int i = 0; i < static_cast<int>(route_manager.coords().size()); i++) {
    ASSERT_TRUE(route_manager.UpdateRoute(route_manager.coords()[i], 50));
    double distance = dist_table.back() - dist_table[i];
    EXPECT_NEAR(rms->rms_debug.distance_to_next_stop(), distance, 50);
  }
  const auto routing_result = route_manager.route();
  EXPECT_FALSE(routing_result.path_points_from_current().empty());
  RouteManagerState empty_state;
  route_manager.InjectState(empty_state);
  EXPECT_FALSE(route_manager.route_manager_state().has_route());
}

TEST(ExternalRouteManager, InitSdRoute) {
  SDRouteProto sd_route;
  sd_route.set_route_id(100);
  sd_route.set_length(100);
  sd_route.set_status(SDRouteProto::CREATE);

  auto* section = sd_route.add_sd_sections();
  section->set_length(100);
  {
    auto* link1 = section->add_sd_links();
    link1->set_length(100);
    link1->set_link_id(1);
    auto* geo_pt_1 = link1->mutable_points()->Add();
    geo_pt_1->set_longitude(2.0);
    geo_pt_1->set_latitude(0.501);

    auto* geo_pt_2 = link1->mutable_points()->Add();
    geo_pt_2->set_longitude(2.0);
    geo_pt_2->set_latitude(0.5010001);

    auto* geo_pt_3 = link1->mutable_points()->Add();
    geo_pt_3->set_longitude(2.0);
    geo_pt_3->set_latitude(0.5010002);
  }

  SdRouteMatchResultProto sd_match;
  sd_match.set_route_id(100);
  auto* segment = sd_match.add_segments();
  segment->mutable_start()->set_link_index(0);
  segment->mutable_start()->set_link_point_index(0);
  auto* start_pt = segment->mutable_start()->mutable_point();
  start_pt->set_longitude(2.0);
  start_pt->set_longitude(0.50100005);

  segment->mutable_end()->set_link_index(0);
  segment->mutable_end()->set_link_point_index(1);
  auto* end_pt = segment->mutable_end()->mutable_point();
  end_pt->set_longitude(2.0);
  end_pt->set_latitude(0.50100015);
  ExternalRouteManager erm;
  const auto s = erm.InitSdRoute(
      std::make_shared<const SDRouteProto>(std::move(sd_route)),
      std::make_shared<const SdRouteMatchResultProto>(std::move(sd_match)));
  ASSERT_OK(s);
}

}  // namespace

}  // namespace qcraft::planner::route
