find_package(Protobuf REQUIRED)
protobuf_generate_cpp(ONBOARD_PREDICTION_PROTO_SRCS ONBOARD_PREDICTION_PROTO_HDRS
  act_net.proto 
  cutin.proto 
  prediction_cutin_sl.proto 
  conflict_resolver.proto 
  prediction_common.proto 
  prophnet.proto 
  vehicle_tnt.proto
)
protobuf_generate_python(ONBOARD_PREDICTION_PROTO_PY
  act_net.proto
  cutin.proto
  prediction_cutin_sl.proto
  conflict_resolver.proto
  prediction_common.proto
  prophnet.proto
  vehicle_tnt.proto
)
add_custom_target(onboard_prediction_proto_py_code ALL DEPENDS ${ONBOARD_PREDICTION_PROTO_PY})
set_target_properties(onboard_prediction_proto_py_code PROPERTIES FOLDER "OnBoard/Prediction/Proto")
add_library(onboard_prediction_proto STATIC
  ${ONBOARD_PREDICTION_PROTO_SRCS}
  ${ONBOARD_PREDICTION_PROTO_HDRS}
)
set_target_properties(onboard_prediction_proto PROPERTIES FOLDER "OnBoard/Prediction/Proto")
target_link_libraries(onboard_prediction_proto PUBLIC
  onboard_proto_perception_proto
  onboard_math_geometry_proto
)