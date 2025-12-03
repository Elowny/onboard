find_package(Protobuf REQUIRED)
protobuf_generate_cpp(ONBOARD_PLANNER_ROUTER_PROTO_SRCS ONBOARD_PLANNER_ROUTER_PROTO_HDRS
  route_manager_output.proto
  route_params.proto
)
protobuf_generate_python(ONBOARD_PLANNER_ROUTER_PROTO_PY
  route_manager_output.proto
  route_params.proto
)
add_custom_target(onboard_planner_router_proto_py_code ALL DEPENDS ${ONBOARD_PLANNER_ROUTER_PROTO_PY})
set_target_properties(onboard_planner_router_proto_py_code PROPERTIES FOLDER "OnBoard/Planner/Router/Proto")

add_library(onboard_planner_router_proto STATIC
  ${ONBOARD_PLANNER_ROUTER_PROTO_SRCS}
  ${ONBOARD_PLANNER_ROUTER_PROTO_HDRS}
)
set_target_properties(onboard_planner_router_proto PROPERTIES FOLDER "OnBoard/Planner/Router/Proto")
target_include_directories(onboard_planner_router_proto SYSTEM PUBLIC
  ${Protobuf_INCLUDE_DIRS}
)
target_link_directories(onboard_planner_router_proto PUBLIC
  ${Protobuf_LIBRARY_DIRS}
)
target_link_libraries(onboard_planner_router_proto PUBLIC
  onboard_lite_proto
  onboard_proto_turn_signal_proto
  onboard_proto_route_proto
  onboard_proto_hmi_content_proto
  common_proto
  ${Protobuf_LIBS}
)