find_package(Protobuf REQUIRED)
protobuf_generate_cpp(ONBOARD_TRAJECTORY_VALIDATION_PROTO_SRCS ONBOARD_TRAJECTORY_VALIDATION_PROTO_HDRS
  trajectory_validation.proto
)
protobuf_generate_python(ONBOARD_TRAJECTORY_VALIDATION_PROTO_PY
  trajectory_validation.proto
)
add_custom_target(onboard_trajectory_validation_proto_py_code ALL DEPENDS ${ONBOARD_TRAJECTORY_VALIDATION_PROTO_PY})
set_target_properties(onboard_trajectory_validation_proto_py_code PROPERTIES FOLDER "OnBoard/Planner/Proto")

add_library(onboard_planner_proto_trajectory_validation_proto STATIC
  ${ONBOARD_TRAJECTORY_VALIDATION_PROTO_SRCS}
  ${ONBOARD_TRAJECTORY_VALIDATION_PROTO_HDRS}
)
set_target_properties(onboard_planner_proto_trajectory_validation_proto PROPERTIES FOLDER "OnBoard/Planner/Proto")
target_include_directories(onboard_planner_proto_trajectory_validation_proto SYSTEM PUBLIC
  ${Protobuf_INCLUDE_DIRS}
)
target_link_directories(onboard_planner_proto_trajectory_validation_proto PUBLIC
  ${Protobuf_LIBRARY_DIRS}
)
target_link_libraries(onboard_planner_proto_trajectory_validation_proto PUBLIC
  ${Protobuf_LIBS}
)

protobuf_generate_cpp(ONBOARD_ASYNC_PLANNER_STATE_PROTO_SRCS ONBOARD_ASYNC_PLANNER_STATE_PROTO_HDRS
  async_planner_state.proto
)
protobuf_generate_python(ONBOARD_ASYNC_PLANNER_STATE_PROTO_PY
  async_planner_state.proto
)
add_custom_target(onboard_async_planner_state_proto_py_code ALL DEPENDS ${ONBOARD_ASYNC_PLANNER_STATE_PROTO_PY})
set_target_properties(onboard_async_planner_state_proto_py_code PROPERTIES FOLDER "OnBoard/Planner/Proto")

add_library(onboard_planner_proto_async_planner_state_proto STATIC
  ${ONBOARD_ASYNC_PLANNER_STATE_PROTO_SRCS}
  ${ONBOARD_ASYNC_PLANNER_STATE_PROTO_HDRS}
)
set_target_properties(onboard_planner_proto_async_planner_state_proto PROPERTIES FOLDER "OnBoard/Planner/Proto")
target_include_directories(onboard_planner_proto_async_planner_state_proto SYSTEM PUBLIC
  ${Protobuf_INCLUDE_DIRS}
)
target_link_directories(onboard_planner_proto_async_planner_state_proto PUBLIC
  ${Protobuf_LIBRARY_DIRS}
)
target_link_libraries(onboard_planner_proto_async_planner_state_proto PUBLIC
  ${Protobuf_LIBS}
)

protobuf_generate_cpp(ONBOARD_EST_PLANNER_DEBUG_PROTO_SRCS ONBOARD_EST_PLANNER_DEBUG_PROTO_HDRS
  est_planner_debug.proto
)
protobuf_generate_python(ONBOARD_EST_PLANNER_DEBUG_PROTO_PY
  est_planner_debug.proto
)
add_custom_target(onboard_est_planner_debug_proto_py_code ALL DEPENDS ${ONBOARD_EST_PLANNER_DEBUG_PROTO_PY})
set_target_properties(onboard_est_planner_debug_proto_py_code PROPERTIES FOLDER "OnBoard/Planner/Proto")

add_library(onboard_planner_proto_est_planner_debug_proto STATIC
  ${ONBOARD_EST_PLANNER_DEBUG_PROTO_SRCS}
  ${ONBOARD_EST_PLANNER_DEBUG_PROTO_HDRS}
)
set_target_properties(onboard_planner_proto_est_planner_debug_proto PROPERTIES FOLDER "OnBoard/Planner/Proto")
target_link_libraries(onboard_planner_proto_est_planner_debug_proto PUBLIC
  onboard_planner_proto_trajectory_validation_proto
  onboard_planner_initializer_proto
  onboard_planner_speed_proto
  onboard_planner_scheduler_proto
  onboard_planner_decision_proto
  onboard_planner_optimization_proto
  onboard_planner_object_proto
  onboard_planner_common_proto
  onboard_proto_turn_signal_proto
  onboard_proto_trajectory_point_proto
  onboard_planner_ml_captain_net_debug_proto
)

protobuf_generate_cpp(ONBOARD_PLANNER_STATE_PROTO_SRCS ONBOARD_PLANNER_STATE_PROTO_HDRS
  planner_state.proto
)
protobuf_generate_python(ONBOARD_PLANNER_STATE_PROTO_PY
  planner_state.proto
)
add_custom_target(onboard_planner_state_proto_py_code ALL DEPENDS ${ONBOARD_PLANNER_STATE_PROTO_PY})
set_target_properties(onboard_planner_state_proto_py_code PROPERTIES FOLDER "OnBoard/Planner/Proto")

add_library(onboard_planner_proto_planner_state_proto STATIC
  ${ONBOARD_PLANNER_STATE_PROTO_SRCS}
  ${ONBOARD_PLANNER_STATE_PROTO_HDRS}
)
set_target_properties(onboard_planner_proto_planner_state_proto PROPERTIES FOLDER "OnBoard/Planner/Proto")
target_link_libraries(onboard_planner_proto_planner_state_proto PUBLIC
  onboard_planner_decision_proto
  onboard_planner_scheduler_proto
  onboard_planner_selector_proto
  onboard_proto_autonomy_state_proto
  onboard_proto_planner_proto
  onboard_proto_trajectory_proto
  onboard_proto_route_proto
  onboard_planner_initializer_proto
  onboard_planner_freespace_proto
  onboard_planner_object_proto
  onboard_planner_plan_proto
  onboard_planner_scheduler_proto
  onboard_planner_speed_proto
  onboard_planner_assist_proto
  onboard_planner_proto_async_planner_state_proto
  onboard_proto_trajectory_point_proto
  onboard_math_geometry_proto
  onboard_maps_proto
  onboard_lite_proto
  common_proto
)