find_package(Protobuf REQUIRED)
protobuf_generate_cpp(ONBOARD_PLANNER_ML_PROTO_SRCS ONBOARD_PLANNER_ML_PROTO_HDRS
  auto_tuning.proto
)
protobuf_generate_python(ONBOARD_PLANNER_ML_PROTO_PY
  auto_tuning.proto
)
add_custom_target(onboard_planner_ml_proto_py_code ALL DEPENDS ${ONBOARD_PLANNER_ML_PROTO_PY})
set_target_properties(onboard_planner_ml_proto_py_code PROPERTIES FOLDER "OnBoard/Planner/Ml/Proto")

add_library(onboard_planner_ml_proto STATIC
  ${ONBOARD_PLANNER_ML_PROTO_SRCS}
  ${ONBOARD_PLANNER_ML_PROTO_HDRS}
)
set_target_properties(onboard_planner_ml_proto PROPERTIES FOLDER "OnBoard/Planner/Ml/Proto")
target_include_directories(onboard_planner_ml_proto SYSTEM PUBLIC
  ${Protobuf_INCLUDE_DIRS}
)
target_link_directories(onboard_planner_ml_proto PUBLIC
  ${Protobuf_LIBRARY_DIRS}
)
target_link_libraries(onboard_planner_ml_proto PUBLIC
  ${Protobuf_LIBS}
)