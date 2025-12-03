find_package(Protobuf REQUIRED)
protobuf_generate_cpp(ONBOARD_PLANNER_ML_CONTEXT_FEATURE_EXTRACTOR_PROTO_SRCS ONBOARD_PLANNER_ML_CONTEXT_FEATURE_EXTRACTOR_PROTO_HDRS
  context_feature.proto
)
protobuf_generate_python(ONBOARD_PLANNER_ML_CONTEXT_FEATURE_EXTRACTOR_PROTO_PY
  context_feature.proto
)
add_custom_target(onboard_planner_ml_context_feature_extractor_proto_py_code ALL DEPENDS ${ONBOARD_PLANNER_ML_CONTEXT_FEATURE_EXTRACTOR_PROTO_PY})
set_target_properties(onboard_planner_ml_context_feature_extractor_proto_py_code PROPERTIES FOLDER "OnBoard/Planner/Ml/ContextFeatureExtractor/Proto")

add_library(onboard_planner_ml_context_feature_extractor_proto STATIC
  ${ONBOARD_PLANNER_ML_CONTEXT_FEATURE_EXTRACTOR_PROTO_SRCS}
  ${ONBOARD_PLANNER_ML_CONTEXT_FEATURE_EXTRACTOR_PROTO_HDRS}
)
set_target_properties(onboard_planner_ml_context_feature_extractor_proto PROPERTIES FOLDER "OnBoard/Planner/Ml/ContextFeatureExtractor/Proto")
target_include_directories(onboard_planner_ml_context_feature_extractor_proto SYSTEM PUBLIC
  ${Protobuf_INCLUDE_DIRS}
)
target_link_directories(onboard_planner_ml_context_feature_extractor_proto PUBLIC
  ${Protobuf_LIBRARY_DIRS}
)
target_link_libraries(onboard_planner_ml_context_feature_extractor_proto PUBLIC
  onboard_prediction_proto
  onboard_math_geometry_proto
  ${Protobuf_LIBS}
)