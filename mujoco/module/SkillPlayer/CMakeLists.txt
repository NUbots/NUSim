include(${PROJECT_SOURCE_DIR}/cmake/FetchOnnxRuntime.cmake)

add_library(k1sim_module_skillplayer STATIC src/SkillPlayer.cpp)
target_link_libraries(
  k1sim_module_skillplayer
  PUBLIC k1sim_shared
  PRIVATE onnxruntime::onnxruntime
)
