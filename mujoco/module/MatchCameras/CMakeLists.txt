# Only sim/match uses this, which is Linux-only (see module/MatchReferee).
if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
  return()
endif()

find_package(Boost REQUIRED)
find_package(OpenGL REQUIRED COMPONENTS EGL)

add_library(k1sim_module_matchcameras STATIC src/MatchCameras.cpp)
target_link_libraries(
  k1sim_module_matchcameras
  PUBLIC k1sim_shared
  PRIVATE k1sim_module_camera Boost::headers OpenGL::GL OpenGL::EGL
)
