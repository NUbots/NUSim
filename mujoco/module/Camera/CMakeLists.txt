# Boost.Interprocess is header-only. Threads::Threads flows through k1sim_shared.
find_package(Boost REQUIRED)

if(APPLE)
  # CGL creates a native offscreen context on the camera's own thread, without Cocoa/GLFW.
  find_package(OpenGL REQUIRED)
else()
  find_package(OpenGL REQUIRED COMPONENTS EGL)
endif()

add_library(k1sim_module_camera STATIC src/Camera.cpp src/SharedImageWriter.cpp src/SharedPoseWriter.cpp)
target_link_libraries(
  k1sim_module_camera
  PUBLIC k1sim_shared
  PRIVATE Boost::headers OpenGL::GL
)
if(NOT APPLE)
  target_link_libraries(k1sim_module_camera PRIVATE OpenGL::EGL)
endif()
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  target_link_libraries(k1sim_module_camera PRIVATE rt)
endif()
