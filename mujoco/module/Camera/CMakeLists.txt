# Boost.Interprocess (header-only) needs the Boost headers themselves and
# librt/pthread. NOTE FOR THE LEAD: the k1sim:latest image does not currently
# have Boost headers installed (verified: /usr/include/boost/interprocess is
# absent) -- add `libboost-dev` to mujoco/docker/Dockerfile's apt-get install
# list (matches the package NUbots_K1's own docker/Dockerfile installs for the
# same header-only usage). pthread already flows in transitively via
# k1sim_shared (Threads::Threads); librt's symbols are provided by libc itself
# on glibc >= 2.34 (Ubuntu 22.04), but -lrt is kept for portability/older glibc.
find_package(Boost REQUIRED)
# EGL (offscreen GL, no GLFW) — decouples the camera render thread from the
# Viewer's GLFW/Xlib state and works headless. libegl1-mesa-dev is in the image.
find_package(OpenGL REQUIRED COMPONENTS EGL)

add_library(k1sim_module_camera STATIC
    src/Camera.cpp
    src/SharedImageWriter.cpp
    src/SharedPoseWriter.cpp
)
target_link_libraries(
    k1sim_module_camera
    PUBLIC k1sim_shared
    PRIVATE Boost::headers OpenGL::EGL OpenGL::GL rt
)
