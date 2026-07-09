# Imported target for the prebuilt MuJoCo installed by tools/install_deps.sh.
# Override the location with -DMUJOCO_DIR=... or the MUJOCO_DIR environment variable.
set(MUJOCO_VERSION 3.10.0)

set(_mj_hints "")
if(MUJOCO_DIR)
    list(APPEND _mj_hints "${MUJOCO_DIR}")
endif()
if(DEFINED ENV{MUJOCO_DIR})
    list(APPEND _mj_hints "$ENV{MUJOCO_DIR}")
endif()
if(DEFINED ENV{K1SIM_DEPS_PREFIX})
    list(APPEND _mj_hints "$ENV{K1SIM_DEPS_PREFIX}/mujoco-${MUJOCO_VERSION}")
endif()
list(APPEND _mj_hints "${CMAKE_CURRENT_SOURCE_DIR}/.deps/install/mujoco-${MUJOCO_VERSION}"
     "/opt/k1sim-deps/mujoco-${MUJOCO_VERSION}")

set(_mj_includes "")
set(_mj_libs "")
foreach(root ${_mj_hints})
    list(APPEND _mj_includes "${root}/include")
    list(APPEND _mj_libs "${root}/lib")
endforeach()

find_path(
    MUJOCO_INCLUDE_DIR mujoco/mujoco.h
    HINTS ${_mj_includes}
    NO_DEFAULT_PATH
)
find_library(
    MUJOCO_LIBRARY mujoco
    HINTS ${_mj_libs}
    NO_DEFAULT_PATH
)

if(NOT MUJOCO_INCLUDE_DIR OR NOT MUJOCO_LIBRARY)
    message(
        FATAL_ERROR
            "MuJoCo ${MUJOCO_VERSION} not found (searched: ${_mj_hints}).\n"
            "Build inside docker (docker/k1sim.sh build), run tools/install_deps.sh, "
            "or point MUJOCO_DIR at a MuJoCo install."
    )
endif()

add_library(mujoco::mujoco SHARED IMPORTED)
set_target_properties(
    mujoco::mujoco
    PROPERTIES IMPORTED_LOCATION "${MUJOCO_LIBRARY}"
               INTERFACE_INCLUDE_DIRECTORIES "${MUJOCO_INCLUDE_DIR}"
)
