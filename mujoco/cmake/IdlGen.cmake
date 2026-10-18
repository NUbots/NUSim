# Generates the Fast-DDS type support from mujoco/idl/**/*.idl at build time.
#
# The generated code is a build artifact (${CMAKE_BINARY_DIR}/idl_gen), never committed, as NUbots does for protobuf.
# The .idl files are the source of truth; see module/SdkBridge/PROTOCOL.md for their provenance.
#
# Include this from the directory that defines the k1sim_idl target, so the GENERATED source properties and the target
# share a directory scope.
#
# Exports:
#
# * k1sim_idl_files: the .idl inputs
# * k1sim_idl_outputs: every generated file (the library's source list)
# * k1sim_idl_packages: one generated include directory per IDL package

find_program(
  FASTDDSGEN_EXECUTABLE fastddsgen
  HINTS $ENV{K1SIM_DEPS_PREFIX}/bin ${PROJECT_SOURCE_DIR}/.deps/install/bin /opt/k1sim-deps/bin
  DOC "eProsima Fast-DDS-Gen IDL compiler (installed by tools/install_deps.sh)"
)
if(NOT FASTDDSGEN_EXECUTABLE)
  message(
    FATAL_ERROR
      "fastddsgen not found. It generates the DDS types at build time — run "
      "tools/install_deps.sh (needs a Java 11+ runtime), or build in the docker image "
      "(docker/k1sim.sh), which bakes it into /opt/k1sim-deps/bin."
  )
endif()

set(K1SIM_IDL_DIR ${PROJECT_SOURCE_DIR}/idl)
set(K1SIM_IDL_OUT ${CMAKE_BINARY_DIR}/idl_gen)

# CONFIGURE_DEPENDS: adding a new .idl re-runs cmake on the next build, so a new message needs no edit here at all.
file(GLOB k1sim_idl_files CONFIGURE_DEPENDS "${K1SIM_IDL_DIR}/*/msg/*.idl")
if(NOT k1sim_idl_files)
  message(FATAL_ERROR "No .idl files found under ${K1SIM_IDL_DIR}/*/msg/")
endif()

# fastddsgen emits these six files per type into idl_gen/<package>/; idl/regenerate.sh fails if that stops holding.
set(k1sim_idl_outputs "")
set(k1sim_idl_packages "")
foreach(idl ${k1sim_idl_files})
  get_filename_component(type_name ${idl} NAME_WE)
  get_filename_component(pkg_dir ${idl} DIRECTORY) # .../<pkg>/msg
  get_filename_component(pkg_dir ${pkg_dir} DIRECTORY) # .../<pkg>
  get_filename_component(pkg ${pkg_dir} NAME)
  foreach(suffix ".cxx" ".h" "CdrAux.hpp" "CdrAux.ipp" "PubSubTypes.cxx" "PubSubTypes.h")
    list(APPEND k1sim_idl_outputs "${K1SIM_IDL_OUT}/${pkg}/${type_name}${suffix}")
  endforeach()
  list(APPEND k1sim_idl_packages "${K1SIM_IDL_OUT}/${pkg}")
endforeach()
list(REMOVE_DUPLICATES k1sim_idl_packages)

# A manifest of the .idl set, rewritten at configure time only when the set changes. Depending on it makes removing or
# renaming a message re-run generation (which wipes OUT_DIR); otherwise the old headers linger on the include path.
list(SORT k1sim_idl_files)
string(REPLACE ";" "\n" k1sim_idl_manifest "${k1sim_idl_files}")
file(CONFIGURE OUTPUT ${CMAKE_BINARY_DIR}/idl_manifest.txt CONTENT "${k1sim_idl_manifest}\n")

# One invocation for all .idl files: fastddsgen also generates every #included type, so per-file commands would claim
# the same outputs, which cmake rejects.
add_custom_command(
  OUTPUT ${k1sim_idl_outputs}
  COMMAND ${CMAKE_COMMAND} -E env FASTDDSGEN=${FASTDDSGEN_EXECUTABLE} OUT_DIR=${K1SIM_IDL_OUT} bash
          ${K1SIM_IDL_DIR}/regenerate.sh
  DEPENDS ${k1sim_idl_files} ${K1SIM_IDL_DIR}/regenerate.sh ${CMAKE_BINARY_DIR}/idl_manifest.txt
  COMMENT "Generating Fast-DDS types from mujoco/idl"
  VERBATIM
)
