# Generates the Fast-DDS type support from mujoco/idl/**/*.idl at build time.
#
# The generated code is a build artifact (${CMAKE_BINARY_DIR}/idl_gen), never committed — the same arrangement NUbots
# uses for protobuf (nuclear/message/Neutron.cmake runs protoc over shared/message/**.proto into the build dir).
# mujoco/idl/**/*.idl is the source of truth; see module/SdkBridge/PROTOCOL.md for where those layouts came from.
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

# fastddsgen emits exactly these six files per type, into idl_gen/<package>/. idl/regenerate.sh asserts that invariant
# after every run (a type outside its own package directory, or any file at the output root, is a hard error there) —
# which is what makes this list safe to predict.
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

# A manifest of the .idl set, rewritten at configure time only when that set changes. Depending the generator on it
# makes a REMOVED or renamed message re-run generation (which wipes OUT_DIR first); without it, ninja sees every
# remaining output still present and up to date, and the deleted type's headers linger in the build tree on the include
# path.
list(SORT k1sim_idl_files)
string(REPLACE ";" "\n" k1sim_idl_manifest "${k1sim_idl_files}")
file(CONFIGURE OUTPUT ${CMAKE_BINARY_DIR}/idl_manifest.txt CONTENT "${k1sim_idl_manifest}\n")

# One invocation for every .idl, not one per file: fastddsgen also generates the code for every type an .idl #includes
# (Odometry.idl pulls in Header, Time, PoseWithCovariance, ...), so per-file commands would have two rules claiming the
# same output file, which cmake rejects. That also means every .idl is a dependency of the single command — coarse, but
# a rebuild is one JVM start, and it needs no dependency scraping to stay correct.
add_custom_command(
  OUTPUT ${k1sim_idl_outputs}
  COMMAND ${CMAKE_COMMAND} -E env FASTDDSGEN=${FASTDDSGEN_EXECUTABLE} OUT_DIR=${K1SIM_IDL_OUT} bash
          ${K1SIM_IDL_DIR}/regenerate.sh
  DEPENDS ${k1sim_idl_files} ${K1SIM_IDL_DIR}/regenerate.sh ${CMAKE_BINARY_DIR}/idl_manifest.txt
  COMMENT "Generating Fast-DDS types from mujoco/idl"
  VERBATIM
)
