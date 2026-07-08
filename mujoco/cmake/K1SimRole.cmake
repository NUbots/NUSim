# Role → binary machinery (a lighter port of NUbots' nuclear_role()).
#
# A mujoco/roles/**/*.role file is a CMake snippet that calls k1sim_role() with a
# list of module short-names (e.g. `Simulation SdkBridge Locomotion`). Each name
# X maps to: the reactor class k1sim::module::X, the header module/X/src/X.hpp,
# and the static library target k1sim_module_<lowercase-X>. generate_role.py emits
# a <role>.cpp main() that installs ChronoController + each listed module.
#
# The enclosing glob loop (in CMakeLists.txt) sets `role` (dashed target name, e.g.
# sim-soccer), `role_name` (soccer) and `role_path` (sim) before include()-ing the
# role file, so the binary lands at bin/<role_path>/<role_name>.
find_package(Python3 REQUIRED)

function(k1sim_role)
    set(role_modules ${ARGN})

    add_custom_command(
        OUTPUT "${role}.cpp"
        COMMAND ${Python3_EXECUTABLE} "${K1SIM_ROLE_DIR}/generate_role.py" "${role}.cpp"
                "${PROJECT_SOURCE_DIR}" ${role_modules}
        DEPENDS "${K1SIM_ROLE_DIR}/generate_role.py"
        COMMENT "Generating role ${role}"
        VERBATIM
    )

    add_executable(${role} "${role}.cpp")
    set_target_properties(
        ${role} PROPERTIES OUTPUT_NAME "${role_name}"
                           RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/bin/${role_path}"
    )

    set(role_libs "")
    foreach(m ${role_modules})
        string(TOLOWER "${m}" m_lc)
        list(APPEND role_libs "k1sim_module_${m_lc}")
    endforeach()
    target_link_libraries(${role} PRIVATE ${role_libs} k1sim_shared)
endfunction()
