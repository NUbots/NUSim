# Every test_*.cpp here becomes a ctest target automatically — workstreams add
# test files without touching this CMakeLists (avoids concurrent-edit conflicts).
file(GLOB test_sources CONFIGURE_DEPENDS "test_*.cpp")

foreach(test_src ${test_sources})
    get_filename_component(test_name ${test_src} NAME_WE)
    add_executable(${test_name} ${test_src})
    target_link_libraries(
        ${test_name}
        PRIVATE k1sim_shared
                k1sim_module_consolelog
                k1sim_module_simulation
                k1sim_module_sdkbridge
                k1sim_module_locomotion
                k1sim_module_viewer
    )
    add_test(NAME ${test_name} COMMAND ${test_name})
    set_tests_properties(${test_name} PROPERTIES WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
endforeach()
