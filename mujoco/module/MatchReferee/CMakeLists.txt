# Needs the GameController API, found in the top-level CMakeLists.txt. Without it this module and sim/match are skipped,
# so builds that don't need them are unaffected.
if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux"
   OR NOT GC_API_LIBRARY
   OR NOT GC_API_INCLUDE_DIR
)
  return()
endif()

add_library(k1sim_module_matchreferee STATIC src/MatchReferee.cpp)
target_link_libraries(k1sim_module_matchreferee PUBLIC k1sim_shared k1sim_gamecontroller ${GC_API_LIBRARY})

# Registered here rather than in test/unit so that directory's glob, which every test links against the same fixed
# libraries, doesn't need the optional GameController API.
enable_testing()
add_executable(test_match_referee test/test_match_referee.cpp)
target_link_libraries(
  test_match_referee PRIVATE k1sim_module_matchreferee k1sim_module_simulation k1sim_module_sdkbridge
)
add_test(NAME test_match_referee COMMAND test_match_referee)
set_tests_properties(test_match_referee PROPERTIES WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
