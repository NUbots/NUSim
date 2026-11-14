# The GameController C API is an optional, separately built dependency (tools/install_game_controller_api.sh). Without
# it this module and the sim/match role are skipped, so builds that don't need them are unaffected.
if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
  return()
endif()

set(GC_API_PREFIX "${CMAKE_SOURCE_DIR}/.deps/game_controller_api")
find_library(
  GC_API_LIBRARY
  NAMES game_controller_api
  PATHS "${GC_API_PREFIX}/lib"
  NO_DEFAULT_PATH
)
if(NOT GC_API_LIBRARY OR NOT EXISTS "${GC_API_PREFIX}/include/GameController.h")
  unset(GC_API_LIBRARY CACHE)
  message(STATUS "GameController API not staged: skipping MatchReferee and sim/match "
                 "(run tools/install_game_controller_api.sh to enable them)"
  )
  return()
endif()

add_library(k1sim_module_matchreferee STATIC src/MatchReferee.cpp)
target_include_directories(k1sim_module_matchreferee PUBLIC "${GC_API_PREFIX}/include")
target_compile_definitions(k1sim_module_matchreferee PUBLIC K1SIM_GC_PARAMS_DIR="${GC_API_PREFIX}/config")
target_link_libraries(k1sim_module_matchreferee PUBLIC k1sim_shared ${GC_API_LIBRARY})

# Registered here rather than in test/unit so that directory's glob, which every test links against the same fixed
# libraries, doesn't need the optional GameController API.
enable_testing()
add_executable(test_match_referee test/test_match_referee.cpp)
target_link_libraries(
  test_match_referee PRIVATE k1sim_module_matchreferee k1sim_module_simulation k1sim_module_sdkbridge
)
add_test(NAME test_match_referee COMMAND test_match_referee)
set_tests_properties(test_match_referee PROPERTIES WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
