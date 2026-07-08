add_library(k1sim_module_supervisor STATIC src/Supervisor.cpp)
target_link_libraries(k1sim_module_supervisor PUBLIC k1sim_shared)
# GameControllerPacket.hpp / SupervisorPlacement.hpp / SupervisorConfig.hpp /
# SupervisorLogic.hpp are header-only (no extra sources here) so
# test/unit/test_supervisor.cpp can use them without linking this library —
# see that test CMakeLists' comment on why it doesn't link per-module libs.
