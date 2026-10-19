add_library(k1sim_module_supervisor STATIC src/Supervisor.cpp)
target_link_libraries(k1sim_module_supervisor PUBLIC k1sim_shared)
# GameControllerPacket, SupervisorPlacement, SupervisorConfig and SupervisorLogic are header-only so
# test/unit/test_supervisor.cpp can use them without linking this library.
