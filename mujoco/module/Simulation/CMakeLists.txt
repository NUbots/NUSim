add_library(k1sim_module_simulation STATIC src/Simulation.cpp src/SimCore.cpp)
target_link_libraries(k1sim_module_simulation PUBLIC k1sim_shared)
