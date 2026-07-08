# sdkbridge_synthetic_sim: test-only binary used by test/contract/test_sdk_roundtrip.py
# to exercise the real module::SdkBridge without depending on module::Simulation/
# Locomotion/Viewer being functional yet. See SyntheticState.hpp for rationale.
add_executable(sdkbridge_synthetic_sim main.cpp SyntheticState.cpp)
target_link_libraries(
    sdkbridge_synthetic_sim
    PRIVATE k1sim_shared k1sim_module_consolelog k1sim_module_sdkbridge
)
