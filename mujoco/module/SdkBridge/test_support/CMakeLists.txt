# sdkbridge_synthetic_sim: test-only binary for test/contract/test_sdk_roundtrip.py. Runs the real module::SdkBridge
# against a synthetic state source (see SyntheticState.hpp).
add_executable(sdkbridge_synthetic_sim main.cpp SyntheticState.cpp)
target_link_libraries(sdkbridge_synthetic_sim PRIVATE k1sim_shared k1sim_module_consolelog k1sim_module_sdkbridge)
