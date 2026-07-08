# --- k1sim_idl: vendored fastddsgen output (mujoco/idl_gen/, see idl/regenerate.sh) ---
# Clean-room IDL for the Booster SDK wire types; see PROTOCOL.md for provenance.
set(K1SIM_IDL_GEN_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../idl_gen)

file(GLOB k1sim_idl_srcs CONFIGURE_DEPENDS
     ${K1SIM_IDL_GEN_DIR}/booster_interface/*.cxx
     ${K1SIM_IDL_GEN_DIR}/booster_msgs/*.cxx
)
file(GLOB k1sim_idl_ipp CONFIGURE_DEPENDS
     ${K1SIM_IDL_GEN_DIR}/booster_interface/*.ipp
     ${K1SIM_IDL_GEN_DIR}/booster_msgs/*.ipp
)
# Each <Type>CdrAux.ipp holds explicit fastcdr template specializations
# (calculate_serialized_size/serialize/deserialize/serialize_key for that exact
# type) that <Type>PubSubTypes.cxx calls but neither declares nor includes the
# definition of — fastddsgen expects them compiled as their own translation unit.
# CMake doesn't recognise ".ipp" as a C++ source extension by default, so mark it
# explicitly (each file's include guard + single glob-inclusion keeps this to
# exactly one definition per type across the whole link).
set_source_files_properties(${k1sim_idl_ipp} PROPERTIES LANGUAGE CXX)

add_library(k1sim_idl STATIC ${k1sim_idl_srcs} ${k1sim_idl_ipp})
target_include_directories(
    k1sim_idl
    PUBLIC ${K1SIM_IDL_GEN_DIR}/booster_interface ${K1SIM_IDL_GEN_DIR}/booster_msgs
)
target_link_libraries(k1sim_idl PUBLIC fastrtps fastcdr)

# --- module::SdkBridge ---
add_library(
    k1sim_module_sdkbridge STATIC
    src/SdkBridge.cpp
    src/DdsParticipant.cpp
    src/StatePublisher.cpp
    src/RpcServer.cpp
    src/RpcDispatch.cpp
)
target_link_libraries(
    k1sim_module_sdkbridge
    PUBLIC k1sim_shared k1sim_idl
    PRIVATE fastrtps fastcdr nlohmann_json::nlohmann_json
)

add_subdirectory(test_support)
