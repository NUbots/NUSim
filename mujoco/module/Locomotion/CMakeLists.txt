add_library(
    k1sim_module_locomotion STATIC
    src/Locomotion.cpp
    src/LocomotionController.cpp
    src/GetUpScript.cpp
    src/backends/KinematicBackend.cpp
)
target_link_libraries(k1sim_module_locomotion PUBLIC k1sim_shared)

if(K1_WITH_ONNX)
    include(${CMAKE_SOURCE_DIR}/cmake/FetchOnnxRuntime.cmake)
    target_sources(k1sim_module_locomotion PRIVATE src/backends/PolicyBackend.cpp)
    # PUBLIC: downstream consumers (e.g. test/unit's test_locomotion binary)
    # need to see K1_WITH_ONNX too, to compile their own #ifdef'd policy-backend
    # test cases consistently with how this library was built.
    target_compile_definitions(k1sim_module_locomotion PUBLIC K1_WITH_ONNX)
    # PRIVATE is fine here: for a STATIC library CMake still forwards the
    # link requirement (via $<LINK_ONLY:...>) to whatever finally links this
    # library, it just doesn't leak onnxruntime's include dirs to consumers
    # that don't need them (nothing outside this module includes onnxruntime
    # headers directly).
    target_link_libraries(k1sim_module_locomotion PRIVATE onnxruntime::onnxruntime)
endif()
