# Prebuilt ONNX Runtime (CPU) for the policy locomotion backend (K1_WITH_ONNX=ON).
include(FetchContent)

set(ORT_VERSION 1.20.1)

FetchContent_Declare(
    onnxruntime_prebuilt
    URL https://github.com/microsoft/onnxruntime/releases/download/v${ORT_VERSION}/onnxruntime-linux-x64-${ORT_VERSION}.tgz
)
FetchContent_MakeAvailable(onnxruntime_prebuilt)

add_library(onnxruntime::onnxruntime SHARED IMPORTED)
set_target_properties(
    onnxruntime::onnxruntime
    PROPERTIES IMPORTED_LOCATION "${onnxruntime_prebuilt_SOURCE_DIR}/lib/libonnxruntime.so"
               INTERFACE_INCLUDE_DIRECTORIES "${onnxruntime_prebuilt_SOURCE_DIR}/include"
)
