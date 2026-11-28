# Prebuilt ONNX Runtime (CPU) for the policy locomotion backend (K1_WITH_ONNX=ON).
include(FetchContent)

set(ORT_VERSION 1.20.1)

FetchContent_Declare(
    onnxruntime_prebuilt
    URL https://api.nuget.org/v3-flatcontainer/microsoft.ml.onnxruntime/${ORT_VERSION}/microsoft.ml.onnxruntime.${ORT_VERSION}.nupkg
    URL_HASH SHA256=9c32108e9a1c78b090a95c2a219f244dd5f067d4caf68fb0f967103f3f20839c
    DOWNLOAD_NAME onnxruntime-${ORT_VERSION}.zip
)
FetchContent_MakeAvailable(onnxruntime_prebuilt)

set(ORT_LIB_DIR "${onnxruntime_prebuilt_SOURCE_DIR}/runtimes/linux-x64/native")
file(CREATE_LINK libonnxruntime.so "${ORT_LIB_DIR}/libonnxruntime.so.1" SYMBOLIC)

add_library(onnxruntime::onnxruntime SHARED IMPORTED)
set_target_properties(
    onnxruntime::onnxruntime
    PROPERTIES IMPORTED_LOCATION "${ORT_LIB_DIR}/libonnxruntime.so"
               INTERFACE_INCLUDE_DIRECTORIES "${onnxruntime_prebuilt_SOURCE_DIR}/build/native/include"
)
