# Prebuilt ONNX Runtime (CPU) for the policy locomotion backend (K1_WITH_ONNX=ON).
#
# Sourced from Microsoft's official NuGet package (Microsoft.ML.OnnxRuntime, author-signed by
# Microsoft Corporation) because the github.com/microsoft/onnxruntime release archives return 404
# since Nov 2026. The .nupkg is a zip holding the same linux-x64 libonnxruntime.so + C/C++ headers.
include(FetchContent)

set(ORT_VERSION 1.20.1)

FetchContent_Declare(
    onnxruntime_prebuilt
    URL https://api.nuget.org/v3-flatcontainer/microsoft.ml.onnxruntime/${ORT_VERSION}/microsoft.ml.onnxruntime.${ORT_VERSION}.nupkg
    URL_HASH SHA256=9c32108e9a1c78b090a95c2a219f244dd5f067d4caf68fb0f967103f3f20839c
    DOWNLOAD_NAME onnxruntime-${ORT_VERSION}.zip
)
FetchContent_MakeAvailable(onnxruntime_prebuilt)

add_library(onnxruntime::onnxruntime SHARED IMPORTED)
set_target_properties(
    onnxruntime::onnxruntime
    PROPERTIES IMPORTED_LOCATION "${onnxruntime_prebuilt_SOURCE_DIR}/runtimes/linux-x64/native/libonnxruntime.so"
               INTERFACE_INCLUDE_DIRECTORIES "${onnxruntime_prebuilt_SOURCE_DIR}/build/native/include"
)
