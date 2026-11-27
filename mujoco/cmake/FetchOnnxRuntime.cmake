# ONNX Runtime (prebuilt, CPU) -- module::SkillPlayer runs the mjlab skill policies with it.
#
# Microsoft's NuGet package holds the headers and a shared library for every platform, so nothing is compiled here.
# (The GitHub release archives this used to fetch are no longer downloadable.) The library stays in the build tree and
# the role binary finds it through its build rpath.
include(FetchContent)

set(K1SIM_ORT_VERSION 1.20.1)
if(APPLE)
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "arm64|aarch64")
    set(ort_platform "osx-arm64")
  else()
    set(ort_platform "osx-x64")
  endif()
  set(ort_library "libonnxruntime.dylib")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
  set(ort_platform "linux-arm64")
  set(ort_library "libonnxruntime.so")
else()
  set(ort_platform "linux-x64")
  set(ort_library "libonnxruntime.so")
endif()

FetchContent_Declare(
  onnxruntime
  URL https://api.nuget.org/v3-flatcontainer/microsoft.ml.onnxruntime/${K1SIM_ORT_VERSION}/microsoft.ml.onnxruntime.${K1SIM_ORT_VERSION}.nupkg
  URL_HASH SHA256=9c32108e9a1c78b090a95c2a219f244dd5f067d4caf68fb0f967103f3f20839c
  # A .nupkg is a zip; the name tells CMake to extract it.
  DOWNLOAD_NAME onnxruntime-${K1SIM_ORT_VERSION}.zip
)
FetchContent_MakeAvailable(onnxruntime)

# The package ships the library only under its plain name, but binaries load it by its soname (Linux) or install name
# (macOS), so link that name next to it for the build rpath to find.
set(ort_native_dir "${onnxruntime_SOURCE_DIR}/runtimes/${ort_platform}/native")
if(APPLE)
  set(ort_load_name "libonnxruntime.${K1SIM_ORT_VERSION}.dylib")
else()
  set(ort_load_name "libonnxruntime.so.1")
endif()
file(CREATE_LINK "${ort_library}" "${ort_native_dir}/${ort_load_name}" SYMBOLIC)

add_library(onnxruntime::onnxruntime SHARED IMPORTED GLOBAL)
set_target_properties(
  onnxruntime::onnxruntime
  PROPERTIES IMPORTED_LOCATION "${onnxruntime_SOURCE_DIR}/runtimes/${ort_platform}/native/${ort_library}"
             INTERFACE_INCLUDE_DIRECTORIES "${onnxruntime_SOURCE_DIR}/build/native/include"
)
if(NOT APPLE)
  set_target_properties(onnxruntime::onnxruntime PROPERTIES IMPORTED_SONAME "libonnxruntime.so.1")
endif()
