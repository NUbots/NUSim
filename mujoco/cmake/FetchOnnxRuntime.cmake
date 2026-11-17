# ONNX Runtime (prebuilt, CPU) -- module::SkillPlayer runs the mjlab skill policies with it.
#
# Microsoft's release archive holds headers and one shared library, so nothing is compiled here. The library stays in
# the build tree and the role binary finds it through its build rpath.
include(FetchContent)

set(K1SIM_ORT_VERSION 1.20.1)
if(APPLE)
  set(ort_platform "osx-universal2")
  set(ort_library "libonnxruntime.${K1SIM_ORT_VERSION}.dylib")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
  set(ort_platform "linux-aarch64")
  set(ort_library "libonnxruntime.so.${K1SIM_ORT_VERSION}")
else()
  set(ort_platform "linux-x64")
  set(ort_library "libonnxruntime.so.${K1SIM_ORT_VERSION}")
endif()

FetchContent_Declare(
  onnxruntime
  URL https://github.com/microsoft/onnxruntime/releases/download/v${K1SIM_ORT_VERSION}/onnxruntime-${ort_platform}-${K1SIM_ORT_VERSION}.tgz
)
FetchContent_MakeAvailable(onnxruntime)

add_library(onnxruntime::onnxruntime SHARED IMPORTED GLOBAL)
set_target_properties(
  onnxruntime::onnxruntime
  PROPERTIES IMPORTED_LOCATION "${onnxruntime_SOURCE_DIR}/lib/${ort_library}"
             INTERFACE_INCLUDE_DIRECTORIES "${onnxruntime_SOURCE_DIR}/include"
)
if(NOT APPLE)
  set_target_properties(onnxruntime::onnxruntime PROPERTIES IMPORTED_SONAME "libonnxruntime.so.1")
endif()
