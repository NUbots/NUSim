# NUClear, pinned to the same commit NUbots_K1's docker image installs
# (docker/Dockerfile "install-from-source .../NUClear/archive/925dca0f...").
include(FetchContent)

set(BUILD_TESTS OFF CACHE BOOL "" FORCE)

FetchContent_Declare(
    NUClear
    URL https://github.com/Fastcode/NUClear/archive/925dca0f31484a7df64fd335de5a6c9335483c7f.tar.gz
)

# NUClear's top-level CMakeLists resolves its helper modules (ClangTidy,
# CompilerOptions, Sanitizers) via CMAKE_SOURCE_DIR, which under FetchContent is
# *this* project — so add its module dirs to our path before add_subdirectory.
FetchContent_GetProperties(NUClear)
if(NOT nuclear_POPULATED)
    FetchContent_Populate(NUClear)
    list(APPEND CMAKE_MODULE_PATH "${nuclear_SOURCE_DIR}/cmake" "${nuclear_SOURCE_DIR}/cmake/Modules")
    add_subdirectory("${nuclear_SOURCE_DIR}" "${nuclear_BINARY_DIR}")

    # NUClear only exports include dirs through its install rules; for build-tree
    # (FetchContent) consumers, expose the generated umbrella header
    # (<binary>/include/nuclear) and the source headers it references.
    target_include_directories(
        nuclear SYSTEM PUBLIC "$<BUILD_INTERFACE:${nuclear_BINARY_DIR}/include>"
                              "$<BUILD_INTERFACE:${nuclear_SOURCE_DIR}/src>"
    )
endif()
