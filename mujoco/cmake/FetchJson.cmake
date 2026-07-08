# nlohmann/json (header-only) — used for the Booster RPC JSON bodies.
include(FetchContent)

set(JSON_BuildTests OFF CACHE BOOL "" FORCE)

FetchContent_Declare(
    nlohmann_json
    URL https://github.com/nlohmann/json/archive/refs/tags/v3.11.3.tar.gz
)
FetchContent_MakeAvailable(nlohmann_json)
