find_package(OpenGL REQUIRED)

add_library(k1sim_module_viewer STATIC src/Viewer.cpp src/ViewerUI.cpp)
target_link_libraries(
  k1sim_module_viewer
  PUBLIC k1sim_shared
  PRIVATE glfw OpenGL::GL imgui nlohmann_json::nlohmann_json
)
target_link_libraries(imgui PUBLIC OpenGL::GL)

# UI font: Roboto ships with the fetched Dear ImGui sources (Apache-2.0). ViewerUI falls back to ImGui's built-in
# bitmap font if the file is missing at runtime.
target_compile_definitions(
  k1sim_module_viewer PRIVATE K1SIM_UI_FONT="${imgui_SOURCE_DIR}/misc/fonts/Roboto-Medium.ttf"
)
