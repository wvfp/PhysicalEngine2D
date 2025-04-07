cmake_minimum_required(VERSION 3.10)
project(imgui)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 11)

# 查找 GLFW 库
find_package(glfw3 REQUIRED)

# 查找 OpenGL 库
find_package(OpenGL REQUIRED)

# ImGui 源文件和头文件路径
set(IMGUI_DIR imgui)  # 请替换为你的 ImGui 实际路径
include_directories(${IMGUI_DIR} ${IMGUI_DIR}/backends)

# ImGui 源文件列表
set(IMGUI_SOURCES
    ${IMGUI_DIR}/imgui.cpp
    ${IMGUI_DIR}/imgui_demo.cpp
    ${IMGUI_DIR}/imgui_draw.cpp
    ${IMGUI_DIR}/imgui_tables.cpp
    ${IMGUI_DIR}/imgui_widgets.cpp
    ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp
    ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
)

# 添加 ImGui 库
add_library(imgui ${IMGUI_SOURCES})

# 链接 ImGui 库到 GLFW 和 OpenGL
target_link_libraries(imgui glfw ${OPENGL_LIBRARIES})
