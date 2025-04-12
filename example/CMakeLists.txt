cmake_minimum_required(VERSION 3.20)

project(example LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)

#复制资源文件到输出目录
file(GLOB ResourceFiles_ttf ${CMAKE_CURRENT_SOURCE_DIR}/res/fonts/*.ttf )
file(COPY ${ResourceFiles_ttf} DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/res/fonts)
file(GLOB ResourceFiles_shader ${CMAKE_CURRENT_SOURCE_DIR}/res/shaders/*.* )
file(COPY ${ResourceFiles_shader} DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/res/shaders)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../extern/glfw/glfw/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../extern/glad/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../PE2D/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../extern/imgui/imgui)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../extern/glm)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

add_compile_definitions(
   # -DDEBUG_MODE
)
# 设置example
set(EXAMPLE_SOURCE_FILES 
	src/main.cpp
 ${CMAKE_CURRENT_SOURCE_DIR}/../extern/imgui/imgui/backends/imgui_impl_glfw.cpp
 ${CMAKE_CURRENT_SOURCE_DIR}/../extern/imgui/imgui/backends/imgui_impl_opengl3.cpp
)

set(EXAMPLE_HEADER_FILES 
 ${CMAKE_CURRENT_SOURCE_DIR}/../extern/imgui/imgui/backends/imgui_impl_glfw.h
 ${CMAKE_CURRENT_SOURCE_DIR}/../extern/imgui/imgui/backends/imgui_impl_opengl3.h
 ${CMAKE_CURRENT_SOURCE_DIR}/include/camera/camera.hpp
 ${CMAKE_CURRENT_SOURCE_DIR}/include/shader/shader.hpp
)

set(CMAKE_EXPERIMENTAL_CXX_MODULES ON)

add_executable(example ${EXAMPLE_SOURCE_FILES} ${EXAMPLE_HEADER_FILES} ${ResourceFiles_ttf} ${ResourceFiles_shader})


# 链接 ImGui 库
target_link_libraries(example PRIVATE imgui)

# 链接 GLFW 库
target_link_libraries(example PRIVATE glfw)

# 链接 GLAD 库
target_link_libraries(example PRIVATE glad)

# 链接 PE2D 库
target_link_libraries(example PRIVATE PE2D)

# 链接 GLM 库
target_compile_definitions(example PRIVATE glm)

# 为可执行文件添加头文件搜索路径
target_include_directories(example PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../extern/imgui/imgui
    ${CMAKE_CURRENT_SOURCE_DIR}/../extern/glm/
    ${CMAKE_CURRENT_SOURCE_DIR}/../extern/imgui/imgui/backends
    ${CMAKE_CURRENT_SOURCE_DIR}/../extern/glfw/glfw/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../extern/glad/include
	${CMAKE_CURRENT_SOURCE_DIR}/../PE2D/include
)