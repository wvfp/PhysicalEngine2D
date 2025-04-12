cmake_minimum_required(VERSION 3.15)

project(PhysicalEngine2D)

# 设置 C++ 标准为 C++23
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


# 添加子目录
add_subdirectory(extern/imgui)

add_subdirectory(extern/glfw)

add_subdirectory(extern/glad)

add_subdirectory(extern/glm)

add_subdirectory(PE2D)

add_subdirectory(example)

