cmake_minimum_required(VERSION 3.15)

project(PhysicalEngine2D)

# 设置 C++ 标准为 C++23
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fmodules")
elseif(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fmodules")
elseif(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /experimental:module")
endif()

# 添加子目录
add_subdirectory(extern/imgui)

add_subdirectory(extern/glfw)

add_subdirectory(extern/glad)

add_subdirectory(extern/glm)

add_subdirectory(PE2D)

add_subdirectory(example)

