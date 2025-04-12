project(PE2D LANGUAGES CXX)

# 设置 C++ 标准为 C++20
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 添加 include 目录
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# 收集 modules 目录下的所有.cpp 和 .hpp文件
file(GLOB_RECURSE PE2D_HEADER_FILES "${CMAKE_CURRENT_SOURCE_DIR}/include/*.hpp")
file(GLOB_RECURSE PE2D_SOURCE_FILES "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")
# 创建一个模块库目标
add_library(PE2D STATIC ${PE2D_HEADER_FILES} ${PE2D_SOURCE_FILES})

target_include_directories(PE2D PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
