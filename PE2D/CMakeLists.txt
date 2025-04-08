project(PE2D LANGUAGES CXX)

# 设置 C++ 标准为 C++20
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 添加 include 目录
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include
                    ${CMAKE_CURRENT_SOURCE_DIR}/../extern/glm/glm)

# 收集 modules 目录下的所有.cppm 和 ixx文件
file(GLOB_RECURSE PE2D_MODULE_SOURCE_FILES "${CMAKE_CURRENT_SOURCE_DIR}/modules/*.cppm" "${CMAKE_CURRENT_SOURCE_DIR}/modules/*.ixx")

# 创建一个模块库目标
add_library(PE2D STATIC ${PE2D_MODULE_SOURCE_FILES})

target_link_directories(PE2D PRIVATE glm)

# 设置目标的 include 目录
target_include_directories(PE2D PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../extern/glm/glm>
)

# 为模块设置模块接口文件（.ixx）相关的编译选项（可选，根据实际情况调整）
# 这里假设你的.ixx 文件不需要特殊编译选项，仅作为接口声明
# 如果需要特殊处理，可以添加额外的编译指令

# 设置输出模块文件的名称和路径（可选，默认会根据目标名称生成）
 set_target_properties(PE2D PROPERTIES
     SUFFIX ".cppm"  # 对于模块输出，可能不需要设置此后缀，取决于编译器支持
 )