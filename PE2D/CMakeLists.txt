project(PE2D LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)

set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include
					${CMAKE_CURRENT_SOURCE_DIR}/../extern/glm/glm)

set(PE2D_SOURCE_FILES ./src/test.cpp)

set(PE2D_HEADER_FILES ./include/test.hpp)

add_library(PE2D ${PE2D_SOURCE_FILES} ${PE2D_HEADER_FILES})

target_include_directories(PE2D PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../extern/glm/glm>
)

