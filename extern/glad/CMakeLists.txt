file(GLOB_RECURSE GLAD_SOURCES "src/*.c")

add_library(glad ${GLAD_SOURCES})

target_include_directories(glad PUBLIC include)
