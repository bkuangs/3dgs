cmake_minimum_required(VERSION 3.24)

project(three_dgs_mvp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(FetchContent)

find_package(OpenGL REQUIRED)

find_package(glfw3 CONFIG QUIET)
if (NOT TARGET glfw)
    set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
    set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)

    FetchContent_Declare(
        glfw
        URL https://github.com/glfw/glfw/archive/refs/tags/3.4.tar.gz
    )
    FetchContent_MakeAvailable(glfw)
endif()

find_package(glm CONFIG QUIET)
if (NOT TARGET glm::glm)
    FetchContent_Declare(
        glm
        URL https://github.com/g-truc/glm/archive/refs/tags/1.0.1.tar.gz
    )
    FetchContent_MakeAvailable(glm)
endif()

add_library(colmap_parser STATIC
    src/io/parse_colmap.cpp
)
target_include_directories(colmap_parser PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/io
)

add_executable(convert_colmap
    src/tools/convert_colmap.cpp
)
target_link_libraries(convert_colmap PRIVATE
    colmap_parser
)

add_executable(validate_geom
    src/tools/validate_geom.cpp
)
target_link_libraries(validate_geom PRIVATE
    colmap_parser
)

add_executable(viewer
    src/viewer/main.cpp
)
target_link_libraries(viewer PRIVATE
    colmap_parser
    OpenGL::GL
    glfw
    glm::glm
)

