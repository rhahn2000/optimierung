cmake_minimum_required(VERSION 3.10)

project(Raytracer)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_executable(raytracer main.cpp math/math.cc geometry/geometry.cc scene/Scene.cc scene/Material.cc scene/Light.cc io/ObjLoader.cc)

add_subdirectory(math)
add_subdirectory(geometry)

target_include_directories(raytracer PRIVATE ${CMAKE_SOURCE_DIR})

set_target_properties(raytracer PROPERTIES
    VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
)

target_compile_definitions(raytracer PRIVATE
    DATA_PATH="${CMAKE_SOURCE_DIR}/data/"
)

include(FetchContent)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
)
FetchContent_MakeAvailable(googletest)