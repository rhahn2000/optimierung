cmake_minimum_required(VERSION 3.10)

project(Raytracer)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_executable(hello_world main.cpp math/math.cc geometry/geometry.cc)

add_subdirectory(math)
add_subdirectory(geometry)

target_include_directories(hello_world PRIVATE ${CMAKE_SOURCE_DIR})

include(FetchContent)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
)
FetchContent_MakeAvailable(googletest)