cmake_minimum_required (VERSION 3.9)
project (geometry)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# add_compile_options(-g -Wall -Wextra -Wpedantic -Wl,--stack,16777216)
add_compile_options(/W4 /Z7)

add_executable(geometry_math_test ../math/math_test.cc ../math/math.cc)
target_link_libraries(geometry_math_test gtest gtest_main)

target_include_directories(geometry_math_test PRIVATE ${CMAKE_SOURCE_DIR})
