add_executable(core_test
    Camera_test.cc
    Raytracer_test.cc
    AABB_test.cc
    KDTree_test.cc
    Camera.cc
    Raytracer.cc
    AABB.cc
    KDTree.cc
    ${CMAKE_SOURCE_DIR}/math/math.cc
    ${CMAKE_SOURCE_DIR}/geometry/geometry.cc
    ${CMAKE_SOURCE_DIR}/scene/Scene.cc
    ${CMAKE_SOURCE_DIR}/scene/Material.cc
    ${CMAKE_SOURCE_DIR}/scene/Light.cc
)

target_include_directories(core_test PRIVATE ${CMAKE_SOURCE_DIR})

target_link_libraries(core_test GTest::gtest_main)

include(GoogleTest)
gtest_discover_tests(core_test)