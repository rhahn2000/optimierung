add_executable(scene_test
    Scene.cc
    Scene_Test.cc
    Material.cc
    Light.cc
    ${CMAKE_SOURCE_DIR}/math/math.cc
    ${CMAKE_SOURCE_DIR}/geometry/geometry.cc
    ${CMAKE_SOURCE_DIR}/core/KDTree.cc
    ${CMAKE_SOURCE_DIR}/core/AABB.cc
)

target_include_directories(scene_test PRIVATE ${CMAKE_SOURCE_DIR})

target_link_libraries(scene_test GTest::gtest_main)

include(GoogleTest)
gtest_discover_tests(scene_test)