add_executable(io_test
    ImageWriter_Test.cc
    ImageWriter.cc
    ObjLoader_Test.cc
    ObjLoader.cc
    ${CMAKE_SOURCE_DIR}/math/math.cc
    ${CMAKE_SOURCE_DIR}/geometry/geometry.cc
    ${CMAKE_SOURCE_DIR}/scene/Scene.cc
    ${CMAKE_SOURCE_DIR}/scene/Material.cc
    ${CMAKE_SOURCE_DIR}/scene/Light.cc
    ${CMAKE_SOURCE_DIR}/core/KDTree.cc
    ${CMAKE_SOURCE_DIR}/core/AABB.cc
    ${CMAKE_SOURCE_DIR}/core/RayPacket.cc
)

target_include_directories(io_test PRIVATE ${CMAKE_SOURCE_DIR})

target_link_libraries(io_test GTest::gtest_main)

include(GoogleTest)
gtest_discover_tests(io_test)