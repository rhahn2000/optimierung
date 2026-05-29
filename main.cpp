#include "io/ObjLoader.h"
#include "io/ImageWriter.h"
#include "scene/Scene.h"
#include "core/Camera.h"
#include "scene/Light.h"
#include "core/Raytracer.h"
#include <iostream>
#include <chrono>
#include <cmath>
#include "core/AABB.h"
#include "core/KDTree.h"
 

#ifndef DATA_PATH
#define DATA_PATH "data/"
#endif
int main() {
    // small scene: 3 triangles in the XY-plane, spread along the X-axis
    std::vector<Triangle3df> triangles;
    std::vector<Vector3df> va, vb, vc;
 
    // triangle 0 – left  (x: -3..-1)
    va.push_back({-3.0f, -1.0f, 0.0f});
    vb.push_back({-2.0f,  1.0f, 0.0f});
    vc.push_back({-1.0f, -1.0f, 0.0f});
    triangles.emplace_back(va[0], vb[0], vc[0]);
 
    // triangle 1 – center  (x: -0.5..0.5)
    va.push_back({-0.5f, -1.0f, 0.0f});
    vb.push_back({ 0.0f,  1.0f, 0.0f});
    vc.push_back({ 0.5f, -1.0f, 0.0f});
    triangles.emplace_back(va[1], vb[1], vc[1]);
 
    // triangle 2 – right (x: 1..3)
    va.push_back({1.0f, -1.0f, 0.0f});
    vb.push_back({2.0f,  1.0f, 0.0f});
    vc.push_back({3.0f, -1.0f, 0.0f});
    triangles.emplace_back(va[2], vb[2], vc[2]);
 
    KDTree tree;
    tree.build(triangles, va, vb, vc);
    std::cout << "tree built\n";
 
    Intersection_Context<float, 3> context{};
    int mat_index = -1;
 
    // ray from above hitting triangle 1 (center)
    Ray3df ray1{ {0.0f, 0.0f, 5.0f}, {0.0f, 0.0f, -1.0f} };
    bool hit = tree.intersect(ray1, context, mat_index);
    std::cout << "hits triangle 1 (expected: 1): " << hit
              << "  mat_index=" << mat_index
              << "  t=" << context.t << "\n";
 
    // ray from above hitting triangle 0 (left)
    Ray3df ray2{ {-2.0f, 0.0f, 5.0f}, {0.0f, 0.0f, -1.0f} };
    context = {}; mat_index = -1;
    hit = tree.intersect(ray2, context, mat_index);
    std::cout << "hits triangle 0 (expected: 1): " << hit
              << "  mat_index=" << mat_index
              << "  t=" << context.t << "\n";
 
    // ray from above hitting triangle 2 (right)
    Ray3df ray3{ {2.0f, 0.0f, 5.0f}, {0.0f, 0.0f, -1.0f} };
    context = {}; mat_index = -1;
    hit = tree.intersect(ray3, context, mat_index);
    std::cout << "hits triangle 2 (expected: 1): " << hit
              << "  mat_index=" << mat_index
              << "  t=" << context.t << "\n";
 
    // ray missing all triangles
    Ray3df ray4{ {10.0f, 10.0f, 5.0f}, {0.0f, 0.0f, -1.0f} };
    context = {}; mat_index = -1;
    hit = tree.intersect(ray4, context, mat_index);
    std::cout << "hits nothing    (expected: 0): " << hit << "\n";

    return 0;
}