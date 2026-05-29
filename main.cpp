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
 

#ifndef DATA_PATH
#define DATA_PATH "data/"
#endif
int main() {
     // --- from_triangle ---
    Vector3df a = {0.0f, 0.0f, 0.0f};
    Vector3df b = {2.0f, 4.0f, 0.0f};
    Vector3df c = {4.0f, 0.0f, 3.0f};
    AABB box = AABB::from_triangle(a, b, c);
    std::cout << "from_triangle:\n";
    std::cout << "  min: " << box.min_pt[0] << " " << box.min_pt[1] << " " << box.min_pt[2] << "\n";
    std::cout << "  max: " << box.max_pt[0] << " " << box.max_pt[1] << " " << box.max_pt[2] << "\n";
    // expected: min=(0,0,0)  max=(4,4,3)
 
    // --- grow ---
    box.grow({-1.0f, 5.0f, 10.0f});
    std::cout << "after grow(-1, 5, 10):\n";
    std::cout << "  min: " << box.min_pt[0] << " " << box.min_pt[1] << " " << box.min_pt[2] << "\n";
    std::cout << "  max: " << box.max_pt[0] << " " << box.max_pt[1] << " " << box.max_pt[2] << "\n";
    // expected: min=(-1,0,0)  max=(4,5,10)
 
    // --- slab test: ray hits box ---
    Ray3df ray_hit{ {2.0f, 2.0f, -5.0f}, {0.0f, 0.0f, 1.0f} };
    float t_min, t_max;
    bool hit = box.intersect(ray_hit, t_min, t_max);
    std::cout << "\nray hits box (expected: 1): " << hit << "  t_min=" << t_min << " t_max=" << t_max << "\n";
 
    // --- slab test: ray misses box ---
    Ray3df ray_miss{ {10.0f, 10.0f, -5.0f}, {0.0f, 0.0f, 1.0f} };
    hit = box.intersect(ray_miss, t_min, t_max);
    std::cout << "ray misses box (expected: 0): " << hit << "\n";
 
    // --- slab test: box behind ray origin ---
    Ray3df ray_behind{ {2.0f, 2.0f, 20.0f}, {0.0f, 0.0f, 1.0f} };
    hit = box.intersect(ray_behind, t_min, t_max);
    std::cout << "box behind ray  (expected: 0): " << hit << "\n";

    return 0;
}