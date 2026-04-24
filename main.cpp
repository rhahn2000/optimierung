#include <iostream>
#include "math/math.h"
#include "geometry/geometry.h"

int main() {
    std::cout << "Hello World" << std::endl;
    Vector3df v1 = {1.0f, 0.0f, 0.0f};
    Vector3df v2 = {0.0f, 1.0f, 0.0f};
    // Vector3df v3 = v1.cross_product(v2);
    // std::cout << v3[0] << " " << v3[1] << " " << v3[2] << std::endl;
    Vector3df v3 = {0.0f, 0.0f, 1.0f};
    Triangle3df triangle(v1, v2, v3);
    Ray3df ray = { 
        {0.2f, 1.0f, 0.2f}, // origin
        {0.0f, -1.0f, 0.0f}  // direction
    };
    Intersection_Context<float, 3u> context;
    
    if (triangle.intersects(ray, context)) {
        std::cout << "Treffer!" << std::endl;
        std::cout << "Abstand t: " << context.t << std::endl;
        std::cout << "Schnittpunkt: (" << context.intersection[0] << ", " 
                  << context.intersection[1] << ", " << context.intersection[2] << ")" << std::endl;
        std::cout << "Baryzentrische Koord: u=" << context.u << ", v=" << context.v << std::endl;
    } else {
        std::cout << "Kein Treffer." << std::endl;
    }
    return 0;
}