#include <iostream>
#include "math/math.h"
#include "geometry/geometry.h"

int main() {
    std::cout << "Raytracer startet..." << std::endl;
    // Test-Vektoren
    Vector3df pos{0.0f, 0.0f, 0.0f};
    Vector3df target{0.0f, 0.0f, -1.0f};
    Vector3df up{0.0f, 1.0f, 0.0f};
    // Camera cam(pos, target, up, 90.0f, 1.6f); // Sobald Camera.cc existiert
    return 0;
}