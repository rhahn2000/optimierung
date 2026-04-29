#include "Camera.h"
#include <cmath>

Camera::Camera(Vector3df& position, Vector3df& target, Vector3df& vup, float vangle, float aspectRatio) {
    float theta      = vangle * static_cast<float>(PI) / 180.0f;
    float half_h     = std::tan(theta / 2.0f);
    float half_w     = aspectRatio * half_h;

    // Local camera coordinate system (right-handed)
    // w = points from target to camera position
    Vector3df w = position - target;
    w.normalize();

    // u = right vector
    Vector3df u = vup.cross_product(w);
    u.normalize();

    // v = true up vector
    Vector3df v = w.cross_product(u);

    origin            = position;
    horizontal        = 2.0f * half_w  * u;
    vertical          = 2.0f * half_h  * v;
    lower_left_corner = origin - half_w * u - half_h * v - w;
}

Ray3df Camera::get_ray(float u, float v) {
    Vector3df direction = lower_left_corner + u * horizontal + v * vertical - origin;
    direction.normalize();
    return Ray3df{ origin, direction };
}