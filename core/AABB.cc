#include "AABB.h"
#include <algorithm>
#include <limits>

AABB::AABB()
    : min_pt({ std::numeric_limits<float>::infinity(),
               std::numeric_limits<float>::infinity(),
               std::numeric_limits<float>::infinity() }),
      max_pt({-std::numeric_limits<float>::infinity(),
              -std::numeric_limits<float>::infinity(),
              -std::numeric_limits<float>::infinity() })
{
}

AABB::AABB(const Vector3df& min_p, const Vector3df& max_p)
    : min_pt(min_p), max_pt(max_p)
{
}

void AABB::grow(const Vector3df& point) {
    for (size_t i = 0; i < 3; ++i) {
        if (point[i] < min_pt[i]) min_pt[i] = point[i];
        if (point[i] > max_pt[i]) max_pt[i] = point[i];
    }
}

AABB AABB::from_triangle(const Vector3df& a, const Vector3df& b, const Vector3df& c)
{
    AABB box;
    box.grow(a);
    box.grow(b);
    box.grow(c);
    return box;
}

bool AABB::intersect(const Ray3df& ray, float& t_min, float& t_max) const {
    t_min = -std::numeric_limits<float>::infinity();
    t_max =  std::numeric_limits<float>::infinity();

    for (size_t i = 0; i < 3; ++i) {
        const float inv_dir = 1.0f / ray.direction[i];

        float t_near = (min_pt[i] - ray.origin[i]) * inv_dir;
        float t_far  = (max_pt[i] - ray.origin[i]) * inv_dir;

        if (t_near > t_far) std::swap(t_near, t_far);

        t_min = std::max(t_min, t_near);
        t_max = std::min(t_max, t_far);

        if (t_min > t_max) return false;
    }

    return t_max >= 0.0f;
}