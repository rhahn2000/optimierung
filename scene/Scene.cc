#include "Scene.h"
#include <limits>

void Scene::addTriangle(Triangle3df& triangle, Material& material) {
    triangles.push_back(triangle);
    materials.push_back(material);
    triangles_mt.push_back(TriangleMT{
        Vector3df{0.0f, 0.0f, 0.0f},
        Vector3df{0.0f, 0.0f, 0.0f},
        Vector3df{0.0f, 0.0f, 0.0f}
    });
}

void Scene::addTriangle(Triangle3df& triangle, Material& material, const Vector3df& a, const Vector3df& b, const Vector3df& c) {
    triangles.push_back(triangle);
    materials.push_back(material);
    triangles_mt.push_back(TriangleMT{a, b, c});
}
 

void Scene::addLight(Light& light) {
    lights.push_back(light);
}

bool Scene::intersect(Ray3df& ray, Intersection_Context<float, 3>& context, int& mat_index) {
    float t_min = std::numeric_limits<float>::infinity();
    bool hit = false;
    Intersection_Context<float, 3> tmp_context;

    for (size_t i = 0; i < triangles.size(); i++) {
        if (triangles[i].intersects(ray, tmp_context)) {
            if (tmp_context.t > 1e-4f && tmp_context.t < t_min) {
                t_min = tmp_context.t;
                context = tmp_context;
                context.normal.normalize();
                mat_index = static_cast<int>(i);
                hit = true;
            }
        }
    }
    return hit;
}

bool Scene::intersect_mt(Ray3df& ray, Intersection_Context<float, 3>& context, int& mat_index) {
    const float EPSILON = 1e-8f;
    float t_min = std::numeric_limits<float>::infinity();
    bool hit = false;
 
    for (size_t i = 0; i < triangles_mt.size(); i++) {
        const TriangleMT& tri = triangles_mt[i];
 
        Vector3df E1 = tri.b - tri.a;
        Vector3df E2 = tri.c - tri.a;
 
        // h = D x E2
        Vector3df h = ray.direction.cross_product(E2);
 
        // Determinante a = E1 · h
        float a = E1 * h;
 
        // ray parallel to triangle -> no hit
        if (std::fabs(a) < EPSILON) continue;
 
        float f = 1.0f / a;
 
        // s = O - A
        Vector3df s = ray.origin - tri.a;
 
        // u = f * (s · h)
        float u = f * (s * h);
        if (u < 0.0f || u > 1.0f) continue;
 
        // q = s x E1
        Vector3df q = s.cross_product(E1);
 
        // v = f * (D · q)
        float v = f * (ray.direction * q);
        if (v < 0.0f || u + v > 1.0f) continue;
 
        // t = f * (E2 · q)
        float t = f * (E2 * q);
 
        if (t > 1e-4f && t < t_min) {
            t_min = t;
            mat_index = static_cast<int>(i);
 
            // calculate intersection
            context.t = t;
            context.u = u;
            context.v = v;
            context.intersection = ray.origin + t * ray.direction;
 
            // Normal = E1 x E2
            context.normal = E1.cross_product(E2);
            context.normal.normalize();
 
            // rotate normal if needed
            if (context.normal * ray.direction > 0.0f) {
                context.normal *= -1.0f;
            }
 
            hit = true;
        }
    }
    return hit;
}

const std::vector<Light>& Scene::getLights() const {
    return lights;
}

const Material& Scene::getMaterial(int index) const {
    return materials[index];
}