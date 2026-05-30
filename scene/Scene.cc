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
    vertices_a.push_back(Vector3df{0.0f, 0.0f, 0.0f});
    vertices_b.push_back(Vector3df{0.0f, 0.0f, 0.0f});
    vertices_c.push_back(Vector3df{0.0f, 0.0f, 0.0f});
}

void Scene::addTriangle(Triangle3df& triangle, Material& material, const Vector3df& a, const Vector3df& b, const Vector3df& c) {
    triangles.push_back(triangle);
    materials.push_back(material);
    triangles_mt.push_back(TriangleMT{a, b, c});
    vertices_a.push_back(a);
    vertices_b.push_back(b);
    vertices_c.push_back(c);
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
 
        // calculate edges
        Vector3df E1 = tri.b - tri.a;
        Vector3df E2 = tri.c - tri.a;
 
        // calculate determinante
        Vector3df h = ray.direction.cross_product(E2);
        float a = E1 * h;
 
        // check if ray is parallel to determinante
        if (std::fabs(a) < EPSILON) continue;
 
        // calculate u 
        float f = 1.0f / a;
        Vector3df s = ray.origin - tri.a;
        float u = f * (s * h);

        // check if u is inside triangle
        if (u < 0.0f || u > 1.0f) continue;
 
        // calculate v
        Vector3df q = s.cross_product(E1);
        float v = f * (ray.direction * q);

        // check if v is inside the triangle
        if (v < 0.0f || u + v > 1.0f) continue;
 
        // calculate t (distance to intersection)
        float t = f * (E2 * q);
 
        if (t > 1e-4f && t < t_min) {
            t_min = t;
            mat_index = static_cast<int>(i);
 
            // calculate intersection
            context.t = t;
            context.u = u;
            context.v = v;
            context.intersection = ray.origin + t * ray.direction;
 
            // calculate normal
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

bool Scene::intersect_kdtree(Ray3df& ray, Intersection_Context<float, 3>& context, int& mat_index) {
    return kd_tree.intersect(ray, context, mat_index);
}

void Scene::build_kd_tree() {
    kd_tree.build(triangles, vertices_a, vertices_b, vertices_c);
}

const std::vector<Light>& Scene::getLights() const {
    return lights;
}

const Material& Scene::getMaterial(int index) const {
    return materials[index];
}