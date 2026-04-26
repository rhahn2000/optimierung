#include "Scene.h"
#include <limits>

void Scene::addTriangle(Triangle3df& triangle, Material& material) {
    triangles.push_back(triangle);
    materials.push_back(material);
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

const std::vector<Light>& Scene::getLights() const {
    return lights;
}

const Material& Scene::getMaterial(int index) const {
    return materials[index];
}