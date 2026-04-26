#ifndef SCENE_H
#define SCENE_H

#include "../math/math.h"
#include "../geometry/geometry.h"
#include "Material.h"
#include "Light.h"
#include <vector>

/**
 * @class Scene
 * @brief Scene class. Contains all objects of the enviornment/image for the ray tracer. 
 */
class Scene {
    public:
        /**
         * @brief adds a triangle to the scene.
         * @param triangle the triangle to add
         * @param material the material of the triangle
         * @return void
         */
        void addTriangle(Triangle3df& triangle, Material& material);
        /**
         * @brief adds a light to the scene.
         * @param light the light to add
         * @return void
         */
        void addLight(Light& light);
        /**
         * @brief check which triangle is closest to the ray
         * @param ray the ray to check
         * @param context
         * @param mat_index the index of the material
         * @return boolean; true if ray hits a triangle
         */
        bool intersect(Ray3df& ray, Intersection_Context<float, 3>& context, int& mat_index);
        /**
         * @brief Getter of lights
         * @return all lights of the scene
         */
        const std::vector<Light>& getLights() const;
        /**
         * @brief Getter of a specific material
         * @param index index of the material
         * @return material
         */
        const Material& getMaterial(int index) const;
    private:
        std::vector<Triangle3df> triangles;
        std::vector<Light> lights;
        std::vector<Material> materials;
};
#endif