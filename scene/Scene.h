#ifndef SCENE_H
#define SCENE_H

#include "../math/math.h"
#include "../geometry/geometry.h"
#include "Material.h"
#include "Light.h"
#include <vector>
#include <array>

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
         * @brief overloaded addTriangle for moeller trumbore
         * @see addTriangle
         * @param triangle the triangle to add
         * @param material the material of the triangle
         * @param a vertex a
         * @param b vertex b
         * @param c vertex c
         */
        void addTriangle(Triangle3df& triangle, Material& material, const Vector3df& a, const Vector3df& b, const Vector3df& c);
        /**
         * @brief adds a light to the scene.
         * @param light the light to add
         * @return void
         */
        void addLight(Light& light);
        /**
         * @brief intersection calculation using badouel
         * @param ray the ray to check
         * @param context
         * @param mat_index the index of the material
         * @return boolean; true if ray hits a triangle
         */
        bool intersect(Ray3df& ray, Intersection_Context<float, 3>& context, int& mat_index);
        /**
         * @brief intersection calculation using moeller trumbore
         * @param ray the ray to check
         * @param context 
         * @param mat_index the index of material
         * @return boolean; true if ray hits triangle
         */
        bool intersect_mt(Ray3df& ray, Intersection_Context<float, 3>& context, int& mat_index);
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
        struct TriangleMT {
            Vector3df a, b, c;
        };
        std::vector<TriangleMT> triangles_mt;
};
#endif