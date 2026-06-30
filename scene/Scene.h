#ifndef SCENE_H
#define SCENE_H

#include "../math/math.h"
#include "../geometry/geometry.h"
#include "Material.h"
#include "Light.h"
#include "../core/KDTree.h"
#include "../core/RayPacket.h"
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
         * @brief intersection calculation using kd tree
         * @param ray the ray to check
         * @param context 
         * @param mat_index the index of material
         * @return boolean; true if ray hits triangle
         */
        bool intersect_kdtree(Ray3df& ray, Intersection_Context<float, 3>& context, int& mat_index);
        /**
         * @brief intersection calculation using kd tree packet tracing (AVX)
         * @param packet the ray packet to check
         * @param active_mask bitmask of active rays
         * @param contexts [out] per-ray intersection contexts
         * @param mat_indices [out] per-ray material indices
         * @return bitmask — bit i set iff ray i hit something
         */
        int intersect_packet(const RayPacket& packet, int active_mask,
                             Intersection_Context<float, 3> contexts[8], int mat_indices[8]);
        /**
         * @brief builds the kd tree after objloader finished
         * @return void
         */
        void build_kd_tree();
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
        std::vector<Vector3df> vertices_a;
        std::vector<Vector3df> vertices_b;
        std::vector<Vector3df> vertices_c;
        KDTree kd_tree{triangles, vertices_a, vertices_b, vertices_c};
};
#endif