#ifndef CORE_KDTREE_H
#define CORE_KDTREE_H

#include "AABB.h"
#include "../geometry/geometry.h"
#include "../core/RayPacket.h"
#include <immintrin.h>
#include <vector>
#include <memory>

/**
 * @struct KDNode
 * @brief KDNode struct. Represents a node of the tree.
 */
struct KDNode
{
    AABB box;
    // pointers to child nodes
    std::unique_ptr<KDNode> left;
    std::unique_ptr<KDNode> right;

    std::vector<int> triangle_indices; // list of triangles in scene
    float split_position;
    int split_axis;
    bool is_leaf; // leaf == end of tree / branch

    /**
     * Creates a new KDNode.
     * @brief Constructor of KDNode
     */
    KDNode() : split_position(0.0f), split_axis(0), is_leaf(false) {}
};


/**
 * @class KDTree
 * @brief KDTree class. Tree for optimization.
 */
class KDTree {
    public:
        /**
         * @brief Constructor of KDTree
         * @param triangles list of all triangles
         * @param vertices_a list of all vertices a
         * @param vertices_b list of all vertices b
         * @param vertices_c list of all vertices c
         */
        KDTree(const std::vector<Triangle3df> &triangles,
               const std::vector<Vector3df> &vertices_a,
               const std::vector<Vector3df> &vertices_b,
               const std::vector<Vector3df> &vertices_c);

        /**
         * @brief builds the kd-tree from the scene data provided in the constructor.
         * @returns void
         */
        void build();

        /**
         * @brief checks if a ray hits root. if it hits the root, recursion will start.
         * @param ray ray to check
         * @param context 
         * @param mat_index index of material
         * @return bool; true if intersection
         */
        bool intersect(Ray3df &ray, Intersection_Context<float, 3> &context, int &mat_index) const;

        /**
         * @brief tests all 8 rays of a packet against the kd-tree using AVX slab tests.
         *        for each node, if no ray in the packet hits the box, the entire subtree is skipped.
         * @param packet the ray packet to test
         * @param active_mask bitmask of rays that are still active (not yet terminated)
         * @param contexts intersection contexts
         * @param mat_indices material indices
         * @return bitmask
         */
        int intersect_packet(const RayPacket& packet, int active_mask,
                             Intersection_Context<float, 3> contexts[8], int mat_indices[8]) const;

    private:
        std::unique_ptr<KDNode> root;

        const std::vector<Triangle3df> &scene_triangles;
        const std::vector<Vector3df>   &v_a;
        const std::vector<Vector3df>   &v_b;
        const std::vector<Vector3df>   &v_c;

        const int MAX_DEPTH = 20;
        const int MIN_TRIANGLES = 4;

        /**
         * @brief helper method for recursive tree building
         * @param indices list of indices of triangles in section
         * @param depth current depth in tree
         * @param current_box box of current node
         * @param va vertices a of all triangles
         * @param vb vertices b of all triangles
         * @param vc vertices c of all triangles
         * @returns pointer to finished node / branch
         */
        std::unique_ptr<KDNode> build_recursive(std::vector<int> &indices, int depth,
                                                const AABB &current_box,
                                                const std::vector<Vector3df> &va,
                                                const std::vector<Vector3df> &vb,
                                                const std::vector<Vector3df> &vc);

        /**
         * @brief helper method for recursive intersection search
         * @param node current node of tree to check
         * @param ray ray to check
         * @param t_min time distance between origin and entry point of box
         * @param t_max time distance between origin and exit point of box
         * @param context
         * @param mat_index index of material
         * @returns bool; true if intersection
         */
        bool intersect_recursive(const KDNode *node, Ray3df &ray, float t_min, float t_max,
                                Intersection_Context<float, 3> &context, int &mat_index) const;

        /**
         * @brief recursive helper for packet traversal
         * @param node current node
         * @param packet the ray packet
         * @param active_mask bitmask of rays still active in this subtree
         * @param t_min entry distances into this node
         * @param t_max exit distances from this node
         * @param contexts  intersection contexts
         * @param mat_indices material indices
         * @param closest_t closest hit distances, shared across all recursive calls
         * @returns bitmask
         */
        int intersect_recursive_packet(const KDNode* node, const RayPacket& packet,
                                       int active_mask, __m256 t_min, __m256 t_max,
                                       Intersection_Context<float, 3> contexts[8],
                                       int mat_indices[8],
                                       float closest_t[8]) const;
};

#endif // CORE_KDTREE_H