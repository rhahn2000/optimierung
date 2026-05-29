#ifndef CORE_KDTREE_H
#define CORE_KDTREE_H

#include "AABB.h"
#include "../geometry/geometry.h"
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
         * Creates a new KDTree.
         * @brief Constructor of KDTree
         */
        KDTree();
        /**
         * Destroys the KDTree.
         * @brief Deconstructor of KDTree
         */
        ~KDTree();

        /**
         * @brief starts building a tree after all objects are in scene
         * @param triangles list of all triangles
         * @param vertices_a list of all vertices a
         * @param vertices_b list of all vertices b
         * @param vertices_c list of all vertices c
         * @returns void
         */
        void build(const std::vector<Triangle3df> &triangles, const std::vector<Vector3df> &vertices_a,
                const std::vector<Vector3df> &vertices_b, const std::vector<Vector3df> &vertices_c);

        /**
         * @brief checks if a ray hits root. if it hits the root, recursion will start.
         * @param ray ray to check
         * @param context 
         * @param mat_index index of material
         * @return bool; true if intersection
         */
        bool intersect(Ray3df &ray, Intersection_Context<float, 3> &context, int &mat_index) const;

    private:
        std::unique_ptr<KDNode> root;

        const std::vector<Triangle3df> *scene_triangles;
        const std::vector<Vector3df> *v_a;
        const std::vector<Vector3df> *v_b;
        const std::vector<Vector3df> *v_c;

        const int MAX_DEPTH = 20;
        const int MIN_TRIANGLES = 4;

        /**
         * Rekursiver Bauprozess
         */
        /**
         * @brief helper method for recursive tree building
         * @param indices list of indices of triangles in section
         * @param depth current depth in tree
         * @param current_box box of current node
         * @returns pointer to finished node / branch
         */
        std::unique_ptr<KDNode> build_recursive(std::vector<int> &indices, int depth, const AABB &current_box);

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
};

#endif // CORE_KDTREE_H