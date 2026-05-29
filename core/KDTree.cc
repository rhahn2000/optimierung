#include "KDTree.h"
#include <algorithm>
#include <limits>

KDTree::KDTree()
    : root(nullptr), scene_triangles(nullptr),
      v_a(nullptr), v_b(nullptr), v_c(nullptr)
{
}

KDTree::~KDTree() = default;

void KDTree::build(const std::vector<Triangle3df> &triangles,
                   const std::vector<Vector3df> &vertices_a,
                   const std::vector<Vector3df> &vertices_b,
                   const std::vector<Vector3df> &vertices_c)
{
    scene_triangles = &triangles;
    v_a = &vertices_a;
    v_b = &vertices_b;
    v_c = &vertices_c;

    // build the root AABB over all triangles
    AABB root_box;
    for (size_t i = 0; i < triangles.size(); ++i) {
        AABB tri_box = AABB::from_triangle(vertices_a[i], vertices_b[i], vertices_c[i]);
        root_box.grow(tri_box.min_pt);
        root_box.grow(tri_box.max_pt);
    }

    // all triangle indices go into the root
    std::vector<int> indices(triangles.size());
    for (size_t i = 0; i < triangles.size(); ++i)
        indices[i] = static_cast<int>(i);

    root = build_recursive(indices, 0, root_box);
}

std::unique_ptr<KDNode> KDTree::build_recursive(std::vector<int> &indices,
                                                  int depth,
                                                  const AABB &current_box)
{
    auto node = std::make_unique<KDNode>();
    node->box = current_box;

    // --- leaf condition ---
    if (depth >= MAX_DEPTH || static_cast<int>(indices.size()) <= MIN_TRIANGLES) {
        node->is_leaf = true;
        node->triangle_indices = indices;
        return node;
    }

    // --- find longest axis ---
    float len_x = current_box.max_pt[0] - current_box.min_pt[0];
    float len_y = current_box.max_pt[1] - current_box.min_pt[1];
    float len_z = current_box.max_pt[2] - current_box.min_pt[2];

    int axis = 0;
    if (len_y > len_x && len_y > len_z) axis = 1;
    else if (len_z > len_x && len_z > len_y) axis = 2;

    // --- split at center of box along that axis ---
    float split = 0.5f * (current_box.min_pt[axis] + current_box.max_pt[axis]);
    node->split_axis     = axis;
    node->split_position = split;

    // --- distribute triangles by centroid ---
    std::vector<int> left_indices, right_indices;
    for (int idx : indices) {
        float centroid = ((*v_a)[idx][axis] + (*v_b)[idx][axis] + (*v_c)[idx][axis]) / 3.0f;
        if (centroid <= split)
            left_indices.push_back(idx);
        else
            right_indices.push_back(idx);
    }

    // --- avoid infinite recursion if split does not separate triangles ---
    if (left_indices.empty() || right_indices.empty()) {
        node->is_leaf = true;
        node->triangle_indices = indices;
        return node;
    }

    // --- build child AABBs and recurse ---
    AABB left_box  = current_box;
    AABB right_box = current_box;
    left_box.max_pt[axis]  = split;
    right_box.min_pt[axis] = split;

    node->left  = build_recursive(left_indices,  depth + 1, left_box);
    node->right = build_recursive(right_indices, depth + 1, right_box);

    return node;
}

bool KDTree::intersect(Ray3df &ray, Intersection_Context<float, 3> &context,
                        int &mat_index) const
{
    if (!root) return false;

    float t_min, t_max;
    if (!root->box.intersect(ray, t_min, t_max)) return false;

    return intersect_recursive(root.get(), ray, t_min, t_max, context, mat_index);
}

bool KDTree::intersect_recursive(const KDNode *node, Ray3df &ray,
                                   float t_min, float t_max,
                                   Intersection_Context<float, 3> &context,
                                   int &mat_index) const
{
    if (!node) return false;

    // --- leaf: test all triangles ---
    if (node->is_leaf) {
        bool hit = false;
        float closest_t = std::numeric_limits<float>::infinity();

        for (int idx : node->triangle_indices) {
            Intersection_Context<float, 3> tmp{};
            if ((*scene_triangles)[idx].intersects(ray, tmp) && tmp.t < closest_t) {
                closest_t = tmp.t;
                context   = tmp;
                mat_index = idx;
                hit       = true;
            }
        }
        return hit;
    }

    // --- inner node: recurse into children whose boxes the ray hits ---
    bool hit = false;
    float t_min_child, t_max_child;

    if (node->left && node->left->box.intersect(ray, t_min_child, t_max_child))
        hit |= intersect_recursive(node->left.get(), ray, t_min_child, t_max_child, context, mat_index);

    if (node->right && node->right->box.intersect(ray, t_min_child, t_max_child))
        hit |= intersect_recursive(node->right.get(), ray, t_min_child, t_max_child, context, mat_index);

    return hit;
}