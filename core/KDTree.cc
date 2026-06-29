#include "KDTree.h"
#include <algorithm>
#include <limits>

KDTree::KDTree()
    : root(nullptr),
      scene_triangles(nullptr),
      v_a(nullptr),
      v_b(nullptr),
      v_c(nullptr)
{
}

void KDTree::build(const std::vector<Triangle3df> &triangles,
                   const std::vector<Vector3df> &vertices_a,
                   const std::vector<Vector3df> &vertices_b,
                   const std::vector<Vector3df> &vertices_c)
{
    scene_triangles = &triangles;
    v_a = &vertices_a;
    v_b = &vertices_b;
    v_c = &vertices_c;

    AABB root_box;
    for (size_t i = 0; i < triangles.size(); ++i) {
        AABB tri_box = AABB::from_triangle(vertices_a[i], vertices_b[i], vertices_c[i]);
        root_box.grow(tri_box.min_pt);
        root_box.grow(tri_box.max_pt);
    }

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

    if (depth >= MAX_DEPTH || static_cast<int>(indices.size()) <= MIN_TRIANGLES) {
        node->is_leaf = true;
        node->triangle_indices = indices;
        return node;
    }

    float len_x = current_box.max_pt[0] - current_box.min_pt[0];
    float len_y = current_box.max_pt[1] - current_box.min_pt[1];
    float len_z = current_box.max_pt[2] - current_box.min_pt[2];

    int axis = 0;
    if (len_y > len_x && len_y > len_z) axis = 1;
    else if (len_z > len_x && len_z > len_y) axis = 2;

    // --- True median split based on triangle centroids ---
    std::vector<float> centroids;
    centroids.reserve(indices.size());
    for (int idx : indices) {
        float centroid = ((*v_a)[idx][axis] + (*v_b)[idx][axis] + (*v_c)[idx][axis]) / 3.0f;
        centroids.push_back(centroid);
    }

    auto median_it = centroids.begin() + centroids.size() / 2;
    std::nth_element(centroids.begin(), median_it, centroids.end());
    
    float split = *median_it; // median der dreiecke
    node->split_axis     = axis;
    node->split_position = split;

    // distribute by AABB overlap – triangles spanning the split go to both sides
    std::vector<int> left_indices, right_indices;
    for (int idx : indices) {
        float tri_min = std::min({(*v_a)[idx][axis], (*v_b)[idx][axis], (*v_c)[idx][axis]});
        float tri_max = std::max({(*v_a)[idx][axis], (*v_b)[idx][axis], (*v_c)[idx][axis]});
        if (tri_min <= split) left_indices.push_back(idx);
        if (tri_max >= split) right_indices.push_back(idx);
    }

    if (left_indices.size() == indices.size() || right_indices.size() == indices.size()) {
        node->is_leaf = true;
        node->triangle_indices = indices;
        return node;
    }

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

    // --- leaf: test all triangles, keep closest ---
    if (node->is_leaf) {
        bool hit = false;
        float closest_t = std::numeric_limits<float>::infinity();

        for (int idx : node->triangle_indices) {
            Intersection_Context<float, 3> tmp{};
            if ((*scene_triangles)[idx].intersects(ray, tmp) && tmp.t > 1e-4f && tmp.t < closest_t) {
                closest_t = tmp.t;
                context   = tmp;
                context.normal.normalize();
                // flip normal to point against ray direction (some mesh normals can face inward and without the flip trace could determine entering incorrectly if reflection/transmission)
                if (context.normal * ray.direction > 0.0f) {
                    context.normal *= -1.0f;
                }
                mat_index = idx;
                hit       = true;
            }
        }
        return hit;
    }

    // --- inner node: recurse into both children, keep closest result ---
    bool hit = false;
    float closest_t = std::numeric_limits<float>::infinity();
    Intersection_Context<float, 3> tmp_ctx{};
    int tmp_idx = -1;

    float t_min_l, t_max_l;
    if (node->left && node->left->box.intersect(ray, t_min_l, t_max_l)) {
        if (intersect_recursive(node->left.get(), ray, t_min_l, t_max_l, tmp_ctx, tmp_idx)) {
            if (tmp_ctx.t < closest_t) {
                closest_t = tmp_ctx.t;
                context   = tmp_ctx;
                mat_index = tmp_idx;
                hit       = true;
            }
        }
    }

    float t_min_r, t_max_r;
    if (node->right && node->right->box.intersect(ray, t_min_r, t_max_r)) {
        if (intersect_recursive(node->right.get(), ray, t_min_r, t_max_r, tmp_ctx, tmp_idx)) {
            if (tmp_ctx.t < closest_t) {
                closest_t = tmp_ctx.t;
                context   = tmp_ctx;
                mat_index = tmp_idx;
                hit       = true;
            }
        }
    }

    return hit;
}