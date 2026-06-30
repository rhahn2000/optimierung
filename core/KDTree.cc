#include "KDTree.h"
#include <algorithm>
#include <limits>

KDTree::KDTree(const std::vector<Triangle3df> &triangles,
               const std::vector<Vector3df> &vertices_a,
               const std::vector<Vector3df> &vertices_b,
               const std::vector<Vector3df> &vertices_c)
    : root(nullptr),
      scene_triangles(triangles),
      v_a(vertices_a),
      v_b(vertices_b),
      v_c(vertices_c)
{
}

void KDTree::build()
{
    AABB root_box;
    for (size_t i = 0; i < scene_triangles.size(); ++i) {
        AABB tri_box = AABB::from_triangle(v_a[i], v_b[i], v_c[i]);
        root_box.grow(tri_box.min_pt);
        root_box.grow(tri_box.max_pt);
    }

    std::vector<int> indices(scene_triangles.size());
    for (size_t i = 0; i < scene_triangles.size(); ++i)
        indices[i] = static_cast<int>(i);

    root = build_recursive(indices, 0, root_box, v_a, v_b, v_c);
}

std::unique_ptr<KDNode> KDTree::build_recursive(std::vector<int> &indices,
                                                int depth,
                                                const AABB &current_box,
                                                const std::vector<Vector3df> &va,
                                                const std::vector<Vector3df> &vb,
                                                const std::vector<Vector3df> &vc)
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
        float centroid = (va[idx][axis] + vb[idx][axis] + vc[idx][axis]) / 3.0f;
        centroids.push_back(centroid);
    }

    auto median_it = centroids.begin() + centroids.size() / 2;
    std::nth_element(centroids.begin(), median_it, centroids.end());

    float split = *median_it;
    node->split_axis     = axis;
    node->split_position = split;

    // distribute by AABB overlap – triangles spanning the split go to both sides
    std::vector<int> left_indices, right_indices;
    for (int idx : indices) {
        float tri_min = std::min({va[idx][axis], vb[idx][axis], vc[idx][axis]});
        float tri_max = std::max({va[idx][axis], vb[idx][axis], vc[idx][axis]});
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

    node->left  = build_recursive(left_indices,  depth + 1, left_box,  va, vb, vc);
    node->right = build_recursive(right_indices, depth + 1, right_box, va, vb, vc);

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
            if (scene_triangles[idx].intersects(ray, tmp) && tmp.t > 1e-4f && tmp.t < closest_t) {
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

int KDTree::intersect_packet(const RayPacket& packet, int active_mask,
                              Intersection_Context<float, 3> contexts[8],
                              int mat_indices[8]) const
{
    if (!root) return 0;

    __m256 t_min, t_max;
    int hit_mask = packet.intersect_aabb(root->box, t_min, t_max);
    hit_mask &= active_mask;
    if (!hit_mask) return 0;

    // initialized once here and shared across all recursive calls so that
    // the closest hit per ray is tracked correctly across left and right subtrees
    float closest_t[8];
    for (int i = 0; i < 8; ++i)
        closest_t[i] = std::numeric_limits<float>::infinity();

    return intersect_recursive_packet(root.get(), packet, hit_mask, t_min, t_max, contexts, mat_indices, closest_t);
}

int KDTree::intersect_recursive_packet(const KDNode* node, const RayPacket& packet,
                                        int active_mask, __m256 t_min, __m256 t_max,
                                        Intersection_Context<float, 3> contexts[8],
                                        int mat_indices[8],
                                        float closest_t[8]) const
{
    if (!node || !active_mask) return 0;

    // --- leaf: test each active ray against all triangles, keep closest ---
    if (node->is_leaf) {
        int result_mask = 0;

        // unpack ray data once per leaf entry, not inside the triangle loop
        float origin_x[8], origin_y[8], origin_z[8];
        float direction_x[8], direction_y[8], direction_z[8];
        _mm256_storeu_ps(origin_x,    packet.origin_x);
        _mm256_storeu_ps(origin_y,    packet.origin_y);
        _mm256_storeu_ps(origin_z,    packet.origin_z);
        _mm256_storeu_ps(direction_x, packet.direction_x);
        _mm256_storeu_ps(direction_y, packet.direction_y);
        _mm256_storeu_ps(direction_z, packet.direction_z);

        for (int idx : node->triangle_indices) {
            const Triangle3df& tri = scene_triangles[idx];

            __m256 out_t;
            int tri_mask = packet.intersect_mt(tri, out_t) & active_mask;
            if (!tri_mask) continue;

            float t_vals[8];
            _mm256_storeu_ps(t_vals, out_t);

            for (int i = 0; i < 8; ++i) {
                if (!(tri_mask & (1 << i))) continue;
                // closest_t is shared across all recursive calls —
                // this prevents a farther hit in the right subtree from
                // overwriting a closer hit already found in the left subtree
                if (t_vals[i] > 1e-4f && t_vals[i] < closest_t[i]) {
                    closest_t[i] = t_vals[i];

                    Ray3df ray_i;
                    ray_i.origin    = { origin_x[i],    origin_y[i],    origin_z[i] };
                    ray_i.direction = { direction_x[i], direction_y[i], direction_z[i] };

                    Intersection_Context<float, 3> tmp{};
                    tri.intersects(ray_i, tmp);
                    tmp.normal.normalize();
                    if (tmp.normal * ray_i.direction > 0.0f)
                        tmp.normal *= -1.0f;

                    contexts[i]    = tmp;
                    mat_indices[i] = idx;
                    result_mask   |= (1 << i);
                }
            }
        }
        return result_mask;
    }

    // --- inner node: test both children, pass shared closest_t into each ---
    int result_mask = 0;

    if (node->left) {
        __m256 t_min_l, t_max_l;
        int left_mask = packet.intersect_aabb(node->left->box, t_min_l, t_max_l) & active_mask;
        if (left_mask)
            result_mask |= intersect_recursive_packet(node->left.get(), packet, left_mask,
                                                      t_min_l, t_max_l, contexts, mat_indices, closest_t);
    }

    if (node->right) {
        __m256 t_min_r, t_max_r;
        int right_mask = packet.intersect_aabb(node->right->box, t_min_r, t_max_r) & active_mask;
        if (right_mask)
            result_mask |= intersect_recursive_packet(node->right.get(), packet, right_mask,
                                                      t_min_r, t_max_r, contexts, mat_indices, closest_t);
    }

    return result_mask;
}