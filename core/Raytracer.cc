#include "Raytracer.h"
#include <algorithm>
#include <cmath>

Raytracer::Raytracer(int width, int height, int max_depth)
    : width(width), height(height), max_depth(max_depth),
      framebuffer(width * height)
{
}

void Raytracer::render(Camera cam, Scene scene) {
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            float u = static_cast<float>(x) / (width  - 1);
            float v = static_cast<float>(height - 1 - y) / (height - 1); // flip y

            Ray3df ray = cam.get_ray(u, v);
            framebuffer[y * width + x] = trace(ray, scene, max_depth);
        }
    }
}

// Returns the list of lights that are visible from the hit point (no shadow)
static std::vector<Light> find_visible_lights(
        const Intersection_Context<float, 3>& ctx,
        Scene& scene)
{
    std::vector<Light> visible;
    const float EPSILON = 1e-3f;

    for (const Light& light : scene.getLights()) {
        // Shadow ray from hit point towards light (offset by epsilon to avoid self-intersection / shadow acne)
        Vector3df shadow_dir = light.position - ctx.intersection;
        float dist_to_light  = shadow_dir.length();
        shadow_dir.normalize();

        Vector3df shadow_origin = ctx.intersection + EPSILON * ctx.normal;
        Ray3df shadow_ray{ shadow_origin, shadow_dir };

        Intersection_Context<float, 3> shadow_ctx;
        int shadow_mat = -1;

        // Light is visible if no object is hit, or hit is beyond the light
        if (!scene.intersect(shadow_ray, shadow_ctx, shadow_mat)
            || shadow_ctx.t > dist_to_light)
        {
            visible.push_back(light);
        }
    }
    return visible;
}

// Lambertian shading with ambient term and multiple lights
// L = ka + kd * (1/n) * sum_i( I_i * max(0, n . l_i) )
static Vector3df lambertian(
        const Intersection_Context<float, 3>& ctx,
        const Material& mat,
        const std::vector<Light>& visible_lights,
        int total_lights)
{
    const float ka = 0.2f; // ambient coefficient

    Vector3df color = ka * mat.color;

    if (visible_lights.empty() || total_lights == 0) {
        return color;
    }

    Vector3df diffuse_sum{0.0f, 0.0f, 0.0f};
    for (const Light& light : visible_lights) {
        Vector3df l = light.position - ctx.intersection;
        l.normalize();

        float cos_theta = std::max(0.0f, ctx.normal * l);
        // element-wise multiply light intensity with material color
        diffuse_sum += cos_theta * Vector3df{
            light.intensity[0] * mat.color[0],
            light.intensity[1] * mat.color[1],
            light.intensity[2] * mat.color[2]
        };
    }

    // Average over all lights (including shadowed ones) to prevent overflow
    color += mat.color[0] > 0.0f || mat.color[1] > 0.0f || mat.color[2] > 0.0f
        ? (mat.reflectivity < 1.0f ? (1.0f - mat.reflectivity) : 1.0f)
          * (1.0f / total_lights) * diffuse_sum
        : diffuse_sum;

    // Clamp
    color[0] = std::min(color[0], 1.0f);
    color[1] = std::min(color[1], 1.0f);
    color[2] = std::min(color[2], 1.0f);

    return color;
}

// Schlick approximation for reflectance at a surface
static float schlick(float cos_theta, float eta1, float eta2) {
    float r0 = (eta1 - eta2) / (eta1 + eta2);
    r0 = r0 * r0;
    return r0 + (1.0f - r0) * std::pow(1.0f - cos_theta, 5.0f);
}

Vector3df Raytracer::trace(Ray3df& ray, Scene& scene, int depth) {
    const Vector3df BACKGROUND{0.2f, 0.2f, 0.2f};
    const float EPSILON = 1e-3f;

    if (depth == 0) {
        return BACKGROUND;
    }

    Intersection_Context<float, 3> ctx;
    int mat_idx = -1;

    if (!scene.intersect(ray, ctx, mat_idx)) {
        return BACKGROUND;
    }

    const Material& mat = scene.getMaterial(mat_idx);
    Vector3df color{0.0f, 0.0f, 0.0f};

    // --- Reflection ---
    if (mat.reflectivity > 0.0f) {
        Vector3df refl_dir = ray.direction.get_reflective(ctx.normal);
        Vector3df refl_origin = ctx.intersection + EPSILON * ctx.normal;
        Ray3df refl_ray{ refl_origin, refl_dir };
        color += mat.reflectivity * trace(refl_ray, scene, depth - 1);
    }

    // --- Transmission / Refraction (Whitted-style with Schlick) ---
    if (mat.transparency > 0.0f) {
        // Determine if ray is entering or leaving the material
        bool entering    = (ray.direction * ctx.normal) < 0.0f;
        float eta1       = entering ? 1.0f : mat.refraction_index;
        float eta2       = entering ? mat.refraction_index : 1.0f;
        Vector3df normal = entering ? ctx.normal : Vector3df{-ctx.normal[0], -ctx.normal[1], -ctx.normal[2]};

        float cos_theta = std::max(0.0f, -(ray.direction * normal));
        float R = schlick(cos_theta, eta1, eta2);

        Vector3df refr_transmission{0.0f, 0.0f, 0.0f};
        bool total_internal_reflection = !refract(eta1 / eta2, normal, ray.direction, refr_transmission);

        if (!total_internal_reflection) {
            // Refracted ray
            refr_transmission.normalize();
            Vector3df refr_origin = ctx.intersection - EPSILON * normal;
            Ray3df refr_ray{ refr_origin, refr_transmission };
            color += mat.transparency * (1.0f - R) * trace(refr_ray, scene, depth - 1);
        }

        // Reflective part of transparent surface (Schlick)
        Vector3df refl_dir = ray.direction.get_reflective(normal);
        Vector3df refl_origin = ctx.intersection + EPSILON * normal;
        Ray3df refl_ray{ refl_origin, refl_dir };
        color += mat.transparency * R * trace(refl_ray, scene, depth - 1);
    }

    // --- Lambertian diffuse shading + shadows ---
    std::vector<Light> visible_lights = find_visible_lights(ctx, scene);
    int total_lights = static_cast<int>(scene.getLights().size());
    color += lambertian(ctx, mat, visible_lights, total_lights);

    // Clamp final color
    color[0] = std::min(color[0], 1.0f);
    color[1] = std::min(color[1], 1.0f);
    color[2] = std::min(color[2], 1.0f);

    return color;
}