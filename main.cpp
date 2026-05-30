#include "io/ObjLoader.h"
#include "io/ImageWriter.h"
#include "scene/Scene.h"
#include "core/Camera.h"
#include "scene/Light.h"
#include "core/Raytracer.h"
#include <iostream>
#include <chrono>
#include <cmath>

#ifndef DATA_PATH
#define DATA_PATH "data/"
#endif

static void print_hit(const std::string& name, bool hit,
                      const Intersection_Context<float, 3>& ctx,
                      int mat_idx, const Scene& scene)
{
    if (hit) {
        const Material& m = scene.getMaterial(mat_idx);
        std::cout << "[" << name << "] Hit!\n";
        std::cout << "  t            = " << ctx.t << "\n";
        std::cout << "  intersection = ("
                  << ctx.intersection[0] << ", "
                  << ctx.intersection[1] << ", "
                  << ctx.intersection[2] << ")\n";
        std::cout << "  normal       = ("
                  << ctx.normal[0] << ", "
                  << ctx.normal[1] << ", "
                  << ctx.normal[2] << ")\n";
        std::cout << "  triangle idx = " << mat_idx << "\n";
        std::cout << "  color        = ("
                  << m.color[0] << ", " << m.color[1] << ", " << m.color[2] << ")\n";
    } else {
        std::cout << "[" << name << "] No hit.\n";
    }
}

static double render_timed(Raytracer& rt, Camera& cam, Scene& scene)
{
    auto t0 = std::chrono::high_resolution_clock::now();
    rt.render(cam, scene);
    auto t1 = std::chrono::high_resolution_clock::now();
    return std::chrono::duration<double>(t1 - t0).count();
}

static void print_time(const std::string& label, double seconds)
{
    int min      = static_cast<int>(seconds) / 60;
    int sec      = static_cast<int>(seconds) % 60;
    int ms       = static_cast<int>((seconds - static_cast<int>(seconds)) * 1000);
    int total_ms = static_cast<int>(seconds * 1000);
    std::cout << "Render time [" << label << "]: "
              << min << " min " << sec << " sec " << ms << " ms"
              << "  (" << total_ms << " ms total)\n";
}

int main() {
    Scene scene;
    ObjLoader loader;
    loader.load(std::string(DATA_PATH) + "teapot_n_glass.obj", scene);

    Light light(
        Vector3df{  0.0f, 15.0f, -8.0f },
        Vector3df{  1.0f,  1.0f,  1.0f }
    );
    scene.addLight(light);

    // -----------------------------------------------------------------------
    // Intersection test – all three methods
    // -----------------------------------------------------------------------
    Ray3df ray{ Vector3df{0.0f, 10.0f, 0.0f}, Vector3df{0.0f, -1.0f, 0.0f} };

    Intersection_Context<float, 3> ctx_b, ctx_mt, ctx_kd;
    int idx_b = -1, idx_mt = -1, idx_kd = -1;

    bool hit_b  = scene.intersect(ray, ctx_b, idx_b);
    bool hit_mt = scene.intersect_mt(ray, ctx_mt, idx_mt);
    bool hit_kd = scene.intersect_kdtree(ray, ctx_kd, idx_kd);

    print_hit("Badouel",          hit_b,  ctx_b,  idx_b,  scene);
    std::cout << "\n";
    print_hit("Moeller-Trumbore", hit_mt, ctx_mt, idx_mt, scene);
    std::cout << "\n";
    print_hit("KD-Tree",          hit_kd, ctx_kd, idx_kd, scene);

    if (hit_b && hit_mt && hit_kd) {
        float dt_mt = std::abs(ctx_b.t - ctx_mt.t);
        float dt_kd = std::abs(ctx_b.t - ctx_kd.t);
        std::cout << "\nDeviation Badouel vs MT:     " << dt_mt
                  << (dt_mt < 1e-3f ? " -> OK" : " -> MISMATCH!") << "\n";
        std::cout << "Deviation Badouel vs KDTree: " << dt_kd
                  << (dt_kd < 1e-3f ? " -> OK" : " -> MISMATCH!") << "\n";
    }

    // -----------------------------------------------------------------------
    // Render + timing  (render() uses intersect_kdtree internally)
    // -----------------------------------------------------------------------
    const int W = 800, H = 600;
    const float aspect = static_cast<float>(W) / H;

    Camera camera(
        Vector3df{ 0.0f,  5.0f, -12.0f },
        Vector3df{ 0.0f,  1.0f,  -1.0f },
        Vector3df{ 0.0f,  1.0f,   0.0f },
        45.0f, aspect
    );

    std::cout << "\nStarting render [KD-Tree]...\n";
    Raytracer rt(W, H, 5);
    double t = render_timed(rt, camera, scene);
    print_time("KD-Tree", t);

    ImageWriter writer;
    writer.write_ppm("raytracer_output.ppm", rt.getFramebuffer(), W, H);

    return 0;
}