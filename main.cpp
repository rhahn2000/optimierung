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
 
int main() {
    Scene scene;
    ObjLoader loader;
 
    // load object file and its material file into the scene
    loader.load(std::string(DATA_PATH) + "teapot_n_glass.obj", scene);

    // create a light source and add it to the scene
    Light light(
        Vector3df{  0.0f, 15.0f, -8.0f },
        Vector3df{  1.0f,  1.0f,  1.0f }
    );
    scene.addLight(light);

    // intersection test
    Ray3df ray{ Vector3df{0.0f, 10.0f, 0.0f}, Vector3df{0.0f, -1.0f, 0.0f} };
    Intersection_Context<float, 3> ctx_badouel, ctx_mt;
    int mat_idx_badouel = -1, mat_idx_mt = -1;
    bool hit_badouel = scene.intersect(ray, ctx_badouel, mat_idx_badouel);
    bool hit_mt      = scene.intersect_mt(ray, ctx_mt, mat_idx_mt);

    // lambda to print data after a hit was detected
    auto print_hit = [&](const std::string& name, bool hit,
                         const Intersection_Context<float,3>& ctx, int mat_idx) {
        if (hit) {
            const Material& m = scene.getMaterial(mat_idx);
            std::cout << "[" << name << "] Hit!" << std::endl;
            std::cout << "  t            = " << ctx.t << std::endl;
            std::cout << "  intersection = ("
                      << ctx.intersection[0] << ", "
                      << ctx.intersection[1] << ", "
                      << ctx.intersection[2] << ")" << std::endl;
            std::cout << "  normal       = ("
                      << ctx.normal[0] << ", "
                      << ctx.normal[1] << ", "
                      << ctx.normal[2] << ")" << std::endl;
            std::cout << "  triangle idx = " << mat_idx << std::endl;
            std::cout << "  color        = ("
                      << m.color[0] << ", " << m.color[1] << ", " << m.color[2] << ")"
                      << std::endl;
        } else {
            std::cout << "[" << name << "] No hit." << std::endl;
        }
    };

    // print for badouel and mt
    print_hit("Badouel", hit_badouel, ctx_badouel, mat_idx_badouel);
    std::cout << std::endl;
    print_hit("Moeller-Trumbore", hit_mt, ctx_mt, mat_idx_mt);

    // Check if both methods have the same solution
    if (hit_badouel && hit_mt) {
        float dt = std::abs(ctx_badouel.t - ctx_mt.t);
        std::cout << "\nDeviation t: " << dt
                  << (dt < 1e-3f ? " -> OK" : " -> MISMATCH!") << std::endl;
    }

    // raytracer using mt
    const int img_width  = 200;
    const int img_height = 150;
    const float aspect   = static_cast<float>(img_width) / img_height;
    // camera setup
    Vector3df cam_pos    = Vector3df{ 0.0f, 5.0f, -12.0f};
    Vector3df cam_target = Vector3df{ 0.0f, 1.0f,  -1.0f};
    Vector3df cam_up     = Vector3df{ 0.0f, 1.0f,   0.0f};
    Camera camera(cam_pos, cam_target, cam_up, 45.0f, aspect);
    // render
    Raytracer raytracer(img_width, img_height, 5);
    std::cout << "\nStarting render..." << std::endl;
    auto t_start = std::chrono::high_resolution_clock::now();
    raytracer.render(camera, scene);
    auto t_end = std::chrono::high_resolution_clock::now();
    double seconds = std::chrono::duration<double>(t_end - t_start).count();
    int min = static_cast<int>(seconds) / 60;
    int sec = static_cast<int>(seconds) % 60;
    int ms  = static_cast<int>((seconds - static_cast<int>(seconds)) * 1000);
    std::cout << "Render time: " << min << " min " << sec << " sec " << ms << " ms" << std::endl;
    ImageWriter writer;
    writer.write_ppm("raytracer_output.ppm", raytracer.getFramebuffer(), img_width, img_height);
 
    return 0;
}