#include "io/ObjLoader.h"
#include "io/ImageWriter.h"
#include "scene/Scene.h"
#include "core/Camera.h"
#include "scene/Light.h"
#include "core/Raytracer.h"
#include <iostream>
#include <chrono>

#ifndef DATA_PATH
#define DATA_PATH "data/"
#endif
 
int main() {
    Scene scene;
    ObjLoader loader;
 
    // -----------------------------------------------------------------------
    // Datei laden
    // -----------------------------------------------------------------------
    loader.load(std::string(DATA_PATH) + "teapot_n_glass.obj", scene);

    // -----------------------------------------------------------------------
    // Lichtquelle
    // -----------------------------------------------------------------------
    Light light(
        Vector3df{  0.0f, 15.0f, -8.0f },  // position: hoch und zentriert
        Vector3df{  1.0f,  1.0f,  1.0f }   // intensity: weißes Licht
    );
    scene.addLight(light);

    // -----------------------------------------------------------------------
    // Schnittpunkttest: Strahl von oben nach unten, sollte die Szene treffen
    // -----------------------------------------------------------------------
    Ray3df ray{ Vector3df{0.0f, 10.0f, 0.0f}, Vector3df{0.0f, -1.0f, 0.0f} };
    Intersection_Context<float, 3> ctx;
    int mat_idx = -1;
 
    if (scene.intersect(ray, ctx, mat_idx)) {
        const Material& m = scene.getMaterial(mat_idx);
        std::cout << "Treffer!" << std::endl;
        std::cout << "  t              = " << ctx.t << std::endl;
        std::cout << "  Schnittpunkt   = ("
                  << ctx.intersection[0] << ", "
                  << ctx.intersection[1] << ", "
                  << ctx.intersection[2] << ")" << std::endl;
        std::cout << "  Normale        = ("
                  << ctx.normal[0] << ", "
                  << ctx.normal[1] << ", "
                  << ctx.normal[2] << ")" << std::endl;
        std::cout << "  Dreieck-Index  = " << mat_idx << std::endl;
        std::cout << "  Farbe          = ("
                  << m.color[0] << ", "
                  << m.color[1] << ", "
                  << m.color[2] << ")" << std::endl;
        std::cout << "  Reflektivitaet = " << m.reflectivity << std::endl;
        std::cout << "  Shininess      = " << m.shininess << std::endl;
        std::cout << "  Brechungsindex = " << m.refraction_index << std::endl;
        std::cout << "  Transparenz    = " << m.transparency << std::endl;
    } else {
        std::cout << "Kein Treffer fuer Strahl von oben." << std::endl;
    }

    // -----------------------------------------------------------------------
    // Raytracer
    // -----------------------------------------------------------------------
    const int img_width  = 200;
    const int img_height = 150;
    const float aspect   = static_cast<float>(img_width) / img_height;

    Vector3df cam_pos    = Vector3df{ 0.0f, 5.0f, -12.0f};
    Vector3df cam_target = Vector3df{ 0.0f, 1.0f,  -1.0f};
    Vector3df cam_up     = Vector3df{ 0.0f, 1.0f,   0.0f};
    Camera camera(cam_pos, cam_target, cam_up, 45.0f, aspect);

    Raytracer raytracer(img_width, img_height, 5);

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