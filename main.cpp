#include "io/ObjLoader.h"
#include "io/ImageWriter.h"
#include "scene/Scene.h"
#include <iostream>

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
    // ImageWriter-Test: einfacher RGB-Farbverlauf
    // -----------------------------------------------------------------------
    const int width  = 256;
    const int height = 256;

    std::vector<Vector3df> framebuffer(width * height, Vector3df{0.0f, 0.0f, 0.0f});

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            framebuffer[y * width + x] = Vector3df{
                static_cast<float>(x) / (width  - 1),  // R: links->rechts
                static_cast<float>(y) / (height - 1),  // G: oben->unten
                0.25f                                   // B: konstant
            };
        }
    }

    ImageWriter writer;
    writer.write_ppm("test_output.ppm", framebuffer, width, height);
 
    return 0;
}