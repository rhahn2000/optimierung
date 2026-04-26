#include "io/ObjLoader.h"
#include "io/ImageWriter.h"
#include "scene/Scene.h"
#include "core/Camera.h"
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
    // Camera test: render scene with flat material color, no shading
    // -----------------------------------------------------------------------
    const int img_width  = 200;
    const int img_height = 150;
    const float aspect   = static_cast<float>(img_width) / img_height;

    Vector3df cam_pos    = Vector3df{0.0f, 5.0f, -12.0f};
    Vector3df cam_target = Vector3df{0.0f, 1.0f,  -1.0f};
    Vector3df cam_up     = Vector3df{0.0f, 1.0f,  0.0f};
    Camera camera(cam_pos, cam_target, cam_up, 45.0f, aspect);

    std::vector<Vector3df> render_buffer(img_width * img_height);

    for (int y = 0; y < img_height; y++) {
        for (int x = 0; x < img_width; x++) {
            float u = static_cast<float>(x) / (img_width  - 1);
            float v = static_cast<float>(img_height - 1 - y) / (img_height - 1); // y-Achse umkehren

            Ray3df r = camera.get_ray(u, v);
            Intersection_Context<float, 3> c;
            int mi = -1;

            if (scene.intersect(r, c, mi)) {
                render_buffer[y * img_width + x] = scene.getMaterial(mi).color;
            } else {
                render_buffer[y * img_width + x] = Vector3df{0.2f, 0.2f, 0.2f}; // background
            }
        }
    }

    ImageWriter render_writer;
    render_writer.write_ppm("camera_test.ppm", render_buffer, img_width, img_height);

    // -----------------------------------------------------------------------
    // ImageWriter-Test: einfacher RGB-Farbverlauf
    // -----------------------------------------------------------------------
    const int width  = 256;
    const int height = 256;

    std::vector<Vector3df> framebuffer(width * height);

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