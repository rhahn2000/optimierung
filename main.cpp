#include "io/ObjLoader.h"
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
 
    return 0;
}
 