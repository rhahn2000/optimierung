#include "Material.h"

Material::Material(Vector3df color, float reflectivity, float shininess, float refraction_index, float transparency)
    : color(color), reflectivity(reflectivity), shininess(shininess),
      refraction_index(refraction_index), transparency(transparency)
{
}
