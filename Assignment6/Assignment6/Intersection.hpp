//
// Created by LEI XU on 5/16/19.
//

#ifndef RAYTRACING_INTERSECTION_H
#define RAYTRACING_INTERSECTION_H
#include "Vector.hpp"
#include "Material.hpp"
class Object;
class Sphere;

struct Intersection {
  Intersection() {
    happened = false;
    coords = Vector3f();
    normal = Vector3f();
    distance = std::numeric_limits<double>::max();
    obj = nullptr;
    m = nullptr;
  }
  bool happened;

  // Intersect position
  Vector3f coords;

  // Interpolation of the vertex normals
  Vector3f normal;

  // eye_pos -> this->coords
  double distance;
  Object *obj;
  Material *m;
};
#endif //RAYTRACING_INTERSECTION_H
