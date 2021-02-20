//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include <cmath>
#include "Scene.hpp"

void Scene::buildBVH() {
  printf(" - Generating BVH...\n\n");
  this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const {
  return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const {
  float emit_area_sum = 0;
  for (uint32_t k = 0; k < objects.size(); ++k) {
    if (objects[k]->hasEmit()) {
      emit_area_sum += objects[k]->getArea();
    }
  }
  float p = get_random_float() * emit_area_sum;
  emit_area_sum = 0;
  for (uint32_t k = 0; k < objects.size(); ++k) {
    if (objects[k]->hasEmit()) {
      emit_area_sum += objects[k]->getArea();
      if (p <= emit_area_sum) {
        objects[k]->Sample(pos, pdf);
        break;
      }
    }
  }
}

bool Scene::trace(
    const Ray &ray,
    const std::vector<Object *> &objects,
    float &tNear, uint32_t &index, Object **hitObject) {
  *hitObject = nullptr;
  for (uint32_t k = 0; k < objects.size(); ++k) {
    float tNearK = kInfinity;
    uint32_t indexK;
    Vector2f uvK;
    if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
      *hitObject = objects[k];
      tNear = tNearK;
      index = indexK;
    }
  }

  return (*hitObject != nullptr);
}

Vector3f Scene::castRay(const Ray &ray, int depth) const {
  if (depth > maxDepth)
    return this->backgroundColor;
  // return Vector3f{0., 0., 0.};

  // Consider original ray and the intersection
  Intersection o_inter = intersect(ray);
  if (!o_inter.happened)
    // return this->backgroundColor;
    return Vector3f{0., 0., 0.};

  if (o_inter.obj->hasEmit())
    return Vector3f{1., 1., 1.};

  // Then shade the intersection(o_inter.coords)

  // Direct illumination and indirect...
  Vector3f L_dir, L_indir;
  L_dir = L_indir = Vector3f{0, 0, 0};

  // Intersection with the light
  Intersection inter;
  float pdf_light = 0;
  // Sampling the Light
  sampleLight(inter, pdf_light);

  auto &N = o_inter.normal
  , &NN = inter.normal
  , &p = o_inter.coords
  , &p_p = inter.coords
  , wo = normalize(-ray.direction) // if its opposite
  , ws = normalize(p_p - p);

  Ray r(p, ws);
  auto _inter = intersect(r);
  if (_inter.happened && _inter.obj->hasEmit()) {
    L_dir = inter.emit
        * o_inter.m->eval(wo, ws, N)
        * dotProduct(ws, N)
        * dotProduct(-ws, NN)
        / (std::pow((p_p - p).norm(), 2) // inter.distance(?)
            * pdf_light);
  }

  if (get_random_float() < RussianRoulette) {
    auto wi = o_inter.m->sample(wo, N);
    Ray r_p(p, wi);
    auto inter_p = intersect(r_p);
    if (inter_p.happened && !inter_p.obj->hasEmit()) {
      L_indir = castRay(r_p, depth + 1)
          * o_inter.m->eval(wo, wi, N)
          * dotProduct(wi, N)
          / (o_inter.m->pdf(wo, wi, N)
              * RussianRoulette);
    }
  }

  return L_dir + L_indir;
}