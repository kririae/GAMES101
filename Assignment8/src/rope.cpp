#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

namespace CGL {

Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes) {
  // num_node \ge 2
  if (num_nodes < 2) {
    masses.push_back(new Mass(start, node_mass, false));
    masses[pinned_nodes[0]]->pinned = true;
    return;
  }

  Vector2D dist = (end - start) / (num_nodes - 1);
  for (int i = 0; i < num_nodes; ++i)
    masses.push_back(new Mass(start + i * dist, node_mass, false));

  for (auto i = masses.begin() + 1; i != masses.end(); ++i)
    springs.push_back(new Spring(*(i - 1), *i, k));

  for (auto &i : pinned_nodes) {
    masses[i]->pinned = true;
  }
}

void Rope::simulateEuler(float delta_t, Vector2D gravity) {
  for (auto &s : springs) {
    auto &a = s->m1->position
    , &b = s->m2->position;
    double actual_len = (b - a).norm();

    // Inner force
    Vector2D f = s->k * (b - a) * (actual_len - s->rest_length) / actual_len;
    s->m1->forces += f;
    s->m2->forces -= f;
  }

  for (auto &m : masses) {
    if (!m->pinned) {
      // Apply gravity
      m->forces += m->mass * gravity;

      // Global damping
      m->forces += -0.01 * m->velocity;

      Vector2D a = m->forces / m->mass;
      m->velocity += a * delta_t;
      m->position += m->velocity * delta_t;
    }

    // Reset all forces on each mass
    m->forces = Vector2D(0, 0);
  }
}

void Rope::simulateVerlet(float delta_t, Vector2D gravity) {
  static double damping_factor = 1e-4;

  for (auto &s : springs) {
    auto &a = s->m1->position
    , &b = s->m2->position;
    double actual_len = (b - a).norm();

    // Inner force
    Vector2D f = s->k * (b - a) * (actual_len - s->rest_length) / actual_len;
    s->m1->forces += f;
    s->m2->forces -= f;
  }

  for (auto &m : masses) {
    if (!m->pinned) {
      Vector2D temp_position = m->position;

      m->forces += gravity;
      Vector2D a = m->forces / m->mass;
      m->position += (1 - damping_factor) * (m->position - m->last_position) + a * delta_t * delta_t;
      m->last_position = temp_position;
    }
    m->forces = Vector2D(0, 0);
  }
}
}
