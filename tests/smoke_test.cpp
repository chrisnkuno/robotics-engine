#include <cassert>
#include <string>

#include "rex/sim/engine.hpp"

int main() {
  rex::sim::Engine engine{};

  rex::dynamics::WorldState world{};
  world.bodies.push_back({
    .id = rex::platform::EntityId{.index = 0, .generation = 1},
  });

  rex::collision::ContactManifold manifold{};
  manifold.body_a = rex::platform::EntityId{.index = 0, .generation = 1};
  manifold.body_b = rex::platform::EntityId{.index = 1, .generation = 1};
  manifold.points[0].penetration = 0.015;
  manifold.point_count = 1;
  world.contact_manifolds.push_back(manifold);

  const rex::sim::StepTrace trace = engine.step(world);

  assert(trace.body_count == 1);
  assert(trace.solver.contact_count == 1);
  assert(trace.solver.constraint_count == 3);
  assert(trace.solver.max_penetration > 0.0);
  assert(trace.pipeline_summary.find("sequential-impulse-pgs") != std::string::npos);
  return 0;
}
