#include "rex/sim/engine.hpp"

#include <algorithm>
#include <numeric>

namespace rex::sim {

Engine::Engine(EngineConfig config) : config_(config) {}

auto Engine::config() const noexcept -> const EngineConfig& {
  return config_;
}

auto Engine::step(rex::dynamics::WorldState& world) const -> StepTrace {
  rex::dynamics::integrate_unconstrained(world.bodies, config_.simulation);

  const auto collision_proxies = rex::dynamics::build_collision_proxies(world.bodies);
  const rex::collision::CollisionFrame collision_frame = rex::collision::build_frame(
    collision_proxies,
    world.contact_manifolds,
    config_.simulation.collision);
  world.contact_manifolds = collision_frame.manifolds;

  const std::size_t body_count = world.bodies.size();
  const std::size_t articulation_count = world.articulations.size();
  const std::size_t contact_count = std::transform_reduce(
    world.contact_manifolds.begin(),
    world.contact_manifolds.end(),
    std::size_t{0},
    std::plus<>{},
    [](const rex::collision::ContactManifold& manifold) { return manifold.point_count; });
  const rex::solver::ConstraintAssembly constraint_assembly =
    rex::solver::assemble_contact_rows(world.contact_manifolds);

  rex::solver::SolverResult solver_result{};
  solver_result.contact_count = contact_count;
  solver_result.constraint_count = constraint_assembly.rows.size();

  for (const auto& manifold : world.contact_manifolds) {
    for (std::size_t point_index = 0; point_index < manifold.point_count; ++point_index) {
      solver_result.max_penetration =
        std::max(solver_result.max_penetration, manifold.points[point_index].penetration);
    }
  }

  StepTrace trace{};
  trace.body_count = body_count;
  trace.articulation_count = articulation_count;
  trace.broadphase_pair_count = collision_frame.broadphase_pairs.size();
  trace.manifold_count = world.contact_manifolds.size();
  trace.solver = solver_result;
  trace.pipeline_summary =
    "semi-implicit-euler -> articulated-body-dynamics -> broadphase -> manifolds -> " +
    rex::solver::describe(config_.simulation.solver);

  return trace;
}

}  // namespace rex::sim
