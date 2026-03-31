#include "rex/sim/engine.hpp"

#include <algorithm>
#include <numeric>

namespace rex::sim {

Engine::Engine(EngineConfig config) : config_(config) {}

auto Engine::config() const noexcept -> const EngineConfig& {
  return config_;
}

auto Engine::step(rex::dynamics::WorldState& world) const -> StepTrace {
  const std::size_t body_count = world.bodies.size();
  const std::size_t articulation_count = world.articulations.size();
  const std::size_t contact_count = std::transform_reduce(
    world.contact_manifolds.begin(),
    world.contact_manifolds.end(),
    std::size_t{0},
    std::plus<>{},
    [](const rex::collision::ContactManifold& manifold) { return manifold.point_count; });

  rex::solver::SolverResult solver_result{};
  solver_result.contact_count = contact_count;
  solver_result.constraint_count = contact_count * 3;

  for (const auto& manifold : world.contact_manifolds) {
    for (std::size_t point_index = 0; point_index < manifold.point_count; ++point_index) {
      solver_result.max_penetration =
        std::max(solver_result.max_penetration, manifold.points[point_index].penetration);
    }
  }

  StepTrace trace{};
  trace.body_count = body_count;
  trace.articulation_count = articulation_count;
  trace.solver = solver_result;
  trace.pipeline_summary =
    "semi-implicit-euler -> articulated-body-dynamics -> manifolds -> " +
    rex::solver::describe(config_.simulation.solver);

  return trace;
}

}  // namespace rex::sim
