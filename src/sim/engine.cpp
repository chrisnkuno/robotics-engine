#include "rex/sim/engine.hpp"

#include "rex/platform/profile.hpp"

namespace rex::sim {

Engine::Engine(EngineConfig config) : config_(config) {}

auto Engine::config() const noexcept -> const EngineConfig& {
  return config_;
}

auto Engine::step(rex::dynamics::WorldState& world) const -> StepTrace {
  StepTrace trace{};
  {
    REX_PROFILE_SCOPE("sim::step");
    rex::platform::ScopedMilliseconds total_timer(trace.profile.total_ms);

    {
      REX_PROFILE_SCOPE("sim::integrate");
      rex::platform::ScopedMilliseconds integrate_timer(trace.profile.integrate_ms);
      rex::dynamics::integrate_unconstrained(world.bodies, config_.simulation);
    }

    const auto collision_proxies = rex::dynamics::build_collision_proxies(world.bodies);
    rex::collision::CollisionFrame collision_frame{};
    {
      REX_PROFILE_SCOPE("sim::collision");
      rex::platform::ScopedMilliseconds collision_timer(trace.profile.collision_ms);
      collision_frame = rex::collision::build_frame(
        collision_proxies,
        world.contact_manifolds,
        config_.simulation.collision);
    }
    world.contact_manifolds = collision_frame.manifolds;

    const std::size_t body_count = world.bodies.size();
    const std::size_t articulation_count = world.articulations.size();
    {
      REX_PROFILE_SCOPE("sim::solver");
      rex::platform::ScopedMilliseconds solver_timer(trace.profile.solver_ms);
      trace.solver = rex::solver::solve_contacts(
        world.bodies,
        world.contact_manifolds,
        config_.simulation.solver,
        config_.simulation.step.dt);
    }

    trace.body_count = body_count;
    trace.articulation_count = articulation_count;
    trace.broadphase_pair_count = collision_frame.broadphase_pairs.size();
    trace.manifold_count = world.contact_manifolds.size();
  }

  trace.pipeline_summary =
    "semi-implicit-euler -> articulated-body-dynamics -> broadphase -> manifolds -> " +
    rex::solver::describe(config_.simulation.solver);
  return trace;
}

}  // namespace rex::sim
