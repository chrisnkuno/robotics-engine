#include "rex/viewer/demo.hpp"

#include "rex/geometry/shapes.hpp"

namespace rex::viewer {

namespace {

auto make_sphere_body(
  std::uint32_t index,
  const rex::math::Vec3& translation,
  double radius,
  double inverse_mass = 1.0) -> rex::dynamics::BodyState {
  return {
    .id = rex::platform::EntityId{.index = index, .generation = 1},
    .pose = rex::math::Transform{.translation = translation},
    .inverse_mass = inverse_mass,
    .shape = rex::geometry::Shape{.data = rex::geometry::Sphere{.radius = radius}},
  };
}

auto make_box_body(
  std::uint32_t index,
  const rex::math::Vec3& translation,
  const rex::math::Vec3& half_extents,
  double inverse_mass = 0.0) -> rex::dynamics::BodyState {
  return {
    .id = rex::platform::EntityId{.index = index, .generation = 1},
    .pose = rex::math::Transform{.translation = translation},
    .inverse_mass = inverse_mass,
    .shape = rex::geometry::Shape{.data = rex::geometry::Box{.half_extents = half_extents}},
  };
}

}  // namespace

auto build_demo_replay(std::size_t frame_count) -> ReplayLog {
  rex::sim::EngineConfig config{};
  config.simulation.gravity = {0.0, 0.0, -9.81};

  rex::sim::Engine engine{config};
  rex::dynamics::WorldState world{};
  const std::size_t ground_index =
    world.bodies.add_body(make_box_body(100, {0.0, 0.0, 0.0}, {0.7, 0.5, 0.5}));
  const std::size_t contact_sphere_index =
    world.bodies.add_body(make_sphere_body(101, {1.15, 0.0, 0.0}, 0.75));
  const std::size_t falling_sphere_index =
    world.bodies.add_body(make_sphere_body(102, {-0.6, 0.0, 2.4}, 0.35));
  (void)ground_index;
  (void)contact_sphere_index;
  (void)falling_sphere_index;

  ReplayLog replay{};
  double sim_time = 0.0;
  for (std::size_t frame_index = 0; frame_index < frame_count; ++frame_index) {
    const rex::sim::StepTrace trace = engine.step(world);
    replay.add_frame(capture_frame(world, trace, frame_index, sim_time));
    sim_time += config.simulation.step.dt;
  }

  return replay;
}

}  // namespace rex::viewer
