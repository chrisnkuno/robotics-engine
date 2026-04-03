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
  const rex::math::Quat& rotation = {},
  double inverse_mass = 0.0) -> rex::dynamics::BodyState {
  return {
    .id = rex::platform::EntityId{.index = index, .generation = 1},
    .pose = rex::math::Transform{.rotation = rotation, .translation = translation},
    .inverse_mass = inverse_mass,
    .shape = rex::geometry::Shape{.data = rex::geometry::Box{.half_extents = half_extents}},
  };
}

}  // namespace

DemoSceneRunner::DemoSceneRunner() : engine_(config_) {
  config_.simulation.gravity = {0.0, 0.0, -9.81};
  engine_ = rex::sim::Engine{config_};

  const std::size_t ground_index =
    world_.bodies.add_body(make_box_body(
      100,
      {0.0, 0.0, 0.0},
      {0.7, 0.5, 0.5},
      rex::math::quat_from_axis_angle({0.0, 1.0, 0.0}, 0.45)));
  const std::size_t contact_sphere_index =
    world_.bodies.add_body(make_sphere_body(101, {1.15, 0.0, 0.0}, 0.75));
  const std::size_t falling_sphere_index =
    world_.bodies.add_body(make_sphere_body(102, {-0.8, -0.55, 2.45}, 0.35));
  const std::size_t side_column_index =
    world_.bodies.add_body(make_box_body(
      103,
      {-1.4, 1.05, 0.8},
      {0.22, 0.22, 0.8},
      rex::math::quat_from_axis_angle({0.0, 0.0, 1.0}, 0.2)));
  const std::size_t floating_sphere_index =
    world_.bodies.add_body(make_sphere_body(104, {0.2, 1.1, 1.65}, 0.24));
  (void)ground_index;
  (void)contact_sphere_index;
  (void)falling_sphere_index;
  (void)side_column_index;
  (void)floating_sphere_index;
}

auto DemoSceneRunner::step_frame() -> FrameSnapshot {
  const rex::sim::StepTrace trace = engine_.step(world_);
  FrameSnapshot frame = capture_frame(world_, trace, next_frame_index_, sim_time_);
  ++next_frame_index_;
  sim_time_ += config_.simulation.step.dt;
  return frame;
}

auto build_demo_replay(std::size_t frame_count) -> ReplayLog {
  DemoSceneRunner runner{};
  ReplayLog replay{};
  for (std::size_t frame_index = 0; frame_index < frame_count; ++frame_index) {
    replay.add_frame(runner.step_frame());
  }

  return replay;
}

}  // namespace rex::viewer
