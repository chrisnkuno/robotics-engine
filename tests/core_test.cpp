#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include "rex/collision/contact.hpp"
#include "rex/dynamics/world.hpp"
#include "rex/geometry/shapes.hpp"
#include "rex/solver/solver.hpp"
#include "rex/sim/engine.hpp"

namespace {

void expect(bool condition, const std::string& message) {
  if (!condition) {
    std::cerr << "test failure: " << message << '\n';
    std::exit(1);
  }
}

auto nearly_equal(double lhs, double rhs, double tolerance = 1.0e-9) -> bool {
  return std::abs(lhs - rhs) <= tolerance;
}

void expect_vec3_nearly_equal(
  const rex::math::Vec3& actual,
  const rex::math::Vec3& expected,
  double tolerance,
  const std::string& message) {
  expect(nearly_equal(actual.x, expected.x, tolerance), message + " (x)");
  expect(nearly_equal(actual.y, expected.y, tolerance), message + " (y)");
  expect(nearly_equal(actual.z, expected.z, tolerance), message + " (z)");
}

auto make_sphere_body(
  std::uint32_t index,
  const rex::math::Vec3& translation,
  double radius = 0.5,
  double inverse_mass = 1.0) -> rex::dynamics::BodyState {
  return {
    .id = rex::platform::EntityId{.index = index, .generation = 1},
    .pose = rex::math::Transform{
      .translation = translation,
    },
    .inverse_mass = inverse_mass,
    .shape = rex::geometry::Shape{
      .data = rex::geometry::Sphere{.radius = radius},
    },
  };
}

auto make_box_body(
  std::uint32_t index,
  const rex::math::Vec3& translation,
  const rex::math::Vec3& half_extents,
  double inverse_mass = 1.0) -> rex::dynamics::BodyState {
  return {
    .id = rex::platform::EntityId{.index = index, .generation = 1},
    .pose = rex::math::Transform{
      .translation = translation,
    },
    .inverse_mass = inverse_mass,
    .shape = rex::geometry::Shape{
      .data = rex::geometry::Box{.half_extents = half_extents},
    },
  };
}

void test_body_storage_round_trip() {
  rex::dynamics::BodyStorage bodies{};
  bodies.reserve(2);

  const std::size_t body_index =
    bodies.add_body(make_sphere_body(3, rex::math::Vec3{1.0, -2.0, 4.5}, 0.75, 0.25));

  expect(body_index == 0, "first inserted body should use index 0");
  expect(bodies.size() == 1, "body storage should report one body");

  const rex::dynamics::BodyState restored = bodies.state(0);
  expect(restored.id.index == 3, "restored id should match inserted id");
  expect(nearly_equal(restored.pose.translation.x, 1.0), "x translation should round-trip");
  expect(nearly_equal(restored.pose.translation.y, -2.0), "y translation should round-trip");
  expect(nearly_equal(restored.pose.translation.z, 4.5), "z translation should round-trip");
  expect(nearly_equal(restored.inverse_mass, 0.25), "inverse mass should round-trip");
}

void test_broadphase_is_stable_and_sorted() {
  const std::vector<rex::collision::BodyProxy> bodies = {
    {.id = rex::platform::EntityId{.index = 7, .generation = 1},
     .pose = rex::math::Transform{.translation = {0.0, 0.0, 0.0}},
     .shape = rex::geometry::Shape{.data = rex::geometry::Sphere{.radius = 0.6}}},
    {.id = rex::platform::EntityId{.index = 2, .generation = 1},
     .pose = rex::math::Transform{.translation = {0.9, 0.0, 0.0}},
     .shape = rex::geometry::Shape{.data = rex::geometry::Sphere{.radius = 0.6}}},
    {.id = rex::platform::EntityId{.index = 11, .generation = 1},
     .pose = rex::math::Transform{.translation = {4.0, 0.0, 0.0}},
     .shape = rex::geometry::Shape{.data = rex::geometry::Sphere{.radius = 0.6}}},
  };

  const rex::collision::CollisionFrame frame =
    rex::collision::build_frame(bodies, {}, rex::collision::CollisionPipelineConfig{});

  expect(frame.broadphase_pairs.size() == 1, "only one pair should overlap in broadphase");
  expect(frame.broadphase_pairs[0].body_a.index == 2, "lower entity id should be first");
  expect(frame.broadphase_pairs[0].body_b.index == 7, "higher entity id should be second");
}

void test_broadphase_reports_multiple_pairs_in_sorted_order() {
  const std::vector<rex::collision::BodyProxy> bodies = {
    {.id = rex::platform::EntityId{.index = 10, .generation = 1},
     .pose = rex::math::Transform{.translation = {0.0, 0.0, 0.0}},
     .shape = rex::geometry::Shape{.data = rex::geometry::Sphere{.radius = 1.0}}},
    {.id = rex::platform::EntityId{.index = 5, .generation = 1},
     .pose = rex::math::Transform{.translation = {1.0, 0.0, 0.0}},
     .shape = rex::geometry::Shape{.data = rex::geometry::Sphere{.radius = 1.0}}},
    {.id = rex::platform::EntityId{.index = 8, .generation = 1},
     .pose = rex::math::Transform{.translation = {1.9, 0.0, 0.0}},
     .shape = rex::geometry::Shape{.data = rex::geometry::Sphere{.radius = 1.0}}},
  };

  const rex::collision::CollisionFrame frame =
    rex::collision::build_frame(bodies, {}, rex::collision::CollisionPipelineConfig{});

  expect(frame.broadphase_pairs.size() == 3, "three mutually overlapping spheres should produce three pairs");
  expect(frame.broadphase_pairs[0].body_a.index == 5, "first pair should start with the lowest id");
  expect(frame.broadphase_pairs[0].body_b.index == 8, "first pair should be sorted by id");
  expect(frame.broadphase_pairs[1].body_a.index == 5, "second pair should preserve sorted order");
  expect(frame.broadphase_pairs[1].body_b.index == 10, "second pair should preserve sorted order");
  expect(frame.broadphase_pairs[2].body_a.index == 8, "third pair should preserve sorted order");
  expect(frame.broadphase_pairs[2].body_b.index == 10, "third pair should preserve sorted order");
}

void test_broadphase_is_input_order_invariant() {
  const std::vector<rex::collision::BodyProxy> first_order = {
    {.id = rex::platform::EntityId{.index = 10, .generation = 1},
     .pose = rex::math::Transform{.translation = {0.0, 0.0, 0.0}},
     .shape = rex::geometry::Shape{.data = rex::geometry::Sphere{.radius = 1.0}}},
    {.id = rex::platform::EntityId{.index = 5, .generation = 1},
     .pose = rex::math::Transform{.translation = {1.0, 0.0, 0.0}},
     .shape = rex::geometry::Shape{.data = rex::geometry::Sphere{.radius = 1.0}}},
    {.id = rex::platform::EntityId{.index = 8, .generation = 1},
     .pose = rex::math::Transform{.translation = {1.9, 0.0, 0.0}},
     .shape = rex::geometry::Shape{.data = rex::geometry::Sphere{.radius = 1.0}}},
  };
  const std::vector<rex::collision::BodyProxy> second_order = {
    {.id = rex::platform::EntityId{.index = 8, .generation = 1},
     .pose = rex::math::Transform{.translation = {1.9, 0.0, 0.0}},
     .shape = rex::geometry::Shape{.data = rex::geometry::Sphere{.radius = 1.0}}},
    {.id = rex::platform::EntityId{.index = 10, .generation = 1},
     .pose = rex::math::Transform{.translation = {0.0, 0.0, 0.0}},
     .shape = rex::geometry::Shape{.data = rex::geometry::Sphere{.radius = 1.0}}},
    {.id = rex::platform::EntityId{.index = 5, .generation = 1},
     .pose = rex::math::Transform{.translation = {1.0, 0.0, 0.0}},
     .shape = rex::geometry::Shape{.data = rex::geometry::Sphere{.radius = 1.0}}},
  };

  const rex::collision::CollisionFrame first_frame =
    rex::collision::build_frame(first_order, {}, rex::collision::CollisionPipelineConfig{});
  const rex::collision::CollisionFrame second_frame =
    rex::collision::build_frame(second_order, {}, rex::collision::CollisionPipelineConfig{});

  expect(
    first_frame.broadphase_pairs.size() == second_frame.broadphase_pairs.size(),
    "pair count should not depend on input order");
  expect(
    first_frame.manifolds.size() == second_frame.manifolds.size(),
    "manifold count should not depend on input order");

  for (std::size_t pair_index = 0; pair_index < first_frame.broadphase_pairs.size(); ++pair_index) {
    expect(
      first_frame.broadphase_pairs[pair_index].body_a == second_frame.broadphase_pairs[pair_index].body_a,
      "pair ordering should be deterministic across input permutations");
    expect(
      first_frame.broadphase_pairs[pair_index].body_b == second_frame.broadphase_pairs[pair_index].body_b,
      "pair ordering should be deterministic across input permutations");
  }

  for (std::size_t manifold_index = 0; manifold_index < first_frame.manifolds.size(); ++manifold_index) {
    expect(
      first_frame.manifolds[manifold_index].body_a == second_frame.manifolds[manifold_index].body_a,
      "manifold ordering should be deterministic across input permutations");
    expect(
      first_frame.manifolds[manifold_index].body_b == second_frame.manifolds[manifold_index].body_b,
      "manifold ordering should be deterministic across input permutations");
  }
}

void test_persistent_manifold_keeps_cached_impulse() {
  const std::vector<rex::collision::BodyProxy> bodies = {
    {.id = rex::platform::EntityId{.index = 4, .generation = 1},
     .pose = rex::math::Transform{.translation = {0.0, 0.0, 0.0}},
     .shape = rex::geometry::Shape{.data = rex::geometry::Sphere{.radius = 0.75}}},
    {.id = rex::platform::EntityId{.index = 9, .generation = 1},
     .pose = rex::math::Transform{.translation = {1.2, 0.0, 0.0}},
     .shape = rex::geometry::Shape{.data = rex::geometry::Sphere{.radius = 0.75}}},
  };

  rex::collision::ContactManifold previous{};
  previous.body_a = rex::platform::EntityId{.index = 4, .generation = 1};
  previous.body_b = rex::platform::EntityId{.index = 9, .generation = 1};
  previous.point_count = 1;
  previous.points[0].cached_normal_impulse = 3.5;

  const rex::collision::CollisionFrame frame = rex::collision::build_frame(
    bodies,
    std::vector<rex::collision::ContactManifold>{previous},
    rex::collision::CollisionPipelineConfig{});

  expect(frame.manifolds.size() == 1, "overlapping spheres should yield one manifold");
  expect(
    nearly_equal(frame.manifolds[0].points[0].cached_normal_impulse, 3.5),
    "persistent manifold should keep cached impulse");
}

void test_nonpersistent_manifold_drops_cached_impulse() {
  const std::vector<rex::collision::BodyProxy> bodies = {
    {.id = rex::platform::EntityId{.index = 4, .generation = 1},
     .pose = rex::math::Transform{.translation = {0.0, 0.0, 0.0}},
     .shape = rex::geometry::Shape{.data = rex::geometry::Sphere{.radius = 0.75}}},
    {.id = rex::platform::EntityId{.index = 9, .generation = 1},
     .pose = rex::math::Transform{.translation = {1.2, 0.0, 0.0}},
     .shape = rex::geometry::Shape{.data = rex::geometry::Sphere{.radius = 0.75}}},
  };

  rex::collision::ContactManifold previous{};
  previous.body_a = rex::platform::EntityId{.index = 4, .generation = 1};
  previous.body_b = rex::platform::EntityId{.index = 9, .generation = 1};
  previous.point_count = 1;
  previous.points[0].cached_normal_impulse = 3.5;

  rex::collision::CollisionPipelineConfig config{};
  config.enable_persistent_manifolds = false;

  const rex::collision::CollisionFrame frame =
    rex::collision::build_frame(bodies, std::vector<rex::collision::ContactManifold>{previous}, config);

  expect(frame.manifolds.size() == 1, "overlapping spheres should still yield one manifold");
  expect(
    nearly_equal(frame.manifolds[0].points[0].cached_normal_impulse, 0.0),
    "disabling persistence should clear cached impulse reuse");
}

void test_sphere_box_contact_generation() {
  const std::vector<rex::collision::BodyProxy> bodies = {
    {.id = rex::platform::EntityId{.index = 4, .generation = 1},
     .pose = rex::math::Transform{.translation = {1.2, 0.0, 0.0}},
     .shape = rex::geometry::Shape{.data = rex::geometry::Sphere{.radius = 0.8}}},
    {.id = rex::platform::EntityId{.index = 9, .generation = 1},
     .pose = rex::math::Transform{.translation = {0.0, 0.0, 0.0}},
     .shape = rex::geometry::Shape{.data = rex::geometry::Box{.half_extents = {0.5, 0.5, 0.5}}}},
  };

  const rex::collision::CollisionFrame frame =
    rex::collision::build_frame(bodies, {}, rex::collision::CollisionPipelineConfig{});

  expect(frame.manifolds.size() == 1, "sphere-box overlap should create one manifold");
  expect(frame.manifolds[0].body_a.index == 4, "sphere-box manifold should preserve sorted pair order");
  expect(frame.manifolds[0].body_b.index == 9, "sphere-box manifold should preserve sorted pair order");
  expect(frame.manifolds[0].points[0].normal.x < 0.0, "sphere-to-box normal should point toward the box");
  expect(nearly_equal(frame.manifolds[0].points[0].penetration, 0.1), "sphere-box penetration should match geometry");
}

void test_box_box_contact_generation() {
  const std::vector<rex::collision::BodyProxy> bodies = {
    {.id = rex::platform::EntityId{.index = 3, .generation = 1},
     .pose = rex::math::Transform{.translation = {0.0, 0.0, 0.0}},
     .shape = rex::geometry::Shape{.data = rex::geometry::Box{.half_extents = {0.5, 0.5, 0.5}}}},
    {.id = rex::platform::EntityId{.index = 6, .generation = 1},
     .pose = rex::math::Transform{.translation = {0.8, 0.0, 0.0}},
     .shape = rex::geometry::Shape{.data = rex::geometry::Box{.half_extents = {0.5, 0.5, 0.5}}}},
  };

  const rex::collision::CollisionFrame frame =
    rex::collision::build_frame(bodies, {}, rex::collision::CollisionPipelineConfig{});

  expect(frame.manifolds.size() == 1, "box-box overlap should create one manifold");
  expect(frame.manifolds[0].points[0].normal.x > 0.0, "box-box normal should point from the first box to the second");
  expect(nearly_equal(frame.manifolds[0].points[0].penetration, 0.2), "box-box penetration should match overlap");
}

void test_collision_frame_invariants_hold_for_overlaps() {
  const std::vector<rex::collision::BodyProxy> bodies = {
    {.id = rex::platform::EntityId{.index = 1, .generation = 1},
     .pose = rex::math::Transform{.translation = {0.0, 0.0, 0.0}},
     .shape = rex::geometry::Shape{.data = rex::geometry::Sphere{.radius = 1.0}}},
    {.id = rex::platform::EntityId{.index = 2, .generation = 1},
     .pose = rex::math::Transform{.translation = {1.2, 0.0, 0.0}},
     .shape = rex::geometry::Shape{.data = rex::geometry::Sphere{.radius = 1.0}}},
    {.id = rex::platform::EntityId{.index = 3, .generation = 1},
     .pose = rex::math::Transform{.translation = {0.4, 0.0, 0.8}},
     .shape = rex::geometry::Shape{.data = rex::geometry::Box{.half_extents = {0.6, 0.5, 0.6}}}},
  };

  const rex::collision::CollisionFrame frame =
    rex::collision::build_frame(bodies, {}, rex::collision::CollisionPipelineConfig{});

  expect(frame.broadphase_pairs.size() == 3, "all three overlapping shapes should produce unique broadphase pairs");
  for (std::size_t pair_index = 1; pair_index < frame.broadphase_pairs.size(); ++pair_index) {
    const auto& previous = frame.broadphase_pairs[pair_index - 1];
    const auto& current = frame.broadphase_pairs[pair_index];
    expect(previous.body_a < current.body_a || (previous.body_a == current.body_a && previous.body_b < current.body_b),
      "broadphase pairs should stay globally sorted");
  }

  for (const auto& manifold : frame.manifolds) {
    expect(manifold.point_count > 0, "each manifold should contain at least one point");
    for (std::size_t point_index = 0; point_index < manifold.point_count; ++point_index) {
      const auto& point = manifold.points[point_index];
      expect(point.penetration > 0.0, "manifold penetration must remain positive");
      expect(nearly_equal(rex::math::norm(point.normal), 1.0, 1.0e-9), "contact normals should stay unit length");
    }
  }
}

void test_sphere_sphere_contact_matches_exact_geometry() {
  const std::vector<rex::math::Vec3> rhs_positions = {
    {1.5, 0.0, 0.0},
    {1.2, 0.4, 0.3},
    {0.6, 0.2, 1.1},
  };

  for (const auto& rhs_position : rhs_positions) {
    const std::vector<rex::collision::BodyProxy> bodies = {
      {.id = rex::platform::EntityId{.index = 1, .generation = 1},
       .pose = rex::math::Transform{.translation = {0.0, 0.0, 0.0}},
       .shape = rex::geometry::Shape{.data = rex::geometry::Sphere{.radius = 0.9}}},
      {.id = rex::platform::EntityId{.index = 2, .generation = 1},
       .pose = rex::math::Transform{.translation = rhs_position},
       .shape = rex::geometry::Shape{.data = rex::geometry::Sphere{.radius = 0.8}}},
    };
    const rex::collision::CollisionFrame frame =
      rex::collision::build_frame(bodies, {}, rex::collision::CollisionPipelineConfig{});

    expect(frame.manifolds.size() == 1, "sphere-sphere overlap should yield one manifold");
    const auto& point = frame.manifolds[0].points[0];
    const rex::math::Vec3 delta = rhs_position - rex::math::Vec3{0.0, 0.0, 0.0};
    const double distance = rex::math::norm(delta);
    const rex::math::Vec3 expected_normal = rex::math::normalized_or(delta);
    const double expected_penetration = 1.7 - distance;
    const rex::math::Vec3 expected_position = expected_normal * (0.9 - (expected_penetration * 0.5));

    expect_vec3_nearly_equal(point.normal, expected_normal, 1.0e-9, "sphere-sphere normal should match center delta");
    expect(nearly_equal(point.penetration, expected_penetration, 1.0e-9), "sphere-sphere penetration should match exact geometry");
    expect_vec3_nearly_equal(point.position, expected_position, 1.0e-9, "sphere-sphere contact position should lie at the overlap midpoint");
  }
}

void test_sphere_box_contact_matches_surface_projection() {
  const rex::math::Vec3 sphere_center{1.2, 0.2, 0.1};
  const std::vector<rex::collision::BodyProxy> bodies = {
    {.id = rex::platform::EntityId{.index = 1, .generation = 1},
     .pose = rex::math::Transform{.translation = sphere_center},
     .shape = rex::geometry::Shape{.data = rex::geometry::Sphere{.radius = 0.8}}},
    {.id = rex::platform::EntityId{.index = 2, .generation = 1},
     .pose = rex::math::Transform{.translation = {0.0, 0.0, 0.0}},
     .shape = rex::geometry::Shape{.data = rex::geometry::Box{.half_extents = {0.5, 0.5, 0.5}}}},
  };
  const rex::collision::CollisionFrame frame =
    rex::collision::build_frame(bodies, {}, rex::collision::CollisionPipelineConfig{});

  expect(frame.manifolds.size() == 1, "sphere-box overlap should generate a manifold");
  const auto& point = frame.manifolds[0].points[0];
  const rex::math::Vec3 expected_surface_point{0.5, 0.2, 0.1};
  const rex::math::Vec3 expected_box_to_sphere = sphere_center - expected_surface_point;
  const double expected_penetration = 0.8 - rex::math::norm(expected_box_to_sphere);

  expect_vec3_nearly_equal(point.position, expected_surface_point, 1.0e-9, "sphere-box contact should lie on the closest box surface point");
  expect_vec3_nearly_equal(point.normal, {-1.0, 0.0, 0.0}, 1.0e-9, "sphere-box normal should point back toward the box");
  expect(nearly_equal(point.penetration, expected_penetration, 1.0e-9), "sphere-box penetration should equal radius minus distance to the surface");
}

void test_box_box_contact_stays_inside_intersection_volume() {
  const std::vector<rex::collision::BodyProxy> bodies = {
    {.id = rex::platform::EntityId{.index = 1, .generation = 1},
     .pose = rex::math::Transform{.translation = {0.0, 0.0, 0.0}},
     .shape = rex::geometry::Shape{.data = rex::geometry::Box{.half_extents = {0.5, 0.6, 0.4}}}},
    {.id = rex::platform::EntityId{.index = 2, .generation = 1},
     .pose = rex::math::Transform{.translation = {0.8, 0.1, 0.1}},
     .shape = rex::geometry::Shape{.data = rex::geometry::Box{.half_extents = {0.5, 0.5, 0.5}}}},
  };
  const rex::collision::CollisionFrame frame =
    rex::collision::build_frame(bodies, {}, rex::collision::CollisionPipelineConfig{});

  expect(frame.manifolds.size() == 1, "box-box overlap should generate one manifold");
  const auto& point = frame.manifolds[0].points[0];

  expect(nearly_equal(rex::math::norm(point.normal), 1.0, 1.0e-9), "box-box normal should be unit length");
  expect_vec3_nearly_equal(point.normal, {1.0, 0.0, 0.0}, 1.0e-9, "box-box minimum penetration axis should be x");
  expect(point.position.x >= 0.3 && point.position.x <= 0.5, "box-box contact x should stay inside the overlap interval");
  expect(point.position.y >= -0.4 && point.position.y <= 0.5, "box-box contact y should stay inside the overlap interval");
  expect(point.position.z >= -0.3 && point.position.z <= 0.4, "box-box contact z should stay inside the overlap interval");
}

void test_engine_step_integrates_gravity() {
  rex::sim::Engine engine{};
  rex::dynamics::WorldState world{};
  const std::size_t body_index =
    world.bodies.add_body(make_sphere_body(1, rex::math::Vec3{0.0, 0.0, 1.0}, 0.25, 1.0));

  const rex::sim::StepTrace trace = engine.step(world);
  expect(body_index == 0, "first world body should use index 0");

  const rex::dynamics::BodyState state = world.bodies.state(body_index);

  expect(trace.body_count == 1, "trace should report one body");
  expect(trace.manifold_count == 0, "single falling body should not create manifolds");
  expect(state.linear_velocity.z < 0.0, "gravity should change velocity");
  expect(state.pose.translation.z < 1.0, "gravity should advance the body downward");
}

void test_semi_implicit_euler_matches_closed_form_under_constant_gravity() {
  rex::sim::EngineConfig config{};
  config.simulation.gravity = {0.0, 0.0, -10.0};
  config.simulation.step.dt = 0.1;

  rex::sim::Engine engine{config};
  rex::dynamics::WorldState world{};
  const rex::math::Vec3 initial_position{2.0, 0.0, 1.5};
  const rex::math::Vec3 initial_velocity{0.5, 0.0, -1.0};
  const std::size_t body_index = world.bodies.add_body({
    .id = rex::platform::EntityId{.index = 99, .generation = 1},
    .pose = rex::math::Transform{.translation = initial_position},
    .linear_velocity = initial_velocity,
    .inverse_mass = 1.0,
    .shape = rex::geometry::Shape{.data = rex::geometry::Sphere{.radius = 0.25}},
  });

  constexpr std::size_t kStepCount = 5;
  for (std::size_t step_index = 0; step_index < kStepCount; ++step_index) {
    (void)engine.step(world);
  }

  const rex::dynamics::BodyState state = world.bodies.state(body_index);
  const double n = static_cast<double>(kStepCount);
  const double dt = config.simulation.step.dt;
  const rex::math::Vec3 g = config.simulation.gravity;
  const rex::math::Vec3 expected_velocity = initial_velocity + (g * (n * dt));
  const rex::math::Vec3 expected_position =
    initial_position + (initial_velocity * (n * dt)) + (g * (dt * dt * (n * (n + 1.0) * 0.5)));

  expect_vec3_nearly_equal(state.linear_velocity, expected_velocity, 1.0e-9, "semi-implicit velocity should match the closed form");
  expect_vec3_nearly_equal(state.pose.translation, expected_position, 1.0e-9, "semi-implicit position should match the closed form");
}

void test_step_finite_difference_matches_discrete_transition_jacobian() {
  rex::sim::EngineConfig config{};
  config.simulation.gravity = {0.0, 0.0, -10.0};
  config.simulation.step.dt = 0.1;

  auto simulate_one_step = [&](double position_z, double velocity_z) {
    rex::sim::Engine engine{config};
    rex::dynamics::WorldState world{};
    const std::size_t body_index = world.bodies.add_body({
      .id = rex::platform::EntityId{.index = 7, .generation = 1},
      .pose = rex::math::Transform{.translation = {0.0, 0.0, position_z}},
      .linear_velocity = {0.0, 0.0, velocity_z},
      .inverse_mass = 1.0,
      .shape = rex::geometry::Shape{.data = rex::geometry::Sphere{.radius = 0.25}},
    });
    (void)engine.step(world);
    const rex::dynamics::BodyState state = world.bodies.state(body_index);
    return std::pair<double, double>{state.pose.translation.z, state.linear_velocity.z};
  };

  constexpr double kEpsilon = 1.0e-6;
  const auto position_plus = simulate_one_step(1.5 + kEpsilon, -0.7);
  const auto position_minus = simulate_one_step(1.5 - kEpsilon, -0.7);
  const auto velocity_plus = simulate_one_step(1.5, -0.7 + kEpsilon);
  const auto velocity_minus = simulate_one_step(1.5, -0.7 - kEpsilon);

  const double dnext_z_dz = (position_plus.first - position_minus.first) / (2.0 * kEpsilon);
  const double dnext_v_dz = (position_plus.second - position_minus.second) / (2.0 * kEpsilon);
  const double dnext_z_dv = (velocity_plus.first - velocity_minus.first) / (2.0 * kEpsilon);
  const double dnext_v_dv = (velocity_plus.second - velocity_minus.second) / (2.0 * kEpsilon);

  expect(nearly_equal(dnext_z_dz, 1.0, 1.0e-6), "finite-difference d(next_z)/d(z) should equal the discrete transition Jacobian");
  expect(nearly_equal(dnext_v_dz, 0.0, 1.0e-6), "finite-difference d(next_v)/d(z) should stay zero without contacts");
  expect(nearly_equal(dnext_z_dv, config.simulation.step.dt, 1.0e-6), "finite-difference d(next_z)/d(v) should equal dt");
  expect(nearly_equal(dnext_v_dv, 1.0, 1.0e-6), "finite-difference d(next_v)/d(v) should equal one");
}

void test_solver_assembles_contact_rows() {
  rex::collision::ContactManifold manifold{};
  manifold.body_a = rex::platform::EntityId{.index = 3, .generation = 1};
  manifold.body_b = rex::platform::EntityId{.index = 8, .generation = 1};
  manifold.point_count = 1;
  manifold.points[0].normal = {0.0, 0.0, 1.0};

  const rex::solver::ConstraintAssembly assembly =
    rex::solver::assemble_contact_rows(std::vector<rex::collision::ContactManifold>{manifold});

  expect(assembly.rows.size() == 3, "one manifold point should assemble three solver rows");
  expect(assembly.rows[0].axis == rex::solver::ConstraintAxis::kNormal, "first row should be normal");
  expect(
    nearly_equal(rex::math::dot(assembly.rows[0].direction, manifold.points[0].normal), 1.0),
    "normal row should align with the manifold normal");
  expect(
    nearly_equal(rex::math::dot(assembly.rows[0].direction, assembly.rows[1].direction), 0.0),
    "first tangent should be orthogonal to the normal");
  expect(
    nearly_equal(rex::math::dot(assembly.rows[0].direction, assembly.rows[2].direction), 0.0),
    "second tangent should be orthogonal to the normal");
  expect(
    nearly_equal(rex::math::dot(assembly.rows[1].direction, assembly.rows[2].direction), 0.0),
    "tangent rows should be orthogonal to each other");
}

void test_static_body_does_not_integrate() {
  rex::sim::Engine engine{};
  rex::dynamics::WorldState world{};
  const std::size_t body_index =
    world.bodies.add_body(make_sphere_body(5, rex::math::Vec3{0.0, 0.0, 2.0}, 0.25, 0.0));

  const rex::sim::StepTrace trace = engine.step(world);
  const rex::dynamics::BodyState state = world.bodies.state(body_index);

  expect(trace.body_count == 1, "trace should report one static body");
  expect(nearly_equal(state.linear_velocity.z, 0.0), "static body velocity should stay fixed");
  expect(nearly_equal(state.pose.translation.z, 2.0), "static body position should stay fixed");
}

void test_engine_step_builds_contacts_and_solver_counts() {
  rex::sim::EngineConfig config{};
  config.simulation.gravity = {0.0, 0.0, 0.0};

  rex::sim::Engine engine{config};
  rex::dynamics::WorldState world{};
  const std::size_t first_body =
    world.bodies.add_body(make_sphere_body(1, rex::math::Vec3{0.0, 0.0, 0.0}, 0.75, 1.0));
  const std::size_t second_body =
    world.bodies.add_body(make_sphere_body(2, rex::math::Vec3{1.0, 0.0, 0.0}, 0.75, 1.0));

  const rex::sim::StepTrace trace = engine.step(world);

  expect(first_body == 0, "first overlapping body should use index 0");
  expect(second_body == 1, "second overlapping body should use index 1");
  expect(trace.broadphase_pair_count == 1, "overlapping spheres should create one broadphase pair");
  expect(trace.manifold_count == 1, "overlapping spheres should create one manifold");
  expect(trace.solver.contact_count == 1, "one manifold point should count as one contact");
  expect(trace.solver.constraint_count == 3, "one contact should map to one normal and two tangents");
  expect(world.contact_manifolds.size() == 1, "world should store the generated manifold");
  expect(world.contact_manifolds[0].point_count == 1, "sphere-sphere narrowphase should emit one point");
}

void test_engine_step_builds_multiple_contacts() {
  rex::sim::EngineConfig config{};
  config.simulation.gravity = {0.0, 0.0, 0.0};

  rex::sim::Engine engine{config};
  rex::dynamics::WorldState world{};
  const std::size_t first_body =
    world.bodies.add_body(make_sphere_body(10, rex::math::Vec3{0.0, 0.0, 0.0}, 1.0, 1.0));
  const std::size_t second_body =
    world.bodies.add_body(make_sphere_body(5, rex::math::Vec3{1.0, 0.0, 0.0}, 1.0, 1.0));
  const std::size_t third_body =
    world.bodies.add_body(make_sphere_body(8, rex::math::Vec3{1.9, 0.0, 0.0}, 1.0, 1.0));

  const rex::sim::StepTrace trace = engine.step(world);

  expect(first_body == 0, "first body index should be stable");
  expect(second_body == 1, "second body index should be stable");
  expect(third_body == 2, "third body index should be stable");
  expect(trace.broadphase_pair_count == 3, "three overlapping spheres should produce three broadphase pairs");
  expect(trace.manifold_count == 3, "three overlapping spheres should produce three manifolds");
  expect(trace.solver.contact_count == 3, "three manifolds should produce three contacts");
  expect(trace.solver.constraint_count == 9, "three contacts should assemble nine solver rows");
}

void test_engine_step_builds_mixed_shape_contacts() {
  rex::sim::EngineConfig config{};
  config.simulation.gravity = {0.0, 0.0, 0.0};

  rex::sim::Engine engine{config};
  rex::dynamics::WorldState world{};
  const std::size_t box_index =
    world.bodies.add_body(make_box_body(9, rex::math::Vec3{0.0, 0.0, 0.0}, rex::math::Vec3{0.5, 0.5, 0.5}));
  const std::size_t sphere_index =
    world.bodies.add_body(make_sphere_body(4, rex::math::Vec3{1.2, 0.0, 0.0}, 0.8, 1.0));

  const rex::sim::StepTrace trace = engine.step(world);

  expect(box_index == 0, "box body index should be stable");
  expect(sphere_index == 1, "sphere body index should be stable");
  expect(trace.broadphase_pair_count == 1, "mixed overlap should create one broadphase pair");
  expect(trace.manifold_count == 1, "mixed overlap should create one manifold");
  expect(trace.solver.contact_count == 1, "mixed overlap should produce one contact");
  expect(trace.solver.constraint_count == 3, "mixed overlap should assemble three solver rows");
  expect(world.contact_manifolds[0].points[0].penetration > 0.0, "mixed overlap should keep penetration data");
}

}  // namespace

int main() {
  test_body_storage_round_trip();
  test_broadphase_is_stable_and_sorted();
  test_broadphase_reports_multiple_pairs_in_sorted_order();
  test_broadphase_is_input_order_invariant();
  test_persistent_manifold_keeps_cached_impulse();
  test_nonpersistent_manifold_drops_cached_impulse();
  test_sphere_box_contact_generation();
  test_box_box_contact_generation();
  test_collision_frame_invariants_hold_for_overlaps();
  test_sphere_sphere_contact_matches_exact_geometry();
  test_sphere_box_contact_matches_surface_projection();
  test_box_box_contact_stays_inside_intersection_volume();
  test_engine_step_integrates_gravity();
  test_semi_implicit_euler_matches_closed_form_under_constant_gravity();
  test_step_finite_difference_matches_discrete_transition_jacobian();
  test_solver_assembles_contact_rows();
  test_static_body_does_not_integrate();
  test_engine_step_builds_contacts_and_solver_counts();
  test_engine_step_builds_multiple_contacts();
  test_engine_step_builds_mixed_shape_contacts();
  std::cout << "all core tests passed\n";
  return 0;
}
