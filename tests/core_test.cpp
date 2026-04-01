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

}  // namespace

int main() {
  test_body_storage_round_trip();
  test_broadphase_is_stable_and_sorted();
  test_persistent_manifold_keeps_cached_impulse();
  test_nonpersistent_manifold_drops_cached_impulse();
  test_engine_step_integrates_gravity();
  test_solver_assembles_contact_rows();
  test_static_body_does_not_integrate();
  test_engine_step_builds_contacts_and_solver_counts();
  std::cout << "all core tests passed\n";
  return 0;
}
