#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <numbers>
#include <span>
#include <string>
#include <vector>

#include "rex/collision/contact.hpp"
#include "rex/dynamics/world.hpp"
#include "rex/geometry/shapes.hpp"
#include "rex/kinematics/articulation.hpp"
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

auto broadphase_pair_less(const rex::collision::BroadphasePair& lhs, const rex::collision::BroadphasePair& rhs) -> bool {
  if (lhs.body_a < rhs.body_a) {
    return true;
  }

  if (rhs.body_a < lhs.body_a) {
    return false;
  }

  return lhs.body_b < rhs.body_b;
}

auto make_sorted_pair(rex::platform::EntityId lhs, rex::platform::EntityId rhs) -> rex::collision::BroadphasePair {
  return rhs < lhs
    ? rex::collision::BroadphasePair{.body_a = rhs, .body_b = lhs}
    : rex::collision::BroadphasePair{.body_a = lhs, .body_b = rhs};
}

auto brute_force_broadphase_pairs(std::span<const rex::collision::BodyProxy> bodies)
  -> std::vector<rex::collision::BroadphasePair> {
  std::vector<rex::collision::BroadphasePair> pairs{};
  for (std::size_t lhs_index = 0; lhs_index < bodies.size(); ++lhs_index) {
    for (std::size_t rhs_index = lhs_index + 1; rhs_index < bodies.size(); ++rhs_index) {
      const double radius_sum =
        rex::geometry::bounding_radius(bodies[lhs_index].shape) +
        rex::geometry::bounding_radius(bodies[rhs_index].shape);
      const rex::math::Vec3 delta =
        bodies[rhs_index].pose.translation - bodies[lhs_index].pose.translation;
      if (rex::math::dot(delta, delta) > (radius_sum * radius_sum)) {
        continue;
      }

      pairs.push_back(make_sorted_pair(bodies[lhs_index].id, bodies[rhs_index].id));
    }
  }

  std::sort(pairs.begin(), pairs.end(), broadphase_pair_less);
  return pairs;
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

void expect_quat_equivalent(
  const rex::math::Quat& actual,
  const rex::math::Quat& expected,
  double tolerance,
  const std::string& message) {
  const double alignment =
    std::abs((actual.w * expected.w) + (actual.x * expected.x) + (actual.y * expected.y) + (actual.z * expected.z));
  expect(nearly_equal(alignment, 1.0, tolerance), message);
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

void test_broadphase_matches_bruteforce_with_large_overflow_body() {
  const std::vector<rex::collision::BodyProxy> bodies = {
    {.id = rex::platform::EntityId{.index = 40, .generation = 1},
     .pose = rex::math::Transform{.translation = {0.0, 0.0, -2.0}},
     .shape = rex::geometry::Shape{.data = rex::geometry::Box{.half_extents = {10.0, 10.0, 1.0}}}},
    {.id = rex::platform::EntityId{.index = 5, .generation = 1},
     .pose = rex::math::Transform{.translation = {-0.2, 0.0, 0.2}},
     .shape = rex::geometry::Shape{.data = rex::geometry::Sphere{.radius = 0.7}}},
    {.id = rex::platform::EntityId{.index = 7, .generation = 1},
     .pose = rex::math::Transform{.translation = {0.6, 0.0, 0.2}},
     .shape = rex::geometry::Shape{.data = rex::geometry::Sphere{.radius = 0.7}}},
    {.id = rex::platform::EntityId{.index = 9, .generation = 1},
     .pose = rex::math::Transform{
       .rotation = rex::math::quat_from_axis_angle({0.0, 1.0, 0.0}, 0.4),
       .translation = {2.0, 0.0, 0.1}},
     .shape = rex::geometry::Shape{.data = rex::geometry::Box{.half_extents = {0.6, 0.5, 0.4}}}},
    {.id = rex::platform::EntityId{.index = 11, .generation = 1},
     .pose = rex::math::Transform{.translation = {2.7, 0.0, 0.1}},
     .shape = rex::geometry::Shape{.data = rex::geometry::Sphere{.radius = 0.5}}},
  };

  const rex::collision::CollisionFrame frame =
    rex::collision::build_frame(bodies, {}, rex::collision::CollisionPipelineConfig{});
  const std::vector<rex::collision::BroadphasePair> expected = brute_force_broadphase_pairs(bodies);

  expect(frame.broadphase_pairs.size() == expected.size(), "grid broadphase should match brute-force pair count");
  for (std::size_t pair_index = 0; pair_index < expected.size(); ++pair_index) {
    expect(frame.broadphase_pairs[pair_index].body_a == expected[pair_index].body_a, "grid broadphase should preserve brute-force pair a ids");
    expect(frame.broadphase_pairs[pair_index].body_b == expected[pair_index].body_b, "grid broadphase should preserve brute-force pair b ids");
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
  previous.points[0].cached_tangent_impulse_u = -0.75;
  previous.points[0].cached_tangent_impulse_v = 1.25;

  const rex::collision::CollisionFrame frame = rex::collision::build_frame(
    bodies,
    std::vector<rex::collision::ContactManifold>{previous},
    rex::collision::CollisionPipelineConfig{});

  expect(frame.manifolds.size() == 1, "overlapping spheres should yield one manifold");
  expect(
    nearly_equal(frame.manifolds[0].points[0].cached_normal_impulse, 3.5),
    "persistent manifold should keep cached impulse");
  expect(
    nearly_equal(frame.manifolds[0].points[0].cached_tangent_impulse_u, -0.75),
    "persistent manifold should keep cached tangent impulse u");
  expect(
    nearly_equal(frame.manifolds[0].points[0].cached_tangent_impulse_v, 1.25),
    "persistent manifold should keep cached tangent impulse v");
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
  previous.points[0].cached_tangent_impulse_u = -0.75;
  previous.points[0].cached_tangent_impulse_v = 1.25;

  rex::collision::CollisionPipelineConfig config{};
  config.enable_persistent_manifolds = false;

  const rex::collision::CollisionFrame frame =
    rex::collision::build_frame(bodies, std::vector<rex::collision::ContactManifold>{previous}, config);

  expect(frame.manifolds.size() == 1, "overlapping spheres should still yield one manifold");
  expect(
    nearly_equal(frame.manifolds[0].points[0].cached_normal_impulse, 0.0),
    "disabling persistence should clear cached impulse reuse");
  expect(
    nearly_equal(frame.manifolds[0].points[0].cached_tangent_impulse_u, 0.0),
    "disabling persistence should clear cached tangent impulse u");
  expect(
    nearly_equal(frame.manifolds[0].points[0].cached_tangent_impulse_v, 0.0),
    "disabling persistence should clear cached tangent impulse v");
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

void test_step_profile_reports_nonnegative_phase_timings() {
  rex::sim::Engine engine{};
  rex::dynamics::WorldState world{};
  (void)world.bodies.add_body(make_sphere_body(1, {0.0, 0.0, 1.0}, 0.5, 1.0));

  const rex::sim::StepTrace trace = engine.step(world);

  expect(trace.profile.integrate_ms >= 0.0, "integrate timing should be nonnegative");
  expect(trace.profile.collision_ms >= 0.0, "collision timing should be nonnegative");
  expect(trace.profile.solver_ms >= 0.0, "solver timing should be nonnegative");
  expect(trace.profile.total_ms >= 0.0, "total timing should be nonnegative");
  expect(
    trace.profile.total_ms + 1.0e-9 >=
      trace.profile.integrate_ms + trace.profile.collision_ms + trace.profile.solver_ms,
    "total timing should cover the measured pipeline phases");
}

void test_transform_inverse_and_compose_round_trip() {
  const rex::math::Transform transform{
    .rotation = rex::math::quat_from_axis_angle({0.2, 1.0, -0.3}, 0.7),
    .translation = {1.2, -0.5, 0.7},
  };
  const rex::math::Vec3 point{0.3, -1.1, 2.0};

  const rex::math::Vec3 transformed = rex::math::transform_point(transform, point);
  const rex::math::Vec3 recovered = rex::math::inverse_transform_point(transform, transformed);
  const rex::math::Transform round_trip = rex::math::compose(transform, rex::math::inverse(transform));

  expect_vec3_nearly_equal(recovered, point, 1.0e-9, "inverse transform should recover the original point");
  expect_quat_equivalent(
    round_trip.rotation,
    rex::math::Transform::identity().rotation,
    1.0e-9,
    "transform composed with its inverse should recover identity rotation");
  expect_vec3_nearly_equal(
    round_trip.translation,
    {0.0, 0.0, 0.0},
    1.0e-9,
    "transform composed with its inverse should recover zero translation");
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

void test_angular_integration_matches_closed_form_rotation() {
  rex::sim::EngineConfig config{};
  config.simulation.gravity = {0.0, 0.0, 0.0};
  config.simulation.step.dt = 0.05;

  const rex::math::Quat initial_rotation = rex::math::quat_from_axis_angle({0.0, 1.0, 0.0}, 0.2);
  const rex::math::Vec3 angular_velocity{0.0, 1.5, 0.0};

  rex::sim::Engine engine{config};
  rex::dynamics::WorldState world{};
  const std::size_t body_index = world.bodies.add_body({
    .id = rex::platform::EntityId{.index = 77, .generation = 1},
    .pose = rex::math::Transform{.rotation = initial_rotation, .translation = {0.0, 0.0, 0.0}},
    .angular_velocity = angular_velocity,
    .inverse_mass = 1.0,
    .shape = rex::geometry::Shape{.data = rex::geometry::Box{.half_extents = {0.4, 0.3, 0.2}}},
  });

  constexpr std::size_t kStepCount = 8;
  for (std::size_t step_index = 0; step_index < kStepCount; ++step_index) {
    (void)engine.step(world);
  }

  const rex::dynamics::BodyState state = world.bodies.state(body_index);
  const rex::math::Quat expected_rotation =
    rex::math::integrate_rotation(initial_rotation, angular_velocity, config.simulation.step.dt * static_cast<double>(kStepCount));
  const double quat_norm = std::sqrt(
    (state.pose.rotation.w * state.pose.rotation.w) +
    (state.pose.rotation.x * state.pose.rotation.x) +
    (state.pose.rotation.y * state.pose.rotation.y) +
    (state.pose.rotation.z * state.pose.rotation.z));

  expect(nearly_equal(quat_norm, 1.0, 1.0e-9), "orientation integration should preserve unit quaternions");
  expect_quat_equivalent(
    state.pose.rotation,
    expected_rotation,
    1.0e-9,
    "angular integration should match the closed-form constant-velocity rotation");
}

void test_shape_inverse_inertia_matches_closed_form() {
  const rex::geometry::Shape sphere{.data = rex::geometry::Sphere{.radius = 0.5}};
  const rex::geometry::Shape box{.data = rex::geometry::Box{.half_extents = {1.0, 2.0, 3.0}}};

  const rex::math::Vec3 sphere_inverse_inertia = rex::geometry::inverse_inertia_body(sphere, 2.0);
  const rex::math::Vec3 box_inverse_inertia = rex::geometry::inverse_inertia_body(box, 0.5);

  expect_vec3_nearly_equal(
    sphere_inverse_inertia,
    {20.0, 20.0, 20.0},
    1.0e-9,
    "sphere inverse inertia should match the closed form");
  expect_vec3_nearly_equal(
    box_inverse_inertia,
    {1.5 / 13.0, 0.15, 0.3},
    1.0e-9,
    "box inverse inertia should match the closed form");
}

void test_apply_inverse_inertia_respects_box_rotation() {
  const rex::geometry::Shape box{.data = rex::geometry::Box{.half_extents = {1.0, 2.0, 3.0}}};
  const rex::math::Quat rotation = rex::math::quat_from_axis_angle({0.0, 1.0, 0.0}, 0.5 * std::numbers::pi);
  const rex::math::Vec3 world_x_response =
    rex::geometry::apply_inverse_inertia(box, rotation, 1.0, {1.0, 0.0, 0.0});

  expect(nearly_equal(world_x_response.x, 0.6, 1.0e-9), "rotated box inverse inertia should map world x onto body z");
  expect(nearly_equal(world_x_response.y, 0.0, 1.0e-9), "rotated box inverse inertia should preserve orthogonality in y");
  expect(nearly_equal(world_x_response.z, 0.0, 1.0e-9), "rotated box inverse inertia should not leak into world z");
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

void test_contact_solver_conserves_momentum_and_energy_for_elastic_spheres() {
  rex::sim::EngineConfig config{};
  config.simulation.gravity = {0.0, 0.0, 0.0};
  config.simulation.step.dt = 0.0;
  config.simulation.solver.restitution = 1.0;
  config.simulation.solver.position_correction_factor = 0.0;
  config.simulation.solver.position_iterations = 0;

  rex::sim::Engine engine{config};
  rex::dynamics::WorldState world{};
  const std::size_t first_index = world.bodies.add_body({
    .id = rex::platform::EntityId{.index = 1, .generation = 1},
    .pose = rex::math::Transform{.translation = {0.0, 0.0, 0.0}},
    .linear_velocity = {1.0, 0.0, 0.0},
    .inverse_mass = 1.0,
    .shape = rex::geometry::Shape{.data = rex::geometry::Sphere{.radius = 0.5}},
  });
  const std::size_t second_index = world.bodies.add_body({
    .id = rex::platform::EntityId{.index = 2, .generation = 1},
    .pose = rex::math::Transform{.translation = {0.9, 0.0, 0.0}},
    .linear_velocity = {0.0, 0.0, 0.0},
    .inverse_mass = 1.0,
    .shape = rex::geometry::Shape{.data = rex::geometry::Sphere{.radius = 0.5}},
  });

  const auto momentum_before = world.bodies.linear_velocity(0) + world.bodies.linear_velocity(1);
  const double kinetic_before =
    0.5 * rex::math::dot(world.bodies.linear_velocity(0), world.bodies.linear_velocity(0)) +
    0.5 * rex::math::dot(world.bodies.linear_velocity(1), world.bodies.linear_velocity(1));

  (void)engine.step(world);

  const auto momentum_after = world.bodies.linear_velocity(0) + world.bodies.linear_velocity(1);
  const double kinetic_after =
    0.5 * rex::math::dot(world.bodies.linear_velocity(0), world.bodies.linear_velocity(0)) +
    0.5 * rex::math::dot(world.bodies.linear_velocity(1), world.bodies.linear_velocity(1));

  expect_vec3_nearly_equal(momentum_after, momentum_before, 1.0e-9, "elastic contact should conserve linear momentum");
  expect(nearly_equal(kinetic_after, kinetic_before, 1.0e-9), "elastic contact should conserve kinetic energy");
  expect(first_index == 0, "first elastic test body index should be stable");
  expect(second_index == 1, "second elastic test body index should be stable");
  expect(nearly_equal(world.bodies.linear_velocity(0).x, 0.0, 1.0e-9), "equal-mass elastic spheres should swap velocities");
  expect(nearly_equal(world.bodies.linear_velocity(1).x, 1.0, 1.0e-9), "equal-mass elastic spheres should swap velocities");
}

void test_contact_solver_friction_dissipates_tangential_motion() {
  rex::sim::EngineConfig config{};
  config.simulation.gravity = {0.0, 0.0, 0.0};
  config.simulation.step.dt = 0.0;
  config.simulation.solver.restitution = 0.0;
  config.simulation.solver.friction_coefficient = 0.8;
  config.simulation.solver.position_correction_factor = 0.0;
  config.simulation.solver.position_iterations = 0;

  rex::sim::Engine engine{config};
  rex::dynamics::WorldState world{};
  const std::size_t box_index = world.bodies.add_body(make_box_body(1, {0.0, 0.0, 0.0}, {0.5, 0.5, 0.5}, 0.0));
  const std::size_t sphere_index = world.bodies.add_body({
    .id = rex::platform::EntityId{.index = 2, .generation = 1},
    .pose = rex::math::Transform{.translation = {0.8, 0.0, 0.0}},
    .linear_velocity = {-1.0, 0.0, 0.75},
    .inverse_mass = 1.0,
    .shape = rex::geometry::Shape{.data = rex::geometry::Sphere{.radius = 0.4}},
  });

  const rex::math::Vec3 before_velocity = world.bodies.linear_velocity(sphere_index);
  const double kinetic_before = rex::math::dot(before_velocity, before_velocity);

  (void)engine.step(world);

  const rex::dynamics::BodyState sphere = world.bodies.state(sphere_index);
  const double kinetic_after = rex::math::dot(sphere.linear_velocity, sphere.linear_velocity);

  expect(
    std::abs(sphere.linear_velocity.z) < std::abs(before_velocity.z),
    "friction should reduce tangential speed at the contact");
  expect(box_index == 0, "static box index should remain stable in the friction test");
  expect(
    sphere.linear_velocity.x >= -1.0e-9,
    "inelastic contact against a static wall should eliminate inward normal velocity");
  expect(
    kinetic_after <= kinetic_before + 1.0e-9,
    "frictional contact should not increase kinetic energy");
}

void test_centerline_sphere_collision_does_not_create_spin() {
  rex::sim::EngineConfig config{};
  config.simulation.gravity = {0.0, 0.0, 0.0};
  config.simulation.step.dt = 0.0;
  config.simulation.solver.restitution = 1.0;
  config.simulation.solver.position_correction_factor = 0.0;
  config.simulation.solver.position_iterations = 0;

  rex::sim::Engine engine{config};
  rex::dynamics::WorldState world{};
  (void)world.bodies.add_body({
    .id = rex::platform::EntityId{.index = 1, .generation = 1},
    .pose = rex::math::Transform{.translation = {0.0, 0.0, 0.0}},
    .linear_velocity = {1.0, 0.0, 0.0},
    .inverse_mass = 1.0,
    .shape = rex::geometry::Shape{.data = rex::geometry::Sphere{.radius = 0.5}},
  });
  (void)world.bodies.add_body({
    .id = rex::platform::EntityId{.index = 2, .generation = 1},
    .pose = rex::math::Transform{.translation = {0.9, 0.0, 0.0}},
    .inverse_mass = 1.0,
    .shape = rex::geometry::Shape{.data = rex::geometry::Sphere{.radius = 0.5}},
  });

  (void)engine.step(world);

  expect_vec3_nearly_equal(world.bodies.angular_velocity(0), {0.0, 0.0, 0.0}, 1.0e-9, "centerline sphere contacts should not create spin on body a");
  expect_vec3_nearly_equal(world.bodies.angular_velocity(1), {0.0, 0.0, 0.0}, 1.0e-9, "centerline sphere contacts should not create spin on body b");
}

void test_off_center_normal_contact_generates_box_spin() {
  rex::sim::EngineConfig config{};
  config.simulation.gravity = {0.0, 0.0, 0.0};
  config.simulation.step.dt = 0.0;
  config.simulation.solver.position_correction_factor = 0.0;
  config.simulation.solver.position_iterations = 0;
  config.simulation.solver.friction_coefficient = 0.0;

  rex::sim::Engine engine{config};
  rex::dynamics::WorldState world{};
  (void)world.bodies.add_body({
    .id = rex::platform::EntityId{.index = 1, .generation = 1},
    .pose = rex::math::Transform{.translation = {0.0, 0.0, 0.0}},
    .linear_velocity = {1.0, 0.0, 0.0},
    .inverse_mass = 1.0,
    .shape = rex::geometry::Shape{.data = rex::geometry::Box{.half_extents = {0.5, 0.5, 0.5}}},
  });
  (void)world.bodies.add_body({
    .id = rex::platform::EntityId{.index = 2, .generation = 1},
    .pose = rex::math::Transform{.translation = {1.1, 0.0, 0.35}},
    .inverse_mass = 0.0,
    .shape = rex::geometry::Shape{.data = rex::geometry::Sphere{.radius = 0.7}},
  });

  (void)engine.step(world);

  expect(std::abs(world.bodies.angular_velocity(0).y) > 1.0e-6, "off-center normal impulses should create box spin");
  expect(world.bodies.linear_velocity(0).x < 1.0, "normal impulse should reduce the incoming box speed");
}

void test_frictional_contact_generates_sphere_spin() {
  rex::sim::EngineConfig config{};
  config.simulation.gravity = {0.0, 0.0, 0.0};
  config.simulation.step.dt = 0.0;
  config.simulation.solver.position_correction_factor = 0.0;
  config.simulation.solver.position_iterations = 0;
  config.simulation.solver.friction_coefficient = 0.8;

  rex::sim::Engine engine{config};
  rex::dynamics::WorldState world{};
  (void)world.bodies.add_body({
    .id = rex::platform::EntityId{.index = 1, .generation = 1},
    .pose = rex::math::Transform{.translation = {0.0, 0.0, 0.45}},
    .linear_velocity = {1.0, 0.0, -0.2},
    .inverse_mass = 1.0,
    .shape = rex::geometry::Shape{.data = rex::geometry::Sphere{.radius = 0.5}},
  });
  (void)world.bodies.add_body({
    .id = rex::platform::EntityId{.index = 2, .generation = 1},
    .pose = rex::math::Transform{.translation = {0.0, 0.0, -0.25}},
    .inverse_mass = 0.0,
    .shape = rex::geometry::Shape{.data = rex::geometry::Box{.half_extents = {2.0, 2.0, 0.25}}},
  });

  (void)engine.step(world);

  expect(std::abs(world.bodies.angular_velocity(0).y) > 1.0e-6, "frictional contact should generate sphere spin");
  expect(std::abs(world.bodies.linear_velocity(0).x) < 1.0, "frictional contact should reduce tangential slip speed");
}

void test_contact_solver_projects_dynamic_body_out_of_static_box() {
  rex::sim::EngineConfig config{};
  config.simulation.gravity = {0.0, 0.0, 0.0};
  config.simulation.step.dt = 0.0;
  config.simulation.solver.position_correction_factor = 1.0;
  config.simulation.solver.position_iterations = 3;

  rex::sim::Engine engine{config};
  rex::dynamics::WorldState world{};
  const std::size_t box_index =
    world.bodies.add_body(make_box_body(1, {0.0, 0.0, 0.0}, {0.5, 0.5, 0.5}, 0.0));
  const std::size_t sphere_index =
    world.bodies.add_body(make_sphere_body(2, {0.6, 0.0, 0.0}, 0.4, 1.0));

  (void)engine.step(world);

  const rex::dynamics::BodyState sphere = world.bodies.state(1);
  expect(box_index == 0, "static box index should be stable");
  expect(sphere_index == 1, "dynamic sphere index should be stable");
  expect(sphere.pose.translation.x > 0.6, "position projection should move the sphere away from the static box");
}

void test_rotated_sphere_box_contact_matches_local_frame_geometry() {
  const rex::math::Quat box_rotation = rex::math::quat_from_axis_angle({0.0, 1.0, 0.0}, 0.5);
  const rex::math::Vec3 box_center{0.0, 0.0, 0.0};
  const rex::math::Vec3 sphere_center_world = rex::math::transform_point(
    rex::math::Transform{.rotation = box_rotation, .translation = box_center},
    {0.85, 0.1, 0.0});

  const std::vector<rex::collision::BodyProxy> bodies = {
    {.id = rex::platform::EntityId{.index = 1, .generation = 1},
     .pose = rex::math::Transform{.translation = sphere_center_world},
     .shape = rex::geometry::Shape{.data = rex::geometry::Sphere{.radius = 0.4}}},
    {.id = rex::platform::EntityId{.index = 2, .generation = 1},
     .pose = rex::math::Transform{.rotation = box_rotation, .translation = box_center},
     .shape = rex::geometry::Shape{.data = rex::geometry::Box{.half_extents = {0.5, 0.5, 0.5}}}},
  };

  const rex::collision::CollisionFrame frame =
    rex::collision::build_frame(bodies, {}, rex::collision::CollisionPipelineConfig{});
  expect(frame.manifolds.size() == 1, "rotated sphere-box overlap should generate a manifold");

  const auto& point = frame.manifolds[0].points[0];
  const rex::math::Vec3 local_center = rex::math::inverse_transform_point(
    rex::math::Transform{.rotation = box_rotation, .translation = box_center},
    sphere_center_world);
  const rex::math::Vec3 expected_local_surface{0.5, 0.1, 0.0};
  const rex::math::Vec3 expected_surface =
    rex::math::transform_point(rex::math::Transform{.rotation = box_rotation, .translation = box_center}, expected_local_surface);
  const rex::math::Vec3 expected_normal =
    rex::math::normalized_or(expected_surface - sphere_center_world, {-1.0, 0.0, 0.0});

  expect(nearly_equal(local_center.x, 0.85, 1.0e-9), "test setup should place the sphere in the rotated box frame");
  expect_vec3_nearly_equal(point.position, expected_surface, 1.0e-9, "rotated sphere-box contact point should respect the box frame");
  expect_vec3_nearly_equal(point.normal, expected_normal, 1.0e-9, "rotated sphere-box normal should respect the box frame");
}

void test_rotated_box_box_contact_produces_unit_normal_and_positive_penetration() {
  const std::vector<rex::collision::BodyProxy> bodies = {
    {.id = rex::platform::EntityId{.index = 1, .generation = 1},
     .pose = rex::math::Transform{
       .rotation = rex::math::quat_from_axis_angle({0.0, 1.0, 0.0}, 0.25),
       .translation = {0.0, 0.0, 0.0}},
     .shape = rex::geometry::Shape{.data = rex::geometry::Box{.half_extents = {0.6, 0.5, 0.4}}}},
    {.id = rex::platform::EntityId{.index = 2, .generation = 1},
     .pose = rex::math::Transform{
       .rotation = rex::math::quat_from_axis_angle({0.0, 1.0, 0.0}, -0.35),
       .translation = {0.8, 0.0, 0.1}},
     .shape = rex::geometry::Shape{.data = rex::geometry::Box{.half_extents = {0.5, 0.4, 0.5}}}},
  };

  const rex::collision::CollisionFrame frame =
    rex::collision::build_frame(bodies, {}, rex::collision::CollisionPipelineConfig{});
  expect(frame.manifolds.size() == 1, "rotated boxes should still collide");
  expect(nearly_equal(rex::math::norm(frame.manifolds[0].points[0].normal), 1.0, 1.0e-9), "rotated box normals should stay unit length");
  expect(frame.manifolds[0].points[0].penetration > 0.0, "rotated box penetration should stay positive");
}

void test_articulation_jacobian_matches_finite_difference() {
  rex::kinematics::Articulation articulation{rex::platform::EntityId{.index = 1, .generation = 1}};
  articulation.add_link({.body = rex::platform::EntityId{.index = 1, .generation = 1}, .mass = 1.0});
  articulation.add_link({.body = rex::platform::EntityId{.index = 2, .generation = 1}, .mass = 1.0});
  articulation.add_link({.body = rex::platform::EntityId{.index = 3, .generation = 1}, .mass = 1.0});
  articulation.add_joint({
    .type = rex::kinematics::JointType::kRevolute,
    .parent = rex::platform::EntityId{.index = 1, .generation = 1},
    .child = rex::platform::EntityId{.index = 2, .generation = 1},
    .parent_from_joint = rex::math::Transform{.translation = {0.0, 0.0, 0.0}},
    .child_from_joint = rex::math::Transform::identity(),
    .axis = {0.0, 1.0, 0.0},
  });
  articulation.add_joint({
    .type = rex::kinematics::JointType::kPrismatic,
    .parent = rex::platform::EntityId{.index = 2, .generation = 1},
    .child = rex::platform::EntityId{.index = 3, .generation = 1},
    .parent_from_joint = rex::math::Transform{.translation = {1.0, 0.0, 0.0}},
    .child_from_joint = rex::math::Transform::identity(),
    .axis = {1.0, 0.0, 0.0},
  });

  const std::array<double, 2> positions = {0.4, 0.3};
  const std::vector<rex::math::Vec3> jacobian = rex::kinematics::translational_jacobian(
    articulation,
    positions,
    rex::platform::EntityId{.index = 3, .generation = 1},
    {0.0, 0.0, 0.0});

  expect(jacobian.size() == 2, "articulation jacobian should expose one column per dof");

  constexpr double kEpsilon = 1.0e-6;
  for (std::size_t coordinate = 0; coordinate < jacobian.size(); ++coordinate) {
    std::array<double, 2> plus = positions;
    std::array<double, 2> minus = positions;
    plus[coordinate] += kEpsilon;
    minus[coordinate] -= kEpsilon;

    const auto poses_plus = rex::kinematics::forward_kinematics(articulation, plus);
    const auto poses_minus = rex::kinematics::forward_kinematics(articulation, minus);
    const rex::math::Vec3 point_plus =
      rex::math::transform_point(poses_plus[2], {0.0, 0.0, 0.0});
    const rex::math::Vec3 point_minus =
      rex::math::transform_point(poses_minus[2], {0.0, 0.0, 0.0});
    const rex::math::Vec3 finite_difference = (point_plus - point_minus) / (2.0 * kEpsilon);

    expect_vec3_nearly_equal(
      jacobian[coordinate],
      finite_difference,
      1.0e-6,
      "articulation jacobian should match finite differences");
  }
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
  test_broadphase_matches_bruteforce_with_large_overflow_body();
  test_persistent_manifold_keeps_cached_impulse();
  test_nonpersistent_manifold_drops_cached_impulse();
  test_sphere_box_contact_generation();
  test_box_box_contact_generation();
  test_collision_frame_invariants_hold_for_overlaps();
  test_sphere_sphere_contact_matches_exact_geometry();
  test_sphere_box_contact_matches_surface_projection();
  test_box_box_contact_stays_inside_intersection_volume();
  test_engine_step_integrates_gravity();
  test_step_profile_reports_nonnegative_phase_timings();
  test_transform_inverse_and_compose_round_trip();
  test_semi_implicit_euler_matches_closed_form_under_constant_gravity();
  test_angular_integration_matches_closed_form_rotation();
  test_shape_inverse_inertia_matches_closed_form();
  test_apply_inverse_inertia_respects_box_rotation();
  test_step_finite_difference_matches_discrete_transition_jacobian();
  test_contact_solver_conserves_momentum_and_energy_for_elastic_spheres();
  test_contact_solver_friction_dissipates_tangential_motion();
  test_centerline_sphere_collision_does_not_create_spin();
  test_off_center_normal_contact_generates_box_spin();
  test_frictional_contact_generates_sphere_spin();
  test_contact_solver_projects_dynamic_body_out_of_static_box();
  test_rotated_sphere_box_contact_matches_local_frame_geometry();
  test_rotated_box_box_contact_produces_unit_normal_and_positive_penetration();
  test_articulation_jacobian_matches_finite_difference();
  test_solver_assembles_contact_rows();
  test_static_body_does_not_integrate();
  test_engine_step_builds_contacts_and_solver_counts();
  test_engine_step_builds_multiple_contacts();
  test_engine_step_builds_mixed_shape_contacts();
  std::cout << "all core tests passed\n";
  return 0;
}
