#include "rex/solver/solver.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <limits>
#include <unordered_map>

#include "rex/dynamics/world.hpp"

namespace rex::solver {

namespace {

[[nodiscard]] auto orthogonal_tangent(const rex::math::Vec3& normal) -> rex::math::Vec3 {
  const rex::math::Vec3 reference =
    std::abs(normal.z) < 0.999 ? rex::math::Vec3{0.0, 0.0, 1.0} : rex::math::Vec3{0.0, 1.0, 0.0};
  return rex::math::normalized_or(rex::math::cross(normal, reference));
}

struct TangentBasis {
  rex::math::Vec3 u{};
  rex::math::Vec3 v{};
};

struct BodyLookupPair {
  std::size_t body_a{std::numeric_limits<std::size_t>::max()};
  std::size_t body_b{std::numeric_limits<std::size_t>::max()};

  [[nodiscard]] auto valid() const noexcept -> bool {
    return body_a != std::numeric_limits<std::size_t>::max() &&
           body_b != std::numeric_limits<std::size_t>::max();
  }
};

struct PreparedPoint {
  rex::math::Vec3 position{};
  rex::math::Vec3 normal{};
  TangentBasis basis{};
  rex::math::Vec3 lever_arm_a{};
  rex::math::Vec3 lever_arm_b{};
  double normal_mass{0.0};
  double tangent_mass_u{0.0};
  double tangent_mass_v{0.0};
};

struct PreparedManifold {
  BodyLookupPair body_indices{};
  std::array<PreparedPoint, rex::collision::ContactManifold::kMaxPoints> points{};
  std::size_t point_count{0};

  [[nodiscard]] auto valid() const noexcept -> bool {
    return body_indices.valid();
  }
};

[[nodiscard]] auto entity_key(rex::platform::EntityId id) -> std::uint64_t {
  return (static_cast<std::uint64_t>(id.generation) << 32) | static_cast<std::uint64_t>(id.index);
}

[[nodiscard]] auto tangent_basis(const rex::math::Vec3& normal) -> TangentBasis {
  const rex::math::Vec3 tangent_u = orthogonal_tangent(normal);
  return {
    .u = tangent_u,
    .v = rex::math::normalized_or(rex::math::cross(normal, tangent_u)),
  };
}

[[nodiscard]] auto build_body_lookup(const rex::dynamics::BodyStorage& bodies)
  -> std::unordered_map<std::uint64_t, std::size_t> {
  std::unordered_map<std::uint64_t, std::size_t> lookup{};
  lookup.reserve(bodies.size());
  for (std::size_t body_index = 0; body_index < bodies.size(); ++body_index) {
    lookup.emplace(entity_key(bodies.id(body_index)), body_index);
  }
  return lookup;
}

[[nodiscard]] auto build_manifold_body_lookup(
  const std::unordered_map<std::uint64_t, std::size_t>& body_lookup,
  std::span<const rex::collision::ContactManifold> manifolds) -> std::vector<BodyLookupPair> {
  std::vector<BodyLookupPair> lookup_pairs(manifolds.size());
  for (std::size_t manifold_index = 0; manifold_index < manifolds.size(); ++manifold_index) {
    const auto body_a_it = body_lookup.find(entity_key(manifolds[manifold_index].body_a));
    const auto body_b_it = body_lookup.find(entity_key(manifolds[manifold_index].body_b));
    if (body_a_it == body_lookup.end() || body_b_it == body_lookup.end()) {
      continue;
    }

    lookup_pairs[manifold_index].body_a = body_a_it->second;
    lookup_pairs[manifold_index].body_b = body_b_it->second;
  }
  return lookup_pairs;
}

[[nodiscard]] auto lever_arm(
  const rex::dynamics::BodyStorage& bodies,
  std::size_t body_index,
  const rex::math::Vec3& contact_position) -> rex::math::Vec3 {
  return contact_position - bodies.pose(body_index).translation;
}

auto apply_impulse(
  rex::dynamics::BodyStorage& bodies,
  std::size_t body_index,
  const rex::math::Vec3& lever_arm,
  const rex::math::Vec3& impulse) -> void {
  const double inverse_mass = bodies.inverse_mass(body_index);
  if (inverse_mass <= 0.0) {
    return;
  }

  bodies.linear_velocity_mut(body_index) += impulse * inverse_mass;
  const rex::math::Vec3 angular_impulse = rex::math::cross(lever_arm, impulse);
  bodies.angular_velocity_mut(body_index) += rex::geometry::apply_inverse_inertia(
    bodies.shape(body_index),
    bodies.pose(body_index).rotation,
    inverse_mass,
    angular_impulse);
}

auto apply_position_impulse(
  rex::dynamics::BodyStorage& bodies,
  std::size_t body_index,
  const rex::math::Vec3& contact_position,
  const rex::math::Vec3& impulse) -> void {
  const double inverse_mass = bodies.inverse_mass(body_index);
  if (inverse_mass <= 0.0) {
    return;
  }

  auto& pose = bodies.pose_mut(body_index);
  const rex::math::Vec3 current_lever_arm = contact_position - pose.translation;
  const rex::math::Vec3 angular_impulse = rex::math::cross(current_lever_arm, impulse);
  const rex::math::Vec3 angular_delta = rex::geometry::apply_inverse_inertia(
    bodies.shape(body_index),
    pose.rotation,
    inverse_mass,
    angular_impulse);
  pose.translation += impulse * inverse_mass;
  pose.rotation = rex::math::integrate_rotation(pose.rotation, angular_delta, 1.0);
}

[[nodiscard]] auto point_velocity(
  const rex::dynamics::BodyStorage& bodies,
  std::size_t body_index,
  const rex::math::Vec3& lever_arm) -> rex::math::Vec3 {
  return bodies.linear_velocity(body_index) +
         rex::math::cross(bodies.angular_velocity(body_index), lever_arm);
}

[[nodiscard]] auto relative_point_velocity(
  const rex::dynamics::BodyStorage& bodies,
  std::size_t body_index_a,
  std::size_t body_index_b,
  const rex::math::Vec3& lever_arm_a,
  const rex::math::Vec3& lever_arm_b) -> rex::math::Vec3 {
  return point_velocity(bodies, body_index_b, lever_arm_b) -
         point_velocity(bodies, body_index_a, lever_arm_a);
}

[[nodiscard]] auto impulse_denominator(
  const rex::dynamics::BodyStorage& bodies,
  std::size_t body_index,
  const rex::math::Vec3& lever_arm,
  const rex::math::Vec3& direction) -> double {
  const double inverse_mass = bodies.inverse_mass(body_index);
  if (inverse_mass <= 0.0) {
    return 0.0;
  }

  const rex::math::Vec3 angular_axis = rex::math::cross(lever_arm, direction);
  const rex::math::Vec3 world_inverse_inertia_axis = rex::geometry::apply_inverse_inertia(
    bodies.shape(body_index),
    bodies.pose(body_index).rotation,
    inverse_mass,
    angular_axis);
  return inverse_mass +
         rex::math::dot(
           rex::math::cross(world_inverse_inertia_axis, lever_arm),
           direction);
}

[[nodiscard]] auto build_prepared_manifolds(
  const rex::dynamics::BodyStorage& bodies,
  std::span<const rex::collision::ContactManifold> manifolds,
  std::span<const BodyLookupPair> manifold_body_lookup) -> std::vector<PreparedManifold> {
  std::vector<PreparedManifold> prepared(manifolds.size());
  for (std::size_t manifold_index = 0; manifold_index < manifolds.size(); ++manifold_index) {
    PreparedManifold& prepared_manifold = prepared[manifold_index];
    prepared_manifold.body_indices = manifold_body_lookup[manifold_index];
    prepared_manifold.point_count = manifolds[manifold_index].point_count;
    if (!prepared_manifold.valid()) {
      continue;
    }

    for (std::size_t point_index = 0; point_index < prepared_manifold.point_count; ++point_index) {
      const rex::collision::ContactPoint& point = manifolds[manifold_index].points[point_index];
      PreparedPoint& prepared_point = prepared_manifold.points[point_index];
      prepared_point.position = point.position;
      prepared_point.normal = rex::math::normalized_or(point.normal);
      prepared_point.basis = tangent_basis(prepared_point.normal);
      prepared_point.lever_arm_a =
        lever_arm(bodies, prepared_manifold.body_indices.body_a, point.position);
      prepared_point.lever_arm_b =
        lever_arm(bodies, prepared_manifold.body_indices.body_b, point.position);
      prepared_point.normal_mass =
        impulse_denominator(
          bodies,
          prepared_manifold.body_indices.body_a,
          prepared_point.lever_arm_a,
          prepared_point.normal) +
        impulse_denominator(
          bodies,
          prepared_manifold.body_indices.body_b,
          prepared_point.lever_arm_b,
          prepared_point.normal);
      prepared_point.tangent_mass_u =
        impulse_denominator(
          bodies,
          prepared_manifold.body_indices.body_a,
          prepared_point.lever_arm_a,
          prepared_point.basis.u) +
        impulse_denominator(
          bodies,
          prepared_manifold.body_indices.body_b,
          prepared_point.lever_arm_b,
          prepared_point.basis.u);
      prepared_point.tangent_mass_v =
        impulse_denominator(
          bodies,
          prepared_manifold.body_indices.body_a,
          prepared_point.lever_arm_a,
          prepared_point.basis.v) +
        impulse_denominator(
          bodies,
          prepared_manifold.body_indices.body_b,
          prepared_point.lever_arm_b,
          prepared_point.basis.v);
    }
  }

  return prepared;
}

[[nodiscard]] auto body_proxy(const rex::dynamics::BodyStorage& bodies, std::size_t body_index)
  -> rex::collision::BodyProxy {
  return {
    .id = bodies.id(body_index),
    .pose = bodies.pose(body_index),
    .shape = bodies.shape(body_index),
  };
}

void refresh_manifold_geometry(
  rex::dynamics::BodyStorage& bodies,
  BodyLookupPair body_indices,
  rex::collision::ContactManifold& manifold) {
  if (!body_indices.valid()) {
    return;
  }

  auto refreshed = rex::collision::build_contact_manifold(
    body_proxy(bodies, body_indices.body_a),
    body_proxy(bodies, body_indices.body_b));
  if (!refreshed.has_value()) {
    manifold.point_count = 0;
    return;
  }

  const std::size_t persistent_points = std::min(manifold.point_count, refreshed->point_count);
  for (std::size_t point_index = 0; point_index < persistent_points; ++point_index) {
    refreshed->points[point_index].cached_normal_impulse =
      manifold.points[point_index].cached_normal_impulse;
    refreshed->points[point_index].cached_tangent_impulse_u =
      manifold.points[point_index].cached_tangent_impulse_u;
    refreshed->points[point_index].cached_tangent_impulse_v =
      manifold.points[point_index].cached_tangent_impulse_v;
  }

  manifold = *refreshed;
}

}  // namespace

auto assemble_contact_rows(std::span<const rex::collision::ContactManifold> manifolds)
  -> ConstraintAssembly {
  constexpr double kInfinity = std::numeric_limits<double>::infinity();

  ConstraintAssembly assembly{};

  for (const auto& manifold : manifolds) {
    for (std::size_t point_index = 0; point_index < manifold.point_count; ++point_index) {
      const rex::math::Vec3 normal = rex::math::normalized_or(manifold.points[point_index].normal);
      const TangentBasis basis = tangent_basis(normal);

      assembly.rows.push_back({
        .body_a = manifold.body_a,
        .body_b = manifold.body_b,
        .direction = normal,
        .axis = ConstraintAxis::kNormal,
        .lower_impulse_limit = 0.0,
        .upper_impulse_limit = kInfinity,
      });
      assembly.rows.push_back({
        .body_a = manifold.body_a,
        .body_b = manifold.body_b,
        .direction = basis.u,
        .axis = ConstraintAxis::kTangentU,
        .lower_impulse_limit = -kInfinity,
        .upper_impulse_limit = kInfinity,
      });
      assembly.rows.push_back({
        .body_a = manifold.body_a,
        .body_b = manifold.body_b,
        .direction = basis.v,
        .axis = ConstraintAxis::kTangentV,
        .lower_impulse_limit = -kInfinity,
        .upper_impulse_limit = kInfinity,
      });
    }
  }

  return assembly;
}

auto solve_contacts(
  rex::dynamics::BodyStorage& bodies,
  std::span<rex::collision::ContactManifold> manifolds,
  const SolverConfig& config,
  double dt) -> SolverResult {
  SolverResult result{};

  for (const auto& manifold : manifolds) {
    result.contact_count += manifold.point_count;
    result.constraint_count += manifold.point_count * 3;
    for (std::size_t point_index = 0; point_index < manifold.point_count; ++point_index) {
      result.max_penetration = std::max(result.max_penetration, manifold.points[point_index].penetration);
    }
  }

  if (manifolds.empty()) {
    return result;
  }

  const auto body_lookup = build_body_lookup(bodies);
  const std::vector<BodyLookupPair> manifold_body_lookup =
    build_manifold_body_lookup(body_lookup, manifolds);
  const std::vector<PreparedManifold> prepared_manifolds =
    build_prepared_manifolds(bodies, manifolds, manifold_body_lookup);
  std::vector<std::vector<double>> restitution_targets(manifolds.size());
  for (std::size_t manifold_index = 0; manifold_index < manifolds.size(); ++manifold_index) {
    const auto& manifold = manifolds[manifold_index];
    restitution_targets[manifold_index].resize(manifold.point_count, 0.0);
    const PreparedManifold& prepared_manifold = prepared_manifolds[manifold_index];
    if (!prepared_manifold.valid()) {
      continue;
    }

    for (std::size_t point_index = 0; point_index < manifold.point_count; ++point_index) {
      const rex::math::Vec3 relative_velocity = relative_point_velocity(
        bodies,
        prepared_manifold.body_indices.body_a,
        prepared_manifold.body_indices.body_b,
        prepared_manifold.points[point_index].lever_arm_a,
        prepared_manifold.points[point_index].lever_arm_b);
      const double normal_velocity =
        rex::math::dot(relative_velocity, prepared_manifold.points[point_index].normal);
      restitution_targets[manifold_index][point_index] =
        normal_velocity < 0.0 ? config.restitution * (-normal_velocity) : 0.0;
    }
  }

  if (config.warm_start) {
    for (std::size_t manifold_index = 0; manifold_index < manifolds.size(); ++manifold_index) {
      auto& manifold = manifolds[manifold_index];
      const PreparedManifold& prepared_manifold = prepared_manifolds[manifold_index];
      if (!prepared_manifold.valid()) {
        continue;
      }

      for (std::size_t point_index = 0; point_index < manifold.point_count; ++point_index) {
        const PreparedPoint& prepared_point = prepared_manifold.points[point_index];
        const double cached_normal_impulse = manifold.points[point_index].cached_normal_impulse;
        const double cached_tangent_impulse_u =
          config.friction_coefficient > 0.0 ? manifold.points[point_index].cached_tangent_impulse_u : 0.0;
        const double cached_tangent_impulse_v =
          config.friction_coefficient > 0.0 ? manifold.points[point_index].cached_tangent_impulse_v : 0.0;
        if (cached_normal_impulse == 0.0 && cached_tangent_impulse_u == 0.0 && cached_tangent_impulse_v == 0.0) {
          continue;
        }

        const rex::math::Vec3 total_impulse =
          (prepared_point.normal * cached_normal_impulse) +
          (prepared_point.basis.u * cached_tangent_impulse_u) +
          (prepared_point.basis.v * cached_tangent_impulse_v);

        apply_impulse(
          bodies,
          prepared_manifold.body_indices.body_a,
          prepared_point.lever_arm_a,
          total_impulse * -1.0);
        apply_impulse(
          bodies,
          prepared_manifold.body_indices.body_b,
          prepared_point.lever_arm_b,
          total_impulse);
      }
    }
  } else {
    for (auto& manifold : manifolds) {
      for (std::size_t point_index = 0; point_index < manifold.point_count; ++point_index) {
        manifold.points[point_index].cached_normal_impulse = 0.0;
        manifold.points[point_index].cached_tangent_impulse_u = 0.0;
        manifold.points[point_index].cached_tangent_impulse_v = 0.0;
      }
    }
  }

  for (std::size_t iteration = 0; iteration < config.velocity_iterations; ++iteration) {
    for (std::size_t manifold_index = 0; manifold_index < manifolds.size(); ++manifold_index) {
      auto& manifold = manifolds[manifold_index];
      const PreparedManifold& prepared_manifold = prepared_manifolds[manifold_index];
      if (!prepared_manifold.valid()) {
        continue;
      }

      for (std::size_t point_index = 0; point_index < manifold.point_count; ++point_index) {
        auto& point = manifold.points[point_index];
        const PreparedPoint& prepared_point = prepared_manifold.points[point_index];
        if (prepared_point.normal_mass <= 0.0) {
          continue;
        }

        const rex::math::Vec3 relative_velocity = relative_point_velocity(
          bodies,
          prepared_manifold.body_indices.body_a,
          prepared_manifold.body_indices.body_b,
          prepared_point.lever_arm_a,
          prepared_point.lever_arm_b);
        const double normal_velocity = rex::math::dot(relative_velocity, prepared_point.normal);
        const double restitution_velocity = restitution_targets[manifold_index][point_index];
        const double positional_error = std::max(point.penetration - config.penetration_slop, 0.0);
        const double bias =
          dt > 0.0 ? (config.position_correction_factor * positional_error / dt) : 0.0;
        const double desired_normal_velocity = std::max(restitution_velocity, bias);

        const double incremental_impulse =
          (desired_normal_velocity - normal_velocity) / prepared_point.normal_mass;
        const double previous_impulse = point.cached_normal_impulse;
        const double next_impulse = std::max(previous_impulse + incremental_impulse, 0.0);
        const double applied_impulse = next_impulse - previous_impulse;
        point.cached_normal_impulse = next_impulse;

        if (applied_impulse == 0.0) {
          continue;
        }

        apply_impulse(
          bodies,
          prepared_manifold.body_indices.body_a,
          prepared_point.lever_arm_a,
          prepared_point.normal * -applied_impulse);
        apply_impulse(
          bodies,
          prepared_manifold.body_indices.body_b,
          prepared_point.lever_arm_b,
          prepared_point.normal * applied_impulse);

        const double friction_limit = config.friction_coefficient * point.cached_normal_impulse;
        if (friction_limit <= 0.0) {
          point.cached_tangent_impulse_u = 0.0;
          point.cached_tangent_impulse_v = 0.0;
          continue;
        }

        const auto solve_tangent = [&](const rex::math::Vec3& tangent, double tangent_mass, double& cached_impulse) {
          if (tangent_mass <= 0.0) {
            return;
          }

          const rex::math::Vec3 tangent_relative_velocity = relative_point_velocity(
            bodies,
            prepared_manifold.body_indices.body_a,
            prepared_manifold.body_indices.body_b,
            prepared_point.lever_arm_a,
            prepared_point.lever_arm_b);
          const double tangent_velocity = rex::math::dot(tangent_relative_velocity, tangent);
          const double tangent_increment = -tangent_velocity / tangent_mass;
          const double previous_tangent_impulse = cached_impulse;
          const double next_tangent_impulse = std::clamp(
            previous_tangent_impulse + tangent_increment,
            -friction_limit,
            friction_limit);
          const double applied_tangent_impulse = next_tangent_impulse - previous_tangent_impulse;
          cached_impulse = next_tangent_impulse;
          if (applied_tangent_impulse == 0.0) {
            return;
          }

          apply_impulse(
            bodies,
            prepared_manifold.body_indices.body_a,
            prepared_point.lever_arm_a,
            tangent * -applied_tangent_impulse);
          apply_impulse(
            bodies,
            prepared_manifold.body_indices.body_b,
            prepared_point.lever_arm_b,
            tangent * applied_tangent_impulse);
        };

        solve_tangent(
          prepared_point.basis.u,
          prepared_point.tangent_mass_u,
          point.cached_tangent_impulse_u);
        solve_tangent(
          prepared_point.basis.v,
          prepared_point.tangent_mass_v,
          point.cached_tangent_impulse_v);
      }
    }
  }

  for (std::size_t iteration = 0; iteration < config.position_iterations; ++iteration) {
    for (std::size_t manifold_index = 0; manifold_index < manifolds.size(); ++manifold_index) {
      auto& manifold = manifolds[manifold_index];
      const BodyLookupPair body_indices = manifold_body_lookup[manifold_index];
      if (!body_indices.valid()) {
        continue;
      }

      refresh_manifold_geometry(bodies, body_indices, manifold);
      if (manifold.point_count == 0) {
        continue;
      }

      for (std::size_t point_index = 0; point_index < manifold.point_count; ++point_index) {
        const auto& point = manifold.points[point_index];
        const double penetration = std::max(point.penetration - config.penetration_slop, 0.0);
        if (penetration <= 0.0) {
          continue;
        }

        const rex::math::Vec3 normal = rex::math::normalized_or(point.normal);
        const rex::math::Vec3 lever_arm_a = lever_arm(bodies, body_indices.body_a, point.position);
        const rex::math::Vec3 lever_arm_b = lever_arm(bodies, body_indices.body_b, point.position);
        const double position_mass =
          impulse_denominator(bodies, body_indices.body_a, lever_arm_a, normal) +
          impulse_denominator(bodies, body_indices.body_b, lever_arm_b, normal);
        if (position_mass <= 0.0) {
          continue;
        }

        const double impulse_magnitude =
          config.position_correction_factor * penetration / position_mass;
        const rex::math::Vec3 impulse = normal * impulse_magnitude;
        apply_position_impulse(bodies, body_indices.body_a, point.position, impulse * -1.0);
        apply_position_impulse(bodies, body_indices.body_b, point.position, impulse);
      }
    }
  }

  return result;
}

auto describe(const SolverConfig& config) -> std::string {
  std::string description = "sequential-impulse-pgs";

  description += config.warm_start ? " warm-start" : " cold-start";
  description += config.deterministic_ordering ? " deterministic" : " relaxed-order";
  description += config.position_iterations > 0 ? " projected" : " velocity-only";
  return description;
}

}  // namespace rex::solver
