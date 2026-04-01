#include "rex/solver/solver.hpp"

#include <algorithm>
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

auto apply_impulse(
  rex::dynamics::BodyStorage& bodies,
  std::size_t body_index,
  const rex::math::Vec3& impulse) -> void {
  const double inverse_mass = bodies.inverse_mass(body_index);
  if (inverse_mass <= 0.0) {
    return;
  }

  bodies.linear_velocity_mut(body_index) += impulse * inverse_mass;
}

auto apply_position_correction(
  rex::dynamics::BodyStorage& bodies,
  std::size_t body_index,
  const rex::math::Vec3& delta) -> void {
  if (bodies.inverse_mass(body_index) <= 0.0) {
    return;
  }

  bodies.pose_mut(body_index).translation += delta;
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
  std::vector<std::vector<double>> restitution_targets(manifolds.size());
  for (std::size_t manifold_index = 0; manifold_index < manifolds.size(); ++manifold_index) {
    const auto& manifold = manifolds[manifold_index];
    restitution_targets[manifold_index].resize(manifold.point_count, 0.0);
    const BodyLookupPair body_indices = manifold_body_lookup[manifold_index];
    if (!body_indices.valid()) {
      continue;
    }

    for (std::size_t point_index = 0; point_index < manifold.point_count; ++point_index) {
      const rex::math::Vec3 normal = rex::math::normalized_or(manifold.points[point_index].normal);
      const rex::math::Vec3 relative_velocity =
        bodies.linear_velocity(body_indices.body_b) - bodies.linear_velocity(body_indices.body_a);
      const double normal_velocity = rex::math::dot(relative_velocity, normal);
      restitution_targets[manifold_index][point_index] =
        normal_velocity < 0.0 ? config.restitution * (-normal_velocity) : 0.0;
    }
  }

  if (config.warm_start) {
    for (std::size_t manifold_index = 0; manifold_index < manifolds.size(); ++manifold_index) {
      auto& manifold = manifolds[manifold_index];
      const BodyLookupPair body_indices = manifold_body_lookup[manifold_index];
      if (!body_indices.valid()) {
        continue;
      }

      for (std::size_t point_index = 0; point_index < manifold.point_count; ++point_index) {
        const rex::math::Vec3 normal = rex::math::normalized_or(manifold.points[point_index].normal);
        const TangentBasis basis = tangent_basis(normal);
        const double cached_normal_impulse = manifold.points[point_index].cached_normal_impulse;
        const double cached_tangent_impulse_u =
          config.friction_coefficient > 0.0 ? manifold.points[point_index].cached_tangent_impulse_u : 0.0;
        const double cached_tangent_impulse_v =
          config.friction_coefficient > 0.0 ? manifold.points[point_index].cached_tangent_impulse_v : 0.0;
        if (cached_normal_impulse == 0.0 && cached_tangent_impulse_u == 0.0 && cached_tangent_impulse_v == 0.0) {
          continue;
        }

        const rex::math::Vec3 total_impulse =
          (normal * cached_normal_impulse) +
          (basis.u * cached_tangent_impulse_u) +
          (basis.v * cached_tangent_impulse_v);

        apply_impulse(bodies, body_indices.body_a, total_impulse * -1.0);
        apply_impulse(bodies, body_indices.body_b, total_impulse);
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
      const BodyLookupPair body_indices = manifold_body_lookup[manifold_index];
      if (!body_indices.valid()) {
        continue;
      }

      const double inverse_mass_a = bodies.inverse_mass(body_indices.body_a);
      const double inverse_mass_b = bodies.inverse_mass(body_indices.body_b);
      const double inverse_mass_sum = inverse_mass_a + inverse_mass_b;
      if (inverse_mass_sum <= 0.0) {
        continue;
      }

      for (std::size_t point_index = 0; point_index < manifold.point_count; ++point_index) {
        auto& point = manifold.points[point_index];
        const rex::math::Vec3 normal = rex::math::normalized_or(point.normal);
        const TangentBasis basis = tangent_basis(normal);
        const rex::math::Vec3 relative_velocity =
          bodies.linear_velocity(body_indices.body_b) - bodies.linear_velocity(body_indices.body_a);
        const double normal_velocity = rex::math::dot(relative_velocity, normal);
        const double restitution_velocity = restitution_targets[manifold_index][point_index];
        const double positional_error = std::max(point.penetration - config.penetration_slop, 0.0);
        const double bias =
          dt > 0.0 ? (config.position_correction_factor * positional_error / dt) : 0.0;
        const double desired_normal_velocity = std::max(restitution_velocity, bias);

        const double incremental_impulse =
          (desired_normal_velocity - normal_velocity) / inverse_mass_sum;
        const double previous_impulse = point.cached_normal_impulse;
        const double next_impulse = std::max(previous_impulse + incremental_impulse, 0.0);
        const double applied_impulse = next_impulse - previous_impulse;
        point.cached_normal_impulse = next_impulse;

        if (applied_impulse == 0.0) {
          continue;
        }

        apply_impulse(bodies, body_indices.body_a, normal * -applied_impulse);
        apply_impulse(bodies, body_indices.body_b, normal * applied_impulse);

        const double friction_limit = config.friction_coefficient * point.cached_normal_impulse;
        if (friction_limit <= 0.0) {
          point.cached_tangent_impulse_u = 0.0;
          point.cached_tangent_impulse_v = 0.0;
          continue;
        }

        const auto solve_tangent = [&](const rex::math::Vec3& tangent, double& cached_impulse) {
          const rex::math::Vec3 tangent_relative_velocity =
            bodies.linear_velocity(body_indices.body_b) - bodies.linear_velocity(body_indices.body_a);
          const double tangent_velocity = rex::math::dot(tangent_relative_velocity, tangent);
          const double tangent_increment = -tangent_velocity / inverse_mass_sum;
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

          apply_impulse(bodies, body_indices.body_a, tangent * -applied_tangent_impulse);
          apply_impulse(bodies, body_indices.body_b, tangent * applied_tangent_impulse);
        };

        solve_tangent(basis.u, point.cached_tangent_impulse_u);
        solve_tangent(basis.v, point.cached_tangent_impulse_v);
      }
    }
  }

  for (std::size_t iteration = 0; iteration < config.position_iterations; ++iteration) {
    for (std::size_t manifold_index = 0; manifold_index < manifolds.size(); ++manifold_index) {
      const auto& manifold = manifolds[manifold_index];
      const BodyLookupPair body_indices = manifold_body_lookup[manifold_index];
      if (!body_indices.valid()) {
        continue;
      }

      const double inverse_mass_a = bodies.inverse_mass(body_indices.body_a);
      const double inverse_mass_b = bodies.inverse_mass(body_indices.body_b);
      const double inverse_mass_sum = inverse_mass_a + inverse_mass_b;
      if (inverse_mass_sum <= 0.0) {
        continue;
      }

      for (std::size_t point_index = 0; point_index < manifold.point_count; ++point_index) {
        const auto& point = manifold.points[point_index];
        const double penetration = std::max(point.penetration - config.penetration_slop, 0.0);
        if (penetration <= 0.0) {
          continue;
        }

        const rex::math::Vec3 normal = rex::math::normalized_or(point.normal);
        const rex::math::Vec3 correction =
          normal * (config.position_correction_factor * penetration / inverse_mass_sum);
        apply_position_correction(bodies, body_indices.body_a, correction * -inverse_mass_a);
        apply_position_correction(bodies, body_indices.body_b, correction * inverse_mass_b);
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
