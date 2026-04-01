#include "rex/solver/solver.hpp"

#include <limits>

namespace rex::solver {

namespace {

[[nodiscard]] auto orthogonal_tangent(const rex::math::Vec3& normal) -> rex::math::Vec3 {
  const rex::math::Vec3 reference =
    std::abs(normal.z) < 0.999 ? rex::math::Vec3{0.0, 0.0, 1.0} : rex::math::Vec3{0.0, 1.0, 0.0};
  return rex::math::normalized_or(rex::math::cross(normal, reference));
}

}  // namespace

auto assemble_contact_rows(std::span<const rex::collision::ContactManifold> manifolds)
  -> ConstraintAssembly {
  constexpr double kInfinity = std::numeric_limits<double>::infinity();

  ConstraintAssembly assembly{};

  for (const auto& manifold : manifolds) {
    for (std::size_t point_index = 0; point_index < manifold.point_count; ++point_index) {
      const rex::math::Vec3 normal = rex::math::normalized_or(manifold.points[point_index].normal);
      const rex::math::Vec3 tangent_u = orthogonal_tangent(normal);
      const rex::math::Vec3 tangent_v = rex::math::normalized_or(rex::math::cross(normal, tangent_u));

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
        .direction = tangent_u,
        .axis = ConstraintAxis::kTangentU,
        .lower_impulse_limit = -kInfinity,
        .upper_impulse_limit = kInfinity,
      });
      assembly.rows.push_back({
        .body_a = manifold.body_a,
        .body_b = manifold.body_b,
        .direction = tangent_v,
        .axis = ConstraintAxis::kTangentV,
        .lower_impulse_limit = -kInfinity,
        .upper_impulse_limit = kInfinity,
      });
    }
  }

  return assembly;
}

auto describe(const SolverConfig& config) -> std::string {
  std::string description = "sequential-impulse-pgs";

  description += config.warm_start ? " warm-start" : " cold-start";
  description += config.deterministic_ordering ? " deterministic" : " relaxed-order";
  return description;
}

}  // namespace rex::solver
