#pragma once

#include <array>
#include <cstddef>
#include <optional>
#include <span>
#include <vector>

#include "rex/geometry/shapes.hpp"
#include "rex/math/types.hpp"
#include "rex/platform/handles.hpp"

namespace rex::collision {

struct BodyProxy {
  rex::platform::EntityId id{};
  rex::math::Transform pose{};
  rex::geometry::Shape shape{};
};

struct BroadphasePair {
  rex::platform::EntityId body_a{};
  rex::platform::EntityId body_b{};
};

struct ContactPoint {
  rex::math::Vec3 position{};
  rex::math::Vec3 normal{0.0, 1.0, 0.0};
  double penetration{0.0};
  double cached_normal_impulse{0.0};
  double cached_tangent_impulse_u{0.0};
  double cached_tangent_impulse_v{0.0};
};

struct ContactManifold {
  static constexpr std::size_t kMaxPoints = 4;

  rex::platform::EntityId body_a{};
  rex::platform::EntityId body_b{};
  std::array<ContactPoint, kMaxPoints> points{};
  std::size_t point_count{0};
};

struct CollisionPipelineConfig {
  std::size_t max_points_per_manifold{ContactManifold::kMaxPoints};
  bool enable_persistent_manifolds{true};
};

struct CollisionFrame {
  std::vector<BroadphasePair> broadphase_pairs{};
  std::vector<ContactManifold> manifolds{};
};

[[nodiscard]] auto build_frame(
  std::span<const BodyProxy> bodies,
  std::span<const ContactManifold> previous_manifolds,
  const CollisionPipelineConfig& config) -> CollisionFrame;
[[nodiscard]] auto build_contact_manifold(const BodyProxy& body_a, const BodyProxy& body_b)
  -> std::optional<ContactManifold>;

}  // namespace rex::collision
