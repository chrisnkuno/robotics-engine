#pragma once

#include <array>
#include <cstddef>

#include "rex/math/types.hpp"
#include "rex/platform/handles.hpp"

namespace rex::collision {

struct BroadphasePair {
  rex::platform::EntityId body_a{};
  rex::platform::EntityId body_b{};
};

struct ContactPoint {
  rex::math::Vec3 position{};
  rex::math::Vec3 normal{0.0, 1.0, 0.0};
  double penetration{0.0};
  double cached_normal_impulse{0.0};
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

}  // namespace rex::collision

