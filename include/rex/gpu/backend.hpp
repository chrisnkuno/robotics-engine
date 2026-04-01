#pragma once

#include <cstddef>
#include <string>
#include <span>

#include "rex/collision/contact.hpp"

namespace rex::gpu {

struct CollisionSummary {
  std::size_t broadphase_pair_count{0};
  std::size_t manifold_count{0};
  double max_penetration{0.0};
};

[[nodiscard]] auto backend_name() -> std::string;
[[nodiscard]] auto collision_summary(
  std::span<const rex::collision::BodyProxy> bodies,
  std::span<const rex::collision::ContactManifold> previous_manifolds,
  const rex::collision::CollisionPipelineConfig& config) -> CollisionSummary;

}  // namespace rex::gpu
