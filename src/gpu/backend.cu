#include "rex/gpu/backend.hpp"

#include <algorithm>

namespace rex::gpu {

auto backend_name() -> std::string {
  return "cuda-placeholder";
}

auto collision_summary(
  std::span<const rex::collision::BodyProxy> bodies,
  std::span<const rex::collision::ContactManifold> previous_manifolds,
  const rex::collision::CollisionPipelineConfig& config) -> CollisionSummary {
  const rex::collision::CollisionFrame frame =
    rex::collision::build_frame(bodies, previous_manifolds, config);

  CollisionSummary summary{};
  summary.broadphase_pair_count = frame.broadphase_pairs.size();
  summary.manifold_count = frame.manifolds.size();
  for (const auto& manifold : frame.manifolds) {
    for (std::size_t point_index = 0; point_index < manifold.point_count; ++point_index) {
      summary.max_penetration = std::max(summary.max_penetration, manifold.points[point_index].penetration);
    }
  }

  return summary;
}

}  // namespace rex::gpu
