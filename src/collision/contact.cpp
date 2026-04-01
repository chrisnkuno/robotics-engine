#include "rex/collision/contact.hpp"

#include <algorithm>
#include <optional>
#include <variant>

namespace rex::collision {

namespace {

struct CandidatePair {
  BroadphasePair pair{};
  std::size_t body_index_a{0};
  std::size_t body_index_b{0};
};

[[nodiscard]] auto make_pair(
  rex::platform::EntityId lhs,
  rex::platform::EntityId rhs,
  std::size_t lhs_index,
  std::size_t rhs_index) -> CandidatePair {
  if (rhs < lhs) {
    return {{rhs, lhs}, rhs_index, lhs_index};
  }

  return {{lhs, rhs}, lhs_index, rhs_index};
}

[[nodiscard]] auto pair_less(const BroadphasePair& lhs, const BroadphasePair& rhs) -> bool {
  if (lhs.body_a < rhs.body_a) {
    return true;
  }

  if (rhs.body_a < lhs.body_a) {
    return false;
  }

  return lhs.body_b < rhs.body_b;
}

[[nodiscard]] auto same_pair(const ContactManifold& manifold, const BroadphasePair& pair) -> bool {
  return manifold.body_a == pair.body_a && manifold.body_b == pair.body_b;
}

[[nodiscard]] auto broadphase_overlap(const BodyProxy& lhs, const BodyProxy& rhs) -> bool {
  const double radius_sum =
    rex::geometry::bounding_radius(lhs.shape) + rex::geometry::bounding_radius(rhs.shape);
  const double center_distance = rex::math::distance(lhs.pose.translation, rhs.pose.translation);
  return center_distance <= radius_sum;
}

[[nodiscard]] auto build_sphere_contact(const BodyProxy& lhs, const BodyProxy& rhs)
  -> std::optional<ContactManifold> {
  const auto* lhs_sphere = std::get_if<rex::geometry::Sphere>(&lhs.shape.data);
  const auto* rhs_sphere = std::get_if<rex::geometry::Sphere>(&rhs.shape.data);
  if (lhs_sphere == nullptr || rhs_sphere == nullptr) {
    return std::nullopt;
  }

  const rex::math::Vec3 delta = rhs.pose.translation - lhs.pose.translation;
  const double radius_sum = lhs_sphere->radius + rhs_sphere->radius;
  const double center_distance = rex::math::norm(delta);
  const double penetration = radius_sum - center_distance;
  if (penetration <= 0.0) {
    return std::nullopt;
  }

  const rex::math::Vec3 normal = rex::math::normalized_or(delta);

  ContactManifold manifold{};
  manifold.body_a = lhs.id;
  manifold.body_b = rhs.id;
  manifold.point_count = 1;
  manifold.points[0].normal = normal;
  manifold.points[0].penetration = penetration;
  manifold.points[0].position =
    lhs.pose.translation + normal * (lhs_sphere->radius - (penetration * 0.5));
  return manifold;
}

}  // namespace

auto build_frame(
  std::span<const BodyProxy> bodies,
  std::span<const ContactManifold> previous_manifolds,
  const CollisionPipelineConfig& config) -> CollisionFrame {
  std::vector<CandidatePair> candidates{};
  const std::size_t body_count = bodies.size();
  candidates.reserve(body_count > 1 ? body_count * (body_count - 1) / 2 : 0);

  for (std::size_t lhs_index = 0; lhs_index < bodies.size(); ++lhs_index) {
    for (std::size_t rhs_index = lhs_index + 1; rhs_index < bodies.size(); ++rhs_index) {
      if (!broadphase_overlap(bodies[lhs_index], bodies[rhs_index])) {
        continue;
      }

      candidates.push_back(
        make_pair(bodies[lhs_index].id, bodies[rhs_index].id, lhs_index, rhs_index));
    }
  }

  std::sort(
    candidates.begin(),
    candidates.end(),
    [](const CandidatePair& lhs, const CandidatePair& rhs) { return pair_less(lhs.pair, rhs.pair); });

  CollisionFrame frame{};
  frame.broadphase_pairs.reserve(candidates.size());
  frame.manifolds.reserve(candidates.size());

  for (const CandidatePair& candidate : candidates) {
    frame.broadphase_pairs.push_back(candidate.pair);

    auto maybe_manifold = build_sphere_contact(
      bodies[candidate.body_index_a],
      bodies[candidate.body_index_b]);
    if (!maybe_manifold.has_value()) {
      continue;
    }

    if (config.enable_persistent_manifolds) {
      const auto previous_it = std::find_if(
        previous_manifolds.begin(),
        previous_manifolds.end(),
        [&](const ContactManifold& previous) { return same_pair(previous, candidate.pair); });

      if (previous_it != previous_manifolds.end()) {
        const std::size_t persistent_points =
          std::min(previous_it->point_count, maybe_manifold->point_count);
        for (std::size_t point_index = 0; point_index < persistent_points; ++point_index) {
          maybe_manifold->points[point_index].cached_normal_impulse =
            previous_it->points[point_index].cached_normal_impulse;
        }
      }
    }

    maybe_manifold->point_count =
      std::min<std::size_t>(maybe_manifold->point_count, config.max_points_per_manifold);
    frame.manifolds.push_back(*maybe_manifold);
  }

  return frame;
}

}  // namespace rex::collision
