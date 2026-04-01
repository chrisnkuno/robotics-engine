#include "rex/collision/contact.hpp"

#include <algorithm>
#include <cmath>
#include <optional>
#include <variant>

namespace rex::collision {

namespace {

struct CandidatePair {
  BroadphasePair pair{};
  std::size_t body_index_a{0};
  std::size_t body_index_b{0};
};

struct SweepEntry {
  double min_x{0.0};
  double max_x{0.0};
  rex::platform::EntityId id{};
  std::size_t body_index{0};
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

[[nodiscard]] auto make_sweep_entry(const BodyProxy& body, std::size_t body_index) -> SweepEntry {
  const double radius = rex::geometry::bounding_radius(body.shape);
  const double center_x = body.pose.translation.x;
  return {
    .min_x = center_x - radius,
    .max_x = center_x + radius,
    .id = body.id,
    .body_index = body_index,
  };
}

[[nodiscard]] auto sweep_entry_less(const SweepEntry& lhs, const SweepEntry& rhs) -> bool {
  if (lhs.min_x < rhs.min_x) {
    return true;
  }

  if (rhs.min_x < lhs.min_x) {
    return false;
  }

  if (lhs.id < rhs.id) {
    return true;
  }

  if (rhs.id < lhs.id) {
    return false;
  }

  if (lhs.max_x < rhs.max_x) {
    return true;
  }

  if (rhs.max_x < lhs.max_x) {
    return false;
  }

  return lhs.body_index < rhs.body_index;
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

[[nodiscard]] auto clamp(double value, double lower, double upper) -> double {
  return std::max(lower, std::min(value, upper));
}

[[nodiscard]] auto clamp_to_box(const rex::math::Vec3& local, const rex::geometry::Box& box)
  -> rex::math::Vec3 {
  return {
    clamp(local.x, -box.half_extents.x, box.half_extents.x),
    clamp(local.y, -box.half_extents.y, box.half_extents.y),
    clamp(local.z, -box.half_extents.z, box.half_extents.z),
  };
}

[[nodiscard]] auto box_surface_normal(const rex::math::Vec3& local, const rex::geometry::Box& box)
  -> rex::math::Vec3 {
  const double remaining_x = box.half_extents.x - std::abs(local.x);
  const double remaining_y = box.half_extents.y - std::abs(local.y);
  const double remaining_z = box.half_extents.z - std::abs(local.z);

  if (remaining_x <= remaining_y && remaining_x <= remaining_z) {
    return {local.x >= 0.0 ? 1.0 : -1.0, 0.0, 0.0};
  }

  if (remaining_y <= remaining_z) {
    return {0.0, local.y >= 0.0 ? 1.0 : -1.0, 0.0};
  }

  return {0.0, 0.0, local.z >= 0.0 ? 1.0 : -1.0};
}

[[nodiscard]] auto box_surface_point(
  const rex::math::Vec3& center,
  const rex::math::Vec3& local,
  const rex::geometry::Box& box,
  const rex::math::Vec3& surface_normal) -> rex::math::Vec3 {
  if (surface_normal.x != 0.0) {
    return {
      center.x + (surface_normal.x * box.half_extents.x),
      center.y + local.y,
      center.z + local.z,
    };
  }

  if (surface_normal.y != 0.0) {
    return {
      center.x + local.x,
      center.y + (surface_normal.y * box.half_extents.y),
      center.z + local.z,
    };
  }

  return {
    center.x + local.x,
    center.y + local.y,
    center.z + (surface_normal.z * box.half_extents.z),
  };
}

[[nodiscard]] auto build_sphere_sphere_contact(const BodyProxy& lhs, const BodyProxy& rhs)
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

[[nodiscard]] auto build_sphere_box_contact(
  const BodyProxy& sphere_body,
  const rex::geometry::Sphere& sphere,
  const BodyProxy& box_body,
  const rex::geometry::Box& box,
  bool sphere_is_body_a) -> std::optional<ContactManifold> {
  const rex::math::Vec3 sphere_center = sphere_body.pose.translation;
  const rex::math::Vec3 box_center = box_body.pose.translation;
  const rex::math::Vec3 local_center = sphere_center - box_center;
  const rex::math::Vec3 clamped_local = clamp_to_box(local_center, box);

  rex::math::Vec3 surface_point = box_center + clamped_local;
  rex::math::Vec3 box_to_sphere = sphere_center - surface_point;
  double center_distance = rex::math::norm(box_to_sphere);
  double penetration = sphere.radius - center_distance;

  if (center_distance == 0.0) {
    box_to_sphere = box_surface_normal(local_center, box);
    surface_point = box_surface_point(box_center, local_center, box, box_to_sphere);
    const rex::math::Vec3 remaining = {
      box.half_extents.x - std::abs(local_center.x),
      box.half_extents.y - std::abs(local_center.y),
      box.half_extents.z - std::abs(local_center.z),
    };
    penetration = sphere.radius + std::min({remaining.x, remaining.y, remaining.z});
  }

  if (penetration <= 0.0) {
    return std::nullopt;
  }

  ContactManifold manifold{};
  manifold.body_a = sphere_is_body_a ? sphere_body.id : box_body.id;
  manifold.body_b = sphere_is_body_a ? box_body.id : sphere_body.id;
  manifold.point_count = 1;
  manifold.points[0].normal =
    rex::math::normalized_or(sphere_is_body_a ? (box_to_sphere * -1.0) : box_to_sphere);
  manifold.points[0].penetration = penetration;
  manifold.points[0].position = surface_point;
  return manifold;
}

[[nodiscard]] auto build_box_box_contact(
  const BodyProxy& lhs,
  const rex::geometry::Box& lhs_box,
  const BodyProxy& rhs,
  const rex::geometry::Box& rhs_box) -> std::optional<ContactManifold> {
  const rex::math::Vec3 delta = rhs.pose.translation - lhs.pose.translation;
  const rex::math::Vec3 overlap = {
    lhs_box.half_extents.x + rhs_box.half_extents.x - std::abs(delta.x),
    lhs_box.half_extents.y + rhs_box.half_extents.y - std::abs(delta.y),
    lhs_box.half_extents.z + rhs_box.half_extents.z - std::abs(delta.z),
  };

  if (overlap.x <= 0.0 || overlap.y <= 0.0 || overlap.z <= 0.0) {
    return std::nullopt;
  }

  rex::math::Vec3 normal{};
  double penetration = overlap.x;
  if (overlap.y < penetration) {
    penetration = overlap.y;
  }
  if (overlap.z < penetration) {
    penetration = overlap.z;
  }

  if (penetration == overlap.x) {
    normal = {delta.x >= 0.0 ? 1.0 : -1.0, 0.0, 0.0};
  } else if (penetration == overlap.y) {
    normal = {0.0, delta.y >= 0.0 ? 1.0 : -1.0, 0.0};
  } else {
    normal = {0.0, 0.0, delta.z >= 0.0 ? 1.0 : -1.0};
  }

  const rex::math::Vec3 lhs_min = lhs.pose.translation - lhs_box.half_extents;
  const rex::math::Vec3 lhs_max = lhs.pose.translation + lhs_box.half_extents;
  const rex::math::Vec3 rhs_min = rhs.pose.translation - rhs_box.half_extents;
  const rex::math::Vec3 rhs_max = rhs.pose.translation + rhs_box.half_extents;

  ContactManifold manifold{};
  manifold.body_a = lhs.id;
  manifold.body_b = rhs.id;
  manifold.point_count = 1;
  manifold.points[0].normal = normal;
  manifold.points[0].penetration = penetration;
  manifold.points[0].position = {
    0.5 * (std::max(lhs_min.x, rhs_min.x) + std::min(lhs_max.x, rhs_max.x)),
    0.5 * (std::max(lhs_min.y, rhs_min.y) + std::min(lhs_max.y, rhs_max.y)),
    0.5 * (std::max(lhs_min.z, rhs_min.z) + std::min(lhs_max.z, rhs_max.z)),
  };
  return manifold;
}

[[nodiscard]] auto build_contact(const BodyProxy& lhs, const BodyProxy& rhs)
  -> std::optional<ContactManifold> {
  const auto* lhs_sphere = std::get_if<rex::geometry::Sphere>(&lhs.shape.data);
  const auto* rhs_sphere = std::get_if<rex::geometry::Sphere>(&rhs.shape.data);
  const auto* lhs_box = std::get_if<rex::geometry::Box>(&lhs.shape.data);
  const auto* rhs_box = std::get_if<rex::geometry::Box>(&rhs.shape.data);

  if (lhs_sphere != nullptr && rhs_sphere != nullptr) {
    return build_sphere_sphere_contact(lhs, rhs);
  }

  if (lhs_sphere != nullptr && rhs_box != nullptr) {
    return build_sphere_box_contact(lhs, *lhs_sphere, rhs, *rhs_box, true);
  }

  if (lhs_box != nullptr && rhs_sphere != nullptr) {
    return build_sphere_box_contact(rhs, *rhs_sphere, lhs, *lhs_box, false);
  }

  if (lhs_box != nullptr && rhs_box != nullptr) {
    return build_box_box_contact(lhs, *lhs_box, rhs, *rhs_box);
  }

  return std::nullopt;
}

[[nodiscard]] auto build_broadphase_pairs(std::span<const BodyProxy> bodies) -> std::vector<CandidatePair> {
  std::vector<SweepEntry> entries{};
  entries.reserve(bodies.size());

  for (std::size_t body_index = 0; body_index < bodies.size(); ++body_index) {
    entries.push_back(make_sweep_entry(bodies[body_index], body_index));
  }

  std::sort(entries.begin(), entries.end(), sweep_entry_less);

  std::vector<std::size_t> active_entries{};
  active_entries.reserve(entries.size());

  std::vector<CandidatePair> candidates{};

  for (std::size_t entry_index = 0; entry_index < entries.size(); ++entry_index) {
    const SweepEntry& current = entries[entry_index];

    active_entries.erase(
      std::remove_if(
        active_entries.begin(),
        active_entries.end(),
        [&](std::size_t active_index) { return entries[active_index].max_x < current.min_x; }),
      active_entries.end());

    for (std::size_t active_index : active_entries) {
      const SweepEntry& other = entries[active_index];
      if (!broadphase_overlap(bodies[other.body_index], bodies[current.body_index])) {
        continue;
      }

      candidates.push_back(make_pair(
        bodies[other.body_index].id,
        bodies[current.body_index].id,
        other.body_index,
        current.body_index));
    }

    active_entries.push_back(entry_index);
  }

  std::sort(
    candidates.begin(),
    candidates.end(),
    [](const CandidatePair& lhs, const CandidatePair& rhs) { return pair_less(lhs.pair, rhs.pair); });
  return candidates;
}

}  // namespace

auto build_frame(
  std::span<const BodyProxy> bodies,
  std::span<const ContactManifold> previous_manifolds,
  const CollisionPipelineConfig& config) -> CollisionFrame {
  const std::vector<CandidatePair> candidates = build_broadphase_pairs(bodies);

  CollisionFrame frame{};
  frame.broadphase_pairs.reserve(candidates.size());
  frame.manifolds.reserve(candidates.size());

  for (const CandidatePair& candidate : candidates) {
    frame.broadphase_pairs.push_back(candidate.pair);

    auto maybe_manifold = build_contact(
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
