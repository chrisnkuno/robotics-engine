#include "rex/collision/contact.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <unordered_map>
#include <variant>

namespace rex::collision {

namespace {

struct CandidatePair {
  BroadphasePair pair{};
  std::size_t body_index_a{0};
  std::size_t body_index_b{0};
};

struct BroadphaseBounds {
  rex::math::Vec3 min{};
  rex::math::Vec3 max{};
};

struct CellCoord {
  std::int64_t x{0};
  std::int64_t y{0};
  std::int64_t z{0};

  auto operator==(const CellCoord&) const -> bool = default;
};

struct CellMembership {
  CellCoord coord{};
  rex::platform::EntityId id{};
  std::size_t body_index{0};
};

struct OrientedBox {
  rex::math::Vec3 center{};
  std::array<rex::math::Vec3, 3> axes{};
  std::array<double, 3> half_extents{};
};

struct PairKey {
  std::uint64_t body_a{0};
  std::uint64_t body_b{0};

  auto operator==(const PairKey&) const -> bool = default;
};

struct PairKeyHash {
  [[nodiscard]] auto operator()(const PairKey& key) const noexcept -> std::size_t {
    const std::size_t hash_a = std::hash<std::uint64_t>{}(key.body_a);
    const std::size_t hash_b = std::hash<std::uint64_t>{}(key.body_b);
    return hash_a ^ (hash_b + 0x9e3779b97f4a7c15ull + (hash_a << 6) + (hash_a >> 2));
  }
};

[[nodiscard]] auto entity_key(rex::platform::EntityId id) -> std::uint64_t {
  return (static_cast<std::uint64_t>(id.generation) << 32) | static_cast<std::uint64_t>(id.index);
}

[[nodiscard]] auto pair_key(const BroadphasePair& pair) -> PairKey {
  return {
    .body_a = entity_key(pair.body_a),
    .body_b = entity_key(pair.body_b),
  };
}

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

[[nodiscard]] auto candidate_pair_less(const CandidatePair& lhs, const CandidatePair& rhs) -> bool {
  return pair_less(lhs.pair, rhs.pair);
}

[[nodiscard]] auto candidate_pair_equal(const CandidatePair& lhs, const CandidatePair& rhs) -> bool {
  return lhs.pair.body_a == rhs.pair.body_a && lhs.pair.body_b == rhs.pair.body_b;
}

[[nodiscard]] auto broadphase_bounds(const BodyProxy& body) -> BroadphaseBounds {
  const double radius = rex::geometry::bounding_radius(body.shape);
  return {
    .min = body.pose.translation - rex::math::Vec3{radius, radius, radius},
    .max = body.pose.translation + rex::math::Vec3{radius, radius, radius},
  };
}

[[nodiscard]] auto cell_coord_less(const CellCoord& lhs, const CellCoord& rhs) -> bool {
  if (lhs.x < rhs.x) {
    return true;
  }

  if (rhs.x < lhs.x) {
    return false;
  }

  if (lhs.y < rhs.y) {
    return true;
  }

  if (rhs.y < lhs.y) {
    return false;
  }

  return lhs.z < rhs.z;
}

[[nodiscard]] auto cell_membership_less(const CellMembership& lhs, const CellMembership& rhs) -> bool {
  if (cell_coord_less(lhs.coord, rhs.coord)) {
    return true;
  }

  if (cell_coord_less(rhs.coord, lhs.coord)) {
    return false;
  }

  if (lhs.id < rhs.id) {
    return true;
  }

  if (rhs.id < lhs.id) {
    return false;
  }

  return lhs.body_index < rhs.body_index;
}

[[nodiscard]] auto same_cell(const CellCoord& lhs, const CellCoord& rhs) -> bool {
  return lhs == rhs;
}

[[nodiscard]] auto broadphase_overlap(const BodyProxy& lhs, const BodyProxy& rhs) -> bool {
  const double radius_sum =
    rex::geometry::bounding_radius(lhs.shape) + rex::geometry::bounding_radius(rhs.shape);
  const rex::math::Vec3 delta = rhs.pose.translation - lhs.pose.translation;
  return rex::math::dot(delta, delta) <= (radius_sum * radius_sum);
}

[[nodiscard]] auto choose_cell_size(std::span<const BodyProxy> bodies) -> double {
  if (bodies.empty()) {
    return 1.0;
  }

  std::vector<double> diameters{};
  diameters.reserve(bodies.size());
  for (const BodyProxy& body : bodies) {
    diameters.push_back(rex::geometry::bounding_radius(body.shape) * 2.0);
  }

  auto median = diameters.begin() + static_cast<std::ptrdiff_t>(diameters.size() / 2);
  std::nth_element(diameters.begin(), median, diameters.end());
  return std::max(*median, 1.0e-3);
}

[[nodiscard]] auto cell_index(double value, double cell_size) -> std::int64_t {
  return static_cast<std::int64_t>(std::floor(value / cell_size));
}

[[nodiscard]] auto axis_cell_span(std::int64_t min_index, std::int64_t max_index) -> std::size_t {
  return static_cast<std::size_t>((max_index - min_index) + 1);
}

[[nodiscard]] auto use_overflow_path(const BroadphaseBounds& bounds, double cell_size) -> bool {
  constexpr std::size_t kMaxCellsPerAxis = 4;
  constexpr std::size_t kMaxCellsPerBody = 64;

  const std::int64_t min_x = cell_index(bounds.min.x, cell_size);
  const std::int64_t max_x = cell_index(bounds.max.x, cell_size);
  const std::int64_t min_y = cell_index(bounds.min.y, cell_size);
  const std::int64_t max_y = cell_index(bounds.max.y, cell_size);
  const std::int64_t min_z = cell_index(bounds.min.z, cell_size);
  const std::int64_t max_z = cell_index(bounds.max.z, cell_size);

  const std::size_t span_x = axis_cell_span(min_x, max_x);
  const std::size_t span_y = axis_cell_span(min_y, max_y);
  const std::size_t span_z = axis_cell_span(min_z, max_z);
  return span_x > kMaxCellsPerAxis ||
         span_y > kMaxCellsPerAxis ||
         span_z > kMaxCellsPerAxis ||
         (span_x * span_y * span_z) > kMaxCellsPerBody;
}

void append_grid_memberships(
  std::vector<CellMembership>& memberships,
  const BodyProxy& body,
  std::size_t body_index,
  const BroadphaseBounds& bounds,
  double cell_size) {
  const std::int64_t min_x = cell_index(bounds.min.x, cell_size);
  const std::int64_t max_x = cell_index(bounds.max.x, cell_size);
  const std::int64_t min_y = cell_index(bounds.min.y, cell_size);
  const std::int64_t max_y = cell_index(bounds.max.y, cell_size);
  const std::int64_t min_z = cell_index(bounds.min.z, cell_size);
  const std::int64_t max_z = cell_index(bounds.max.z, cell_size);

  for (std::int64_t x = min_x; x <= max_x; ++x) {
    for (std::int64_t y = min_y; y <= max_y; ++y) {
      for (std::int64_t z = min_z; z <= max_z; ++z) {
        memberships.push_back({
          .coord = {x, y, z},
          .id = body.id,
          .body_index = body_index,
        });
      }
    }
  }
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
  const rex::math::Vec3& local,
  const rex::geometry::Box& box,
  const rex::math::Vec3& surface_normal) -> rex::math::Vec3 {
  if (surface_normal.x != 0.0) {
    return {
      surface_normal.x * box.half_extents.x,
      local.y,
      local.z,
    };
  }

  if (surface_normal.y != 0.0) {
    return {
      local.x,
      surface_normal.y * box.half_extents.y,
      local.z,
    };
  }

  return {
    local.x,
    local.y,
    surface_normal.z * box.half_extents.z,
  };
}

[[nodiscard]] auto make_oriented_box(const BodyProxy& body, const rex::geometry::Box& box) -> OrientedBox {
  return {
    .center = body.pose.translation,
    .axes = {
      rex::math::basis_x(body.pose.rotation),
      rex::math::basis_y(body.pose.rotation),
      rex::math::basis_z(body.pose.rotation),
    },
    .half_extents = {
      box.half_extents.x,
      box.half_extents.y,
      box.half_extents.z,
    },
  };
}

[[nodiscard]] auto projected_radius(const OrientedBox& box, const rex::math::Vec3& axis) -> double {
  double radius = 0.0;
  for (std::size_t axis_index = 0; axis_index < 3; ++axis_index) {
    radius += box.half_extents[axis_index] * std::abs(rex::math::dot(box.axes[axis_index], axis));
  }

  return radius;
}

[[nodiscard]] auto overlap_on_axis(
  const OrientedBox& lhs,
  const OrientedBox& rhs,
  const rex::math::Vec3& axis,
  const rex::math::Vec3& center_delta,
  double& min_penetration,
  rex::math::Vec3& best_normal) -> bool {
  const double axis_length = rex::math::norm(axis);
  if (axis_length < 1.0e-9) {
    return true;
  }

  const rex::math::Vec3 unit_axis = axis / axis_length;
  double projection_lhs = 0.0;
  double projection_rhs = 0.0;
  for (std::size_t axis_index = 0; axis_index < 3; ++axis_index) {
    projection_lhs += lhs.half_extents[axis_index] * std::abs(rex::math::dot(lhs.axes[axis_index], unit_axis));
    projection_rhs += rhs.half_extents[axis_index] * std::abs(rex::math::dot(rhs.axes[axis_index], unit_axis));
  }

  const double center_distance = std::abs(rex::math::dot(center_delta, unit_axis));
  const double penetration = projection_lhs + projection_rhs - center_distance;
  if (penetration <= 0.0) {
    return false;
  }

  if (penetration < min_penetration) {
    min_penetration = penetration;
    best_normal = rex::math::dot(center_delta, unit_axis) >= 0.0 ? unit_axis : -unit_axis;
  }

  return true;
}

[[nodiscard]] auto contact_point_from_reference_face(
  const OrientedBox& lhs,
  const OrientedBox& rhs,
  const rex::math::Vec3& normal) -> rex::math::Vec3 {
  const OrientedBox& reference_box =
    std::max({std::abs(rex::math::dot(lhs.axes[0], normal)),
              std::abs(rex::math::dot(lhs.axes[1], normal)),
              std::abs(rex::math::dot(lhs.axes[2], normal))}) >=
      std::max({std::abs(rex::math::dot(rhs.axes[0], normal)),
                std::abs(rex::math::dot(rhs.axes[1], normal)),
                std::abs(rex::math::dot(rhs.axes[2], normal))})
      ? lhs
      : rhs;
  const bool reference_is_lhs = &reference_box == &lhs;
  const rex::math::Vec3 reference_face_normal = reference_is_lhs ? normal : -normal;

  std::size_t normal_axis_index = 0;
  double best_alignment = std::abs(rex::math::dot(reference_box.axes[0], reference_face_normal));
  for (std::size_t axis_index = 1; axis_index < 3; ++axis_index) {
    const double alignment = std::abs(rex::math::dot(reference_box.axes[axis_index], reference_face_normal));
    if (alignment > best_alignment) {
      best_alignment = alignment;
      normal_axis_index = axis_index;
    }
  }

  const rex::math::Vec3 axis_n = reference_box.axes[normal_axis_index];
  const rex::math::Vec3 axis_t1 = reference_box.axes[(normal_axis_index + 1) % 3];
  const rex::math::Vec3 axis_t2 = reference_box.axes[(normal_axis_index + 2) % 3];
  const double normal_sign = rex::math::dot(axis_n, reference_face_normal) >= 0.0 ? 1.0 : -1.0;

  const double coordinate_n =
    rex::math::dot(reference_box.center, axis_n) + (normal_sign * reference_box.half_extents[normal_axis_index]);

  const auto overlap_center_on_axis = [&](const rex::math::Vec3& axis) {
    const double lhs_center = rex::math::dot(lhs.center, axis);
    const double rhs_center = rex::math::dot(rhs.center, axis);
    const double lhs_radius = projected_radius(lhs, axis);
    const double rhs_radius = projected_radius(rhs, axis);
    const double lower = std::max(lhs_center - lhs_radius, rhs_center - rhs_radius);
    const double upper = std::min(lhs_center + lhs_radius, rhs_center + rhs_radius);
    return 0.5 * (lower + upper);
  };

  const double coordinate_t1 = overlap_center_on_axis(axis_t1);
  const double coordinate_t2 = overlap_center_on_axis(axis_t2);
  return (axis_n * coordinate_n) + (axis_t1 * coordinate_t1) + (axis_t2 * coordinate_t2);
}

void assign_contact_features(
  const BodyProxy& body_a,
  const BodyProxy& body_b,
  ContactPoint& point,
  const rex::math::Vec3& anchor_world_a,
  const rex::math::Vec3& anchor_world_b,
  const rex::math::Vec3& normal_world) {
  point.local_anchor_a = rex::math::inverse_transform_point(body_a.pose, anchor_world_a);
  point.local_anchor_b = rex::math::inverse_transform_point(body_b.pose, anchor_world_b);
  point.local_normal_a = rex::math::inverse_rotate(body_a.pose.rotation, normal_world);
  point.local_normal_b = rex::math::inverse_rotate(body_b.pose.rotation, -normal_world);
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
  assign_contact_features(
    lhs,
    rhs,
    manifold.points[0],
    lhs.pose.translation + (normal * lhs_sphere->radius),
    rhs.pose.translation - (normal * rhs_sphere->radius),
    normal);
  return manifold;
}

[[nodiscard]] auto build_sphere_box_contact(
  const BodyProxy& sphere_body,
  const rex::geometry::Sphere& sphere,
  const BodyProxy& box_body,
  const rex::geometry::Box& box,
  bool sphere_is_body_a) -> std::optional<ContactManifold> {
  const rex::math::Vec3 sphere_center = sphere_body.pose.translation;
  const rex::math::Vec3 local_center = rex::math::inverse_transform_point(box_body.pose, sphere_center);
  const rex::math::Vec3 clamped_local = clamp_to_box(local_center, box);

  rex::math::Vec3 surface_point = rex::math::transform_point(box_body.pose, clamped_local);
  rex::math::Vec3 box_to_sphere = sphere_center - surface_point;
  double center_distance = rex::math::norm(box_to_sphere);
  double penetration = sphere.radius - center_distance;

  if (center_distance == 0.0) {
    const rex::math::Vec3 local_surface_normal = box_surface_normal(local_center, box);
    box_to_sphere = rex::math::rotate(box_body.pose.rotation, local_surface_normal);
    surface_point = rex::math::transform_point(
      box_body.pose,
      box_surface_point(local_center, box, local_surface_normal));
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
  const rex::math::Vec3 sphere_anchor =
    sphere_center + (manifold.points[0].normal * sphere.radius * (sphere_is_body_a ? 1.0 : -1.0));
  if (sphere_is_body_a) {
    assign_contact_features(
      sphere_body,
      box_body,
      manifold.points[0],
      sphere_anchor,
      surface_point,
      manifold.points[0].normal);
  } else {
    assign_contact_features(
      box_body,
      sphere_body,
      manifold.points[0],
      surface_point,
      sphere_anchor,
      manifold.points[0].normal);
  }
  return manifold;
}

[[nodiscard]] auto build_box_box_contact(
  const BodyProxy& lhs,
  const rex::geometry::Box& lhs_box,
  const BodyProxy& rhs,
  const rex::geometry::Box& rhs_box) -> std::optional<ContactManifold> {
  const OrientedBox lhs_obb = make_oriented_box(lhs, lhs_box);
  const OrientedBox rhs_obb = make_oriented_box(rhs, rhs_box);
  const rex::math::Vec3 delta = rhs_obb.center - lhs_obb.center;

  double penetration = std::numeric_limits<double>::infinity();
  rex::math::Vec3 normal{};

  for (std::size_t axis_index = 0; axis_index < 3; ++axis_index) {
    if (!overlap_on_axis(lhs_obb, rhs_obb, lhs_obb.axes[axis_index], delta, penetration, normal)) {
      return std::nullopt;
    }
    if (!overlap_on_axis(lhs_obb, rhs_obb, rhs_obb.axes[axis_index], delta, penetration, normal)) {
      return std::nullopt;
    }
  }

  for (std::size_t lhs_axis = 0; lhs_axis < 3; ++lhs_axis) {
    for (std::size_t rhs_axis = 0; rhs_axis < 3; ++rhs_axis) {
      if (!overlap_on_axis(
            lhs_obb,
            rhs_obb,
            rex::math::cross(lhs_obb.axes[lhs_axis], rhs_obb.axes[rhs_axis]),
            delta,
            penetration,
            normal)) {
        return std::nullopt;
      }
    }
  }

  ContactManifold manifold{};
  manifold.body_a = lhs.id;
  manifold.body_b = rhs.id;
  manifold.point_count = 1;
  manifold.points[0].normal = normal;
  manifold.points[0].penetration = penetration;
  manifold.points[0].position = contact_point_from_reference_face(lhs_obb, rhs_obb, normal);
  assign_contact_features(
    lhs,
    rhs,
    manifold.points[0],
    manifold.points[0].position + (normal * (penetration * 0.5)),
    manifold.points[0].position - (normal * (penetration * 0.5)),
    normal);
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
  const double cell_size = choose_cell_size(bodies);
  std::vector<CellMembership> memberships{};
  std::vector<std::size_t> overflow_body_indices{};
  memberships.reserve(bodies.size() * 4);
  overflow_body_indices.reserve(bodies.size());

  for (std::size_t body_index = 0; body_index < bodies.size(); ++body_index) {
    const BroadphaseBounds body_bounds = broadphase_bounds(bodies[body_index]);
    if (use_overflow_path(body_bounds, cell_size)) {
      overflow_body_indices.push_back(body_index);
      continue;
    }

    append_grid_memberships(memberships, bodies[body_index], body_index, body_bounds, cell_size);
  }

  std::sort(memberships.begin(), memberships.end(), cell_membership_less);

  std::vector<CandidatePair> candidates{};
  candidates.reserve(bodies.size() * 2);

  std::size_t begin = 0;
  while (begin < memberships.size()) {
    std::size_t end = begin + 1;
    while (end < memberships.size() && same_cell(memberships[begin].coord, memberships[end].coord)) {
      ++end;
    }

    for (std::size_t lhs_index = begin; lhs_index < end; ++lhs_index) {
      for (std::size_t rhs_index = lhs_index + 1; rhs_index < end; ++rhs_index) {
        const std::size_t body_index_a = memberships[lhs_index].body_index;
        const std::size_t body_index_b = memberships[rhs_index].body_index;
        if (!broadphase_overlap(bodies[body_index_a], bodies[body_index_b])) {
          continue;
        }

        candidates.push_back(make_pair(
          bodies[body_index_a].id,
          bodies[body_index_b].id,
          body_index_a,
          body_index_b));
      }
    }

    begin = end;
  }

  // Bodies that span many cells stay on a small overflow path so large static geometry does not
  // explode grid occupancy for everyone else.
  for (std::size_t overflow_body_index : overflow_body_indices) {
    for (std::size_t other_body_index = 0; other_body_index < bodies.size(); ++other_body_index) {
      if (overflow_body_index == other_body_index) {
        continue;
      }

      if (!broadphase_overlap(bodies[overflow_body_index], bodies[other_body_index])) {
        continue;
      }

      candidates.push_back(make_pair(
        bodies[overflow_body_index].id,
        bodies[other_body_index].id,
        overflow_body_index,
        other_body_index));
    }
  }

  std::sort(candidates.begin(), candidates.end(), candidate_pair_less);
  candidates.erase(
    std::unique(candidates.begin(), candidates.end(), candidate_pair_equal),
    candidates.end());
  return candidates;
}

}  // namespace

auto build_contact_manifold(const BodyProxy& body_a, const BodyProxy& body_b)
  -> std::optional<ContactManifold> {
  if (body_b.id < body_a.id) {
    return build_contact(body_b, body_a);
  }

  return build_contact(body_a, body_b);
}

auto build_frame(
  std::span<const BodyProxy> bodies,
  std::span<const ContactManifold> previous_manifolds,
  const CollisionPipelineConfig& config) -> CollisionFrame {
  const std::vector<CandidatePair> candidates = build_broadphase_pairs(bodies);
  std::unordered_map<PairKey, const ContactManifold*, PairKeyHash> previous_lookup{};
  if (config.enable_persistent_manifolds) {
    previous_lookup.reserve(previous_manifolds.size());
    for (const ContactManifold& manifold : previous_manifolds) {
      previous_lookup.emplace(pair_key({manifold.body_a, manifold.body_b}), &manifold);
    }
  }

  CollisionFrame frame{};
  frame.broadphase_pairs.reserve(candidates.size());
  frame.manifolds.reserve(candidates.size());

  for (const CandidatePair& candidate : candidates) {
    frame.broadphase_pairs.push_back(candidate.pair);

    auto maybe_manifold = build_contact_manifold(
      bodies[candidate.body_index_a],
      bodies[candidate.body_index_b]);
    if (!maybe_manifold.has_value()) {
      continue;
    }

    if (config.enable_persistent_manifolds) {
      const auto previous_it = previous_lookup.find(pair_key(candidate.pair));
      if (previous_it != previous_lookup.end()) {
        const ContactManifold& previous = *previous_it->second;
        const std::size_t persistent_points =
          std::min(previous.point_count, maybe_manifold->point_count);
        for (std::size_t point_index = 0; point_index < persistent_points; ++point_index) {
          maybe_manifold->points[point_index].cached_normal_impulse =
            previous.points[point_index].cached_normal_impulse;
          maybe_manifold->points[point_index].cached_tangent_impulse_u =
            previous.points[point_index].cached_tangent_impulse_u;
          maybe_manifold->points[point_index].cached_tangent_impulse_v =
            previous.points[point_index].cached_tangent_impulse_v;
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
