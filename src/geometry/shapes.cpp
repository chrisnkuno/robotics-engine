#include "rex/geometry/shapes.hpp"

#include <type_traits>

namespace rex::geometry {

namespace {

[[nodiscard]] auto safe_inverse(double value) -> double {
  return value > 0.0 ? (1.0 / value) : 0.0;
}

}  // namespace

auto bounding_radius(const Shape& shape) -> double {
  return std::visit(
    [](const auto& primitive) -> double {
      using Primitive = std::decay_t<decltype(primitive)>;

      if constexpr (std::is_same_v<Primitive, Sphere>) {
        return primitive.radius;
      } else {
        return rex::math::norm(primitive.half_extents);
      }
    },
    shape.data);
}

auto inverse_inertia_body(const Shape& shape, double inverse_mass) -> rex::math::Vec3 {
  return std::visit(
    [inverse_mass](const auto& primitive) -> rex::math::Vec3 {
      using Primitive = std::decay_t<decltype(primitive)>;

      if (inverse_mass <= 0.0) {
        return {};
      }

      if constexpr (std::is_same_v<Primitive, Sphere>) {
        const double radius_squared = primitive.radius * primitive.radius;
        const double inverse_inertia = radius_squared > 0.0
          ? (2.5 * inverse_mass / radius_squared)
          : 0.0;
        return {inverse_inertia, inverse_inertia, inverse_inertia};
      } else {
        const double inverse_x =
          3.0 * inverse_mass * safe_inverse(
            (primitive.half_extents.y * primitive.half_extents.y) +
            (primitive.half_extents.z * primitive.half_extents.z));
        const double inverse_y =
          3.0 * inverse_mass * safe_inverse(
            (primitive.half_extents.x * primitive.half_extents.x) +
            (primitive.half_extents.z * primitive.half_extents.z));
        const double inverse_z =
          3.0 * inverse_mass * safe_inverse(
            (primitive.half_extents.x * primitive.half_extents.x) +
            (primitive.half_extents.y * primitive.half_extents.y));
        return {inverse_x, inverse_y, inverse_z};
      }
    },
    shape.data);
}

auto apply_inverse_inertia(
  const Shape& shape,
  const rex::math::Quat& rotation,
  double inverse_mass,
  const rex::math::Vec3& value) -> rex::math::Vec3 {
  const rex::math::Vec3 inverse_diagonal = inverse_inertia_body(shape, inverse_mass);
  if (inverse_diagonal.x == 0.0 && inverse_diagonal.y == 0.0 && inverse_diagonal.z == 0.0) {
    return {};
  }

  const rex::math::Vec3 local = rex::math::inverse_rotate(rotation, value);
  const rex::math::Vec3 local_result{
    local.x * inverse_diagonal.x,
    local.y * inverse_diagonal.y,
    local.z * inverse_diagonal.z,
  };
  return rex::math::rotate(rotation, local_result);
}

}  // namespace rex::geometry
