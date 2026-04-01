#pragma once

#include <variant>

#include "rex/math/types.hpp"

namespace rex::geometry {

struct Sphere {
  double radius{0.5};
};

struct Box {
  rex::math::Vec3 half_extents{0.5, 0.5, 0.5};
};

using ShapeData = std::variant<Sphere, Box>;

struct Shape {
  ShapeData data{Sphere{}};
};

[[nodiscard]] auto bounding_radius(const Shape& shape) -> double;
[[nodiscard]] auto inverse_inertia_body(const Shape& shape, double inverse_mass) -> rex::math::Vec3;
[[nodiscard]] auto apply_inverse_inertia(
  const Shape& shape,
  const rex::math::Quat& rotation,
  double inverse_mass,
  const rex::math::Vec3& value) -> rex::math::Vec3;

}  // namespace rex::geometry
