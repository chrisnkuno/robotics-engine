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

}  // namespace rex::geometry

