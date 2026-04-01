#include "rex/geometry/shapes.hpp"

#include <type_traits>

namespace rex::geometry {

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

}  // namespace rex::geometry
