#pragma once

#include <cmath>

namespace rex::math {

struct Vec3 {
  double x{0.0};
  double y{0.0};
  double z{0.0};

  [[nodiscard]] constexpr auto operator+(const Vec3& rhs) const noexcept -> Vec3 {
    return {x + rhs.x, y + rhs.y, z + rhs.z};
  }

  [[nodiscard]] constexpr auto operator-(const Vec3& rhs) const noexcept -> Vec3 {
    return {x - rhs.x, y - rhs.y, z - rhs.z};
  }

  [[nodiscard]] constexpr auto operator*(double scalar) const noexcept -> Vec3 {
    return {x * scalar, y * scalar, z * scalar};
  }

  constexpr auto operator+=(const Vec3& rhs) noexcept -> Vec3& {
    x += rhs.x;
    y += rhs.y;
    z += rhs.z;
    return *this;
  }
};

struct Quat {
  double w{1.0};
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct Transform {
  Quat rotation{};
  Vec3 translation{};

  [[nodiscard]] static constexpr auto identity() noexcept -> Transform {
    return {};
  }
};

[[nodiscard]] constexpr auto dot(const Vec3& lhs, const Vec3& rhs) noexcept -> double {
  return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
}

[[nodiscard]] inline auto norm(const Vec3& value) noexcept -> double {
  return std::sqrt(dot(value, value));
}

}  // namespace rex::math

