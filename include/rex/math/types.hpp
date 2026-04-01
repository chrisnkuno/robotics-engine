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

  [[nodiscard]] constexpr auto operator/(double scalar) const noexcept -> Vec3 {
    return {x / scalar, y / scalar, z / scalar};
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

[[nodiscard]] constexpr auto cross(const Vec3& lhs, const Vec3& rhs) noexcept -> Vec3 {
  return {
    lhs.y * rhs.z - lhs.z * rhs.y,
    lhs.z * rhs.x - lhs.x * rhs.z,
    lhs.x * rhs.y - lhs.y * rhs.x,
  };
}

[[nodiscard]] inline auto norm(const Vec3& value) noexcept -> double {
  return std::sqrt(dot(value, value));
}

[[nodiscard]] inline auto distance(const Vec3& lhs, const Vec3& rhs) noexcept -> double {
  return norm(lhs - rhs);
}

[[nodiscard]] inline auto normalized_or(const Vec3& value, Vec3 fallback = {0.0, 1.0, 0.0}) noexcept
  -> Vec3 {
  const double length = norm(value);
  if (length == 0.0) {
    return fallback;
  }

  return value / length;
}

}  // namespace rex::math
