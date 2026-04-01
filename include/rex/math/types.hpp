#pragma once

#include <cmath>

namespace rex::math {

struct Vec3 {
  double x{0.0};
  double y{0.0};
  double z{0.0};

  [[nodiscard]] constexpr auto operator-() const noexcept -> Vec3 {
    return {-x, -y, -z};
  }

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

  constexpr auto operator-=(const Vec3& rhs) noexcept -> Vec3& {
    x -= rhs.x;
    y -= rhs.y;
    z -= rhs.z;
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

[[nodiscard]] inline auto conjugate(const Quat& value) noexcept -> Quat {
  return {
    .w = value.w,
    .x = -value.x,
    .y = -value.y,
    .z = -value.z,
  };
}

[[nodiscard]] constexpr auto multiply(const Quat& lhs, const Quat& rhs) noexcept -> Quat {
  return {
    .w = (lhs.w * rhs.w) - (lhs.x * rhs.x) - (lhs.y * rhs.y) - (lhs.z * rhs.z),
    .x = (lhs.w * rhs.x) + (lhs.x * rhs.w) + (lhs.y * rhs.z) - (lhs.z * rhs.y),
    .y = (lhs.w * rhs.y) - (lhs.x * rhs.z) + (lhs.y * rhs.w) + (lhs.z * rhs.x),
    .z = (lhs.w * rhs.z) + (lhs.x * rhs.y) - (lhs.y * rhs.x) + (lhs.z * rhs.w),
  };
}

[[nodiscard]] inline auto normalized_or(const Quat& value, Quat fallback = {}) noexcept -> Quat {
  const double length =
    std::sqrt((value.w * value.w) + (value.x * value.x) + (value.y * value.y) + (value.z * value.z));
  if (length == 0.0) {
    return fallback;
  }

  return {
    .w = value.w / length,
    .x = value.x / length,
    .y = value.y / length,
    .z = value.z / length,
  };
}

[[nodiscard]] inline auto quat_from_axis_angle(const Vec3& axis, double radians) noexcept -> Quat {
  const Vec3 direction = normalized_or(axis);
  const double half = radians * 0.5;
  const double s = std::sin(half);
  return normalized_or({
    .w = std::cos(half),
    .x = direction.x * s,
    .y = direction.y * s,
    .z = direction.z * s,
  });
}

[[nodiscard]] inline auto integrate_rotation(
  const Quat& rotation,
  const Vec3& angular_velocity,
  double dt) noexcept -> Quat {
  const double angle = norm(angular_velocity) * dt;
  if (angle == 0.0) {
    return normalized_or(rotation);
  }

  const Quat delta = quat_from_axis_angle(angular_velocity, angle);
  return normalized_or(multiply(delta, rotation));
}

[[nodiscard]] inline auto rotate(const Quat& rotation, const Vec3& value) noexcept -> Vec3 {
  const Quat q = normalized_or(rotation);
  const Quat pure{
    .w = 0.0,
    .x = value.x,
    .y = value.y,
    .z = value.z,
  };
  const Quat rotated = multiply(multiply(q, pure), conjugate(q));
  return {rotated.x, rotated.y, rotated.z};
}

[[nodiscard]] inline auto inverse_rotate(const Quat& rotation, const Vec3& value) noexcept -> Vec3 {
  return rotate(conjugate(normalized_or(rotation)), value);
}

[[nodiscard]] inline auto transform_point(const Transform& transform, const Vec3& value) noexcept -> Vec3 {
  return rotate(transform.rotation, value) + transform.translation;
}

[[nodiscard]] inline auto inverse_transform_point(const Transform& transform, const Vec3& value) noexcept -> Vec3 {
  return inverse_rotate(transform.rotation, value - transform.translation);
}

[[nodiscard]] inline auto compose(const Transform& parent, const Transform& child) noexcept -> Transform {
  return {
    .rotation = normalized_or(multiply(parent.rotation, child.rotation)),
    .translation = transform_point(parent, child.translation),
  };
}

[[nodiscard]] inline auto inverse(const Transform& transform) noexcept -> Transform {
  const Quat inverse_rotation = conjugate(normalized_or(transform.rotation));
  return {
    .rotation = inverse_rotation,
    .translation = rotate(inverse_rotation, -transform.translation),
  };
}

[[nodiscard]] inline auto basis_x(const Quat& rotation) noexcept -> Vec3 {
  return rotate(rotation, {1.0, 0.0, 0.0});
}

[[nodiscard]] inline auto basis_y(const Quat& rotation) noexcept -> Vec3 {
  return rotate(rotation, {0.0, 1.0, 0.0});
}

[[nodiscard]] inline auto basis_z(const Quat& rotation) noexcept -> Vec3 {
  return rotate(rotation, {0.0, 0.0, 1.0});
}

}  // namespace rex::math
