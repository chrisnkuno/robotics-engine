#pragma once

#include <vector>

#include "rex/math/types.hpp"
#include "rex/platform/handles.hpp"

namespace rex::kinematics {

enum class JointType {
  kFixed,
  kRevolute,
  kPrismatic,
  kSpherical,
};

struct LinkSpec {
  rex::platform::EntityId body{};
  double mass{1.0};
};

struct JointSpec {
  JointType type{JointType::kFixed};
  rex::platform::EntityId parent{};
  rex::platform::EntityId child{};
  rex::math::Transform parent_from_joint{};
  rex::math::Transform child_from_joint{};
  rex::math::Vec3 axis{0.0, 0.0, 1.0};
};

class Articulation {
 public:
  explicit Articulation(rex::platform::EntityId root = {}) : root_(root) {}

  void add_link(LinkSpec link);
  void add_joint(JointSpec joint);

  [[nodiscard]] auto root() const noexcept -> rex::platform::EntityId;
  [[nodiscard]] auto links() const noexcept -> const std::vector<LinkSpec>&;
  [[nodiscard]] auto joints() const noexcept -> const std::vector<JointSpec>&;

 private:
  rex::platform::EntityId root_{};
  std::vector<LinkSpec> links_{};
  std::vector<JointSpec> joints_{};
};

}  // namespace rex::kinematics

