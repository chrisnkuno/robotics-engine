#include "rex/kinematics/articulation.hpp"

namespace rex::kinematics {

void Articulation::add_link(LinkSpec link) {
  links_.push_back(link);
}

void Articulation::add_joint(JointSpec joint) {
  joints_.push_back(joint);
}

auto Articulation::root() const noexcept -> rex::platform::EntityId {
  return root_;
}

auto Articulation::links() const noexcept -> const std::vector<LinkSpec>& {
  return links_;
}

auto Articulation::joints() const noexcept -> const std::vector<JointSpec>& {
  return joints_;
}

}  // namespace rex::kinematics

