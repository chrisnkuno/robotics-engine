#include "rex/kinematics/articulation.hpp"

#include <stdexcept>

namespace rex::kinematics {

namespace {

[[nodiscard]] auto joint_dof_count(JointType type) -> std::size_t {
  switch (type) {
    case JointType::kFixed:
      return 0;
    case JointType::kRevolute:
    case JointType::kPrismatic:
      return 1;
    case JointType::kSpherical:
      throw std::runtime_error("spherical joints are not implemented yet");
  }

  return 0;
}

[[nodiscard]] auto motion_from_joint(
  const JointSpec& joint,
  std::span<const double> joint_positions,
  std::size_t& coordinate_index) -> rex::math::Transform {
  switch (joint.type) {
    case JointType::kFixed:
      return rex::math::Transform::identity();
    case JointType::kRevolute: {
      const double angle = joint_positions[coordinate_index++];
      return {
        .rotation = rex::math::quat_from_axis_angle(joint.axis, angle),
        .translation = {},
      };
    }
    case JointType::kPrismatic: {
      const double displacement = joint_positions[coordinate_index++];
      return {
        .rotation = {},
        .translation = rex::math::normalized_or(joint.axis) * displacement,
      };
    }
    case JointType::kSpherical:
      throw std::runtime_error("spherical joints are not implemented yet");
  }

  return rex::math::Transform::identity();
}

[[nodiscard]] auto find_link_index(
  const Articulation& articulation,
  rex::platform::EntityId body) -> std::size_t {
  for (std::size_t index = 0; index < articulation.links().size(); ++index) {
    if (articulation.links()[index].body == body) {
      return index;
    }
  }

  throw std::runtime_error("body not found in articulation");
}

[[nodiscard]] auto joint_influences_body(
  const Articulation& articulation,
  rex::platform::EntityId joint_child,
  rex::platform::EntityId body) -> bool {
  if (joint_child == body) {
    return true;
  }

  for (const JointSpec& candidate : articulation.joints()) {
    if (candidate.parent == joint_child && joint_influences_body(articulation, candidate.child, body)) {
      return true;
    }
  }

  return false;
}

}  // namespace

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

auto degrees_of_freedom(const Articulation& articulation) -> std::size_t {
  std::size_t count = 0;
  for (const JointSpec& joint : articulation.joints()) {
    count += joint_dof_count(joint.type);
  }

  return count;
}

auto forward_kinematics(
  const Articulation& articulation,
  std::span<const double> joint_positions) -> std::vector<rex::math::Transform> {
  if (joint_positions.size() != degrees_of_freedom(articulation)) {
    throw std::runtime_error("joint position vector has the wrong length");
  }

  std::vector<rex::math::Transform> world_from_link(
    articulation.links().size(),
    rex::math::Transform::identity());

  if (articulation.links().empty()) {
    return world_from_link;
  }

  world_from_link[find_link_index(articulation, articulation.root())] = rex::math::Transform::identity();

  std::size_t coordinate_index = 0;
  for (const JointSpec& joint : articulation.joints()) {
    const std::size_t parent_index = find_link_index(articulation, joint.parent);
    const std::size_t child_index = find_link_index(articulation, joint.child);
    const rex::math::Transform world_from_joint =
      rex::math::compose(world_from_link[parent_index], joint.parent_from_joint);
    const rex::math::Transform joint_motion = motion_from_joint(joint, joint_positions, coordinate_index);
    world_from_link[child_index] =
      rex::math::compose(rex::math::compose(world_from_joint, joint_motion), rex::math::inverse(joint.child_from_joint));
  }

  return world_from_link;
}

auto translational_jacobian(
  const Articulation& articulation,
  std::span<const double> joint_positions,
  rex::platform::EntityId body,
  const rex::math::Vec3& point_in_body) -> std::vector<rex::math::Vec3> {
  const std::vector<rex::math::Transform> world_from_link = forward_kinematics(articulation, joint_positions);
  std::vector<rex::math::Vec3> jacobian(
    degrees_of_freedom(articulation),
    rex::math::Vec3{});
  if (jacobian.empty()) {
    return jacobian;
  }

  const std::size_t body_index = find_link_index(articulation, body);
  const rex::math::Vec3 world_point = rex::math::transform_point(world_from_link[body_index], point_in_body);

  std::size_t coordinate_index = 0;
  for (const JointSpec& joint : articulation.joints()) {
    const std::size_t dof_count = joint_dof_count(joint.type);
    const std::size_t parent_index = find_link_index(articulation, joint.parent);
    const rex::math::Transform world_from_joint =
      rex::math::compose(world_from_link[parent_index], joint.parent_from_joint);
    const rex::math::Vec3 axis_world = rex::math::rotate(world_from_joint.rotation, rex::math::normalized_or(joint.axis));

    const bool influences_body = joint_influences_body(articulation, joint.child, body);

    if (dof_count == 0) {
      continue;
    }

    if (influences_body) {
      switch (joint.type) {
        case JointType::kRevolute:
          jacobian[coordinate_index] = rex::math::cross(axis_world, world_point - world_from_joint.translation);
          break;
        case JointType::kPrismatic:
          jacobian[coordinate_index] = axis_world;
          break;
        case JointType::kFixed:
          break;
        case JointType::kSpherical:
          throw std::runtime_error("spherical joints are not implemented yet");
      }
    }

    coordinate_index += dof_count;
  }

  return jacobian;
}

}  // namespace rex::kinematics
