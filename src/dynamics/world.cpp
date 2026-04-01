#include "rex/dynamics/world.hpp"

namespace rex::dynamics {

void BodyStorage::reserve(std::size_t capacity) {
  ids_.reserve(capacity);
  poses_.reserve(capacity);
  linear_velocities_.reserve(capacity);
  angular_velocities_.reserve(capacity);
  inverse_masses_.reserve(capacity);
  shapes_.reserve(capacity);
}

auto BodyStorage::size() const noexcept -> std::size_t {
  return ids_.size();
}

auto BodyStorage::empty() const noexcept -> bool {
  return ids_.empty();
}

auto BodyStorage::add_body(const BodyState& state) -> std::size_t {
  ids_.push_back(state.id);
  poses_.push_back(state.pose);
  linear_velocities_.push_back(state.linear_velocity);
  angular_velocities_.push_back(state.angular_velocity);
  inverse_masses_.push_back(state.inverse_mass);
  shapes_.push_back(state.shape);
  return ids_.size() - 1;
}

auto BodyStorage::state(std::size_t index) const -> BodyState {
  return {
    .id = ids_[index],
    .pose = poses_[index],
    .linear_velocity = linear_velocities_[index],
    .angular_velocity = angular_velocities_[index],
    .inverse_mass = inverse_masses_[index],
    .shape = shapes_[index],
  };
}

auto BodyStorage::id(std::size_t index) const noexcept -> rex::platform::EntityId {
  return ids_[index];
}

auto BodyStorage::pose(std::size_t index) const noexcept -> const rex::math::Transform& {
  return poses_[index];
}

auto BodyStorage::linear_velocity(std::size_t index) const noexcept -> const rex::math::Vec3& {
  return linear_velocities_[index];
}

auto BodyStorage::angular_velocity(std::size_t index) const noexcept -> const rex::math::Vec3& {
  return angular_velocities_[index];
}

auto BodyStorage::inverse_mass(std::size_t index) const noexcept -> double {
  return inverse_masses_[index];
}

auto BodyStorage::pose_mut(std::size_t index) noexcept -> rex::math::Transform& {
  return poses_[index];
}

auto BodyStorage::linear_velocity_mut(std::size_t index) noexcept -> rex::math::Vec3& {
  return linear_velocities_[index];
}

auto BodyStorage::angular_velocity_mut(std::size_t index) noexcept -> rex::math::Vec3& {
  return angular_velocities_[index];
}

auto build_collision_proxies(const BodyStorage& bodies) -> std::vector<rex::collision::BodyProxy> {
  std::vector<rex::collision::BodyProxy> proxies{};
  proxies.reserve(bodies.size());

  for (std::size_t body_index = 0; body_index < bodies.size(); ++body_index) {
    proxies.push_back({
      .id = bodies.ids_[body_index],
      .pose = bodies.poses_[body_index],
      .shape = bodies.shapes_[body_index],
    });
  }

  return proxies;
}

void integrate_unconstrained(BodyStorage& bodies, const SimulationConfig& config) {
  for (std::size_t body_index = 0; body_index < bodies.size(); ++body_index) {
    if (bodies.inverse_mass(body_index) <= 0.0) {
      continue;
    }

    auto& linear_velocity = bodies.linear_velocity_mut(body_index);
    linear_velocity += config.gravity * config.step.dt;
    auto& pose = bodies.pose_mut(body_index);
    pose.translation += linear_velocity * config.step.dt;
    pose.rotation =
      rex::math::integrate_rotation(pose.rotation, bodies.angular_velocity(body_index), config.step.dt);
  }
}

}  // namespace rex::dynamics
