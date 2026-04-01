#pragma once

#include <cstddef>
#include <vector>

#include "rex/collision/contact.hpp"
#include "rex/geometry/shapes.hpp"
#include "rex/kinematics/articulation.hpp"
#include "rex/math/types.hpp"
#include "rex/platform/handles.hpp"
#include "rex/solver/solver.hpp"

namespace rex::dynamics {

enum class Integrator {
  kSemiImplicitEuler,
};

struct BodyState {
  rex::platform::EntityId id{};
  rex::math::Transform pose{};
  rex::math::Vec3 linear_velocity{};
  rex::math::Vec3 angular_velocity{};
  double inverse_mass{1.0};
  rex::geometry::Shape shape{};
};

class BodyStorage {
 public:
  void reserve(std::size_t capacity);

  [[nodiscard]] auto size() const noexcept -> std::size_t;
  [[nodiscard]] auto empty() const noexcept -> bool;
  [[nodiscard]] auto add_body(const BodyState& state) -> std::size_t;
  [[nodiscard]] auto state(std::size_t index) const -> BodyState;

  [[nodiscard]] auto id(std::size_t index) const noexcept -> rex::platform::EntityId;
  [[nodiscard]] auto pose(std::size_t index) const noexcept -> const rex::math::Transform&;
  [[nodiscard]] auto linear_velocity(std::size_t index) const noexcept -> const rex::math::Vec3&;
  [[nodiscard]] auto angular_velocity(std::size_t index) const noexcept -> const rex::math::Vec3&;
  [[nodiscard]] auto inverse_mass(std::size_t index) const noexcept -> double;
  [[nodiscard]] auto shape(std::size_t index) const noexcept -> const rex::geometry::Shape&;
  [[nodiscard]] auto pose_mut(std::size_t index) noexcept -> rex::math::Transform&;
  [[nodiscard]] auto linear_velocity_mut(std::size_t index) noexcept -> rex::math::Vec3&;
  [[nodiscard]] auto angular_velocity_mut(std::size_t index) noexcept -> rex::math::Vec3&;

 private:
  std::vector<rex::platform::EntityId> ids_{};
  std::vector<rex::math::Transform> poses_{};
  std::vector<rex::math::Vec3> linear_velocities_{};
  std::vector<rex::math::Vec3> angular_velocities_{};
  std::vector<double> inverse_masses_{};
  std::vector<rex::geometry::Shape> shapes_{};

  friend auto build_collision_proxies(const BodyStorage& bodies) -> std::vector<rex::collision::BodyProxy>;
};

struct StepConfig {
  double dt{1.0 / 240.0};
  Integrator integrator{Integrator::kSemiImplicitEuler};
};

struct SimulationConfig {
  rex::math::Vec3 gravity{0.0, 0.0, -9.81};
  StepConfig step{};
  rex::collision::CollisionPipelineConfig collision{};
  rex::solver::SolverConfig solver{};
};

struct WorldState {
  BodyStorage bodies{};
  std::vector<rex::kinematics::Articulation> articulations{};
  std::vector<rex::collision::ContactManifold> contact_manifolds{};
};

[[nodiscard]] auto build_collision_proxies(const BodyStorage& bodies) -> std::vector<rex::collision::BodyProxy>;
void integrate_unconstrained(BodyStorage& bodies, const SimulationConfig& config);

}  // namespace rex::dynamics
