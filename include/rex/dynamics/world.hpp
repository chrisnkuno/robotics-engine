#pragma once

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
  std::vector<BodyState> bodies{};
  std::vector<rex::kinematics::Articulation> articulations{};
  std::vector<rex::collision::ContactManifold> contact_manifolds{};
};

}  // namespace rex::dynamics

