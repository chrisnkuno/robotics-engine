#pragma once

#include <cstddef>
#include <span>
#include <string>
#include <vector>

#include "rex/collision/contact.hpp"
#include "rex/math/types.hpp"

namespace rex::solver {

enum class SolverType {
  kSequentialImpulsePgs,
};

enum class FrictionModel {
  kPyramid,
};

enum class StabilizationMode {
  kSplitImpulse,
};

struct SolverConfig {
  SolverType type{SolverType::kSequentialImpulsePgs};
  FrictionModel friction_model{FrictionModel::kPyramid};
  StabilizationMode stabilization{StabilizationMode::kSplitImpulse};
  std::size_t velocity_iterations{12};
  std::size_t position_iterations{2};
  bool warm_start{true};
  bool deterministic_ordering{true};
};

struct SolverResult {
  std::size_t contact_count{0};
  std::size_t constraint_count{0};
  double max_penetration{0.0};
};

enum class ConstraintAxis {
  kNormal,
  kTangentU,
  kTangentV,
};

struct ConstraintRow {
  rex::platform::EntityId body_a{};
  rex::platform::EntityId body_b{};
  rex::math::Vec3 direction{};
  ConstraintAxis axis{ConstraintAxis::kNormal};
  double lower_impulse_limit{0.0};
  double upper_impulse_limit{0.0};
};

struct ConstraintAssembly {
  std::vector<ConstraintRow> rows{};
};

[[nodiscard]] auto assemble_contact_rows(std::span<const rex::collision::ContactManifold> manifolds)
  -> ConstraintAssembly;
[[nodiscard]] auto describe(const SolverConfig& config) -> std::string;

}  // namespace rex::solver
