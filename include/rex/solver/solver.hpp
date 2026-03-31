#pragma once

#include <cstddef>
#include <string>

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

[[nodiscard]] auto describe(const SolverConfig& config) -> std::string;

}  // namespace rex::solver

