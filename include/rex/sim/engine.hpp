#pragma once

#include <cstddef>
#include <string>

#include "rex/dynamics/world.hpp"
#include "rex/solver/solver.hpp"

namespace rex::sim {

struct EngineConfig {
  rex::dynamics::SimulationConfig simulation{};
};

struct StepProfile {
  double integrate_ms{0.0};
  double collision_ms{0.0};
  double solver_ms{0.0};
  double total_ms{0.0};
};

struct StepTrace {
  std::size_t body_count{0};
  std::size_t articulation_count{0};
  std::size_t broadphase_pair_count{0};
  std::size_t manifold_count{0};
  rex::solver::SolverResult solver{};
  StepProfile profile{};
  std::string pipeline_summary{};
};

class Engine {
 public:
  explicit Engine(EngineConfig config = {});

  [[nodiscard]] auto config() const noexcept -> const EngineConfig&;
  [[nodiscard]] auto step(rex::dynamics::WorldState& world) const -> StepTrace;

 private:
  EngineConfig config_{};
};

}  // namespace rex::sim
