#pragma once

#include <string>

#include "rex/dynamics/world.hpp"
#include "rex/solver/solver.hpp"

namespace rex::sim {

struct EngineConfig {
  rex::dynamics::SimulationConfig simulation{};
};

struct StepTrace {
  std::size_t body_count{0};
  std::size_t articulation_count{0};
  rex::solver::SolverResult solver{};
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

