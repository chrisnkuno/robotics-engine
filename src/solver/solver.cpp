#include "rex/solver/solver.hpp"

namespace rex::solver {

auto describe(const SolverConfig& config) -> std::string {
  std::string description = "sequential-impulse-pgs";

  description += config.warm_start ? " warm-start" : " cold-start";
  description += config.deterministic_ordering ? " deterministic" : " relaxed-order";
  return description;
}

}  // namespace rex::solver

