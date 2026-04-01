#pragma once

#include <cstddef>
#include <cstdint>

#include "rex/dynamics/world.hpp"
#include "rex/sim/engine.hpp"
#include "rex/viewer/replay.hpp"

namespace rex::viewer {

class DemoSceneRunner {
 public:
  DemoSceneRunner();

  [[nodiscard]] auto step_frame() -> FrameSnapshot;

 private:
  rex::sim::EngineConfig config_{};
  rex::sim::Engine engine_{};
  rex::dynamics::WorldState world_{};
  std::uint64_t next_frame_index_{0};
  double sim_time_{0.0};
};

[[nodiscard]] auto build_demo_replay(std::size_t frame_count = 8) -> ReplayLog;

}  // namespace rex::viewer
