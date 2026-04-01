#pragma once

#include <cstddef>

#include "rex/viewer/replay.hpp"

namespace rex::viewer {

[[nodiscard]] auto build_demo_replay(std::size_t frame_count = 8) -> ReplayLog;

}  // namespace rex::viewer

