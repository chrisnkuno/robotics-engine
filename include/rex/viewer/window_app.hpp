#pragma once

#include <cstddef>
#include <filesystem>
#include <functional>

#include "rex/viewer/replay.hpp"

namespace rex::viewer {

using LiveFramePump = std::function<bool(ReplayLog&)>;

struct WindowRunOptions {
  std::size_t max_frames{0};
  std::filesystem::path screenshot_path{};
};

[[nodiscard]] auto run_windowed_viewer(const ReplayLog& replay, WindowRunOptions options = {}) -> int;
[[nodiscard]] auto run_live_windowed_viewer(
  ReplayLog replay,
  LiveFramePump frame_pump,
  WindowRunOptions options = {}) -> int;

}  // namespace rex::viewer
