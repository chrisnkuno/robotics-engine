#pragma once

#include <filesystem>
#include <string>

#include "rex/viewer/replay.hpp"

namespace rex::viewer {

struct SvgRenderConfig {
  int width{960};
  int height{720};
  double margin{60.0};
  bool show_contacts{true};
  bool show_normals{true};
};

[[nodiscard]] auto render_frame_svg(
  const FrameSnapshot& frame,
  const SvgRenderConfig& config = {}) -> std::string;

void write_frame_svg(
  const std::filesystem::path& path,
  const FrameSnapshot& frame,
  const SvgRenderConfig& config = {});

}  // namespace rex::viewer

