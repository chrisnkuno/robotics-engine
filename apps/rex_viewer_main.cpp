#include <filesystem>
#include <iostream>
#include <string>

#include "rex/viewer/demo.hpp"
#include "rex/viewer/replay.hpp"
#include "rex/viewer/svg_renderer.hpp"

namespace {

void export_svg_frames(
  const rex::viewer::ReplayLog& replay,
  const std::filesystem::path& output_dir) {
  std::filesystem::create_directories(output_dir);

  for (const auto& frame : replay.frames()) {
    const std::filesystem::path svg_path =
      output_dir / ("frame-" + std::to_string(frame.frame_index) + ".svg");
    rex::viewer::write_frame_svg(svg_path, frame);
  }
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 3) {
    std::cerr << "usage: rex_viewer_app <replay-file> <output-dir>\n";
    std::cerr << "   or: rex_viewer_app --demo <output-dir>\n";
    return 1;
  }

  try {
    const std::filesystem::path output_dir = argv[2];

    rex::viewer::ReplayLog replay{};
    if (std::string_view{argv[1]} == "--demo") {
      replay = rex::viewer::build_demo_replay();
      const std::filesystem::path replay_path = output_dir / "demo.rex";
      std::filesystem::create_directories(output_dir);
      replay.save(replay_path);
      std::cout << "wrote demo replay to " << replay_path << '\n';
    } else {
      const std::filesystem::path replay_path = argv[1];
      replay = rex::viewer::ReplayLog::load(replay_path);
    }

    export_svg_frames(replay, output_dir);
    std::cout << "exported " << replay.size() << " frame(s) to " << output_dir << '\n';
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "viewer error: " << error.what() << '\n';
    return 1;
  }
}
