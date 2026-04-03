#include <filesystem>
#include <iostream>
#include <string>

#include "rex/viewer/demo.hpp"
#include "rex/viewer/replay.hpp"
#include "rex/viewer/svg_renderer.hpp"
#include "rex/viewer/window_app.hpp"

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
  if (argc < 2) {
    std::cerr << "usage: rex_viewer_app <replay-file> <output-dir>\n";
    std::cerr << "   or: rex_viewer_app --demo <output-dir>\n";
    std::cerr << "   or: rex_viewer_app --window <replay-file> [max-frames]\n";
    std::cerr << "   or: rex_viewer_app --demo-window [max-frames]\n";
    std::cerr << "   or: rex_viewer_app --demo-shot <output-bmp>\n";
    std::cerr << "   or: rex_viewer_app --window-shot <replay-file> <output-bmp>\n";
    std::cerr << "   or: rex_viewer_app --live-demo-window [max-frames]\n";
    return 1;
  }

  try {
    const std::string_view mode = argv[1];

    if (mode == "--demo") {
      if (argc != 3) {
        std::cerr << "usage: rex_viewer_app --demo <output-dir>\n";
        return 1;
      }

      const std::filesystem::path output_dir = argv[2];
      rex::viewer::ReplayLog replay = rex::viewer::build_demo_replay();
      const std::filesystem::path replay_path = output_dir / "demo.rex";
      std::filesystem::create_directories(output_dir);
      replay.save(replay_path);
      std::cout << "wrote demo replay to " << replay_path << '\n';
      export_svg_frames(replay, output_dir);
      std::cout << "exported " << replay.size() << " frame(s) to " << output_dir << '\n';
      return 0;
    }

    if (mode == "--window") {
      if (argc < 3 || argc > 4) {
        std::cerr << "usage: rex_viewer_app --window <replay-file> [max-frames]\n";
        return 1;
      }

      const std::filesystem::path replay_path = argv[2];
      const rex::viewer::ReplayLog replay = rex::viewer::ReplayLog::load(replay_path);
      const std::size_t max_frames = argc == 4 ? static_cast<std::size_t>(std::stoul(argv[3])) : 0;
      return rex::viewer::run_windowed_viewer(replay, {.max_frames = max_frames});
    }

    if (mode == "--demo-window") {
      if (argc > 3) {
        std::cerr << "usage: rex_viewer_app --demo-window [max-frames]\n";
        return 1;
      }

      const std::size_t max_frames = argc == 3 ? static_cast<std::size_t>(std::stoul(argv[2])) : 0;
      const rex::viewer::ReplayLog replay = rex::viewer::build_demo_replay();
      return rex::viewer::run_windowed_viewer(replay, {.max_frames = max_frames});
    }

    if (mode == "--demo-shot") {
      if (argc != 3) {
        std::cerr << "usage: rex_viewer_app --demo-shot <output-bmp>\n";
        return 1;
      }

      const rex::viewer::ReplayLog replay = rex::viewer::build_demo_replay();
      return rex::viewer::run_windowed_viewer(replay, {.max_frames = 1, .screenshot_path = argv[2]});
    }

    if (mode == "--window-shot") {
      if (argc != 4) {
        std::cerr << "usage: rex_viewer_app --window-shot <replay-file> <output-bmp>\n";
        return 1;
      }

      const rex::viewer::ReplayLog replay = rex::viewer::ReplayLog::load(argv[2]);
      return rex::viewer::run_windowed_viewer(replay, {.max_frames = 1, .screenshot_path = argv[3]});
    }

    if (mode == "--live-demo-window") {
      if (argc > 3) {
        std::cerr << "usage: rex_viewer_app --live-demo-window [max-frames]\n";
        return 1;
      }

      const std::size_t max_frames = argc == 3 ? static_cast<std::size_t>(std::stoul(argv[2])) : 0;
      return rex::viewer::run_live_windowed_viewer(
        {},
        [runner = rex::viewer::DemoSceneRunner{}](rex::viewer::ReplayLog& replay) mutable {
          replay.add_frame(runner.step_frame());
          return true;
        },
        {.max_frames = max_frames});
    }

    if (argc != 3) {
      std::cerr << "usage: rex_viewer_app <replay-file> <output-dir>\n";
      return 1;
    }

    const std::filesystem::path replay_path = argv[1];
    const std::filesystem::path output_dir = argv[2];
    const rex::viewer::ReplayLog replay = rex::viewer::ReplayLog::load(replay_path);
    export_svg_frames(replay, output_dir);
    std::cout << "exported " << replay.size() << " frame(s) to " << output_dir << '\n';
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "viewer error: " << error.what() << '\n';
    return 1;
  }
}
