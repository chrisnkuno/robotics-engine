#define SDL_MAIN_HANDLED
#include <SDL2/SDL.h>

#include "rex/viewer/window_app.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <sstream>
#include <stdexcept>
#include <string>

#include "rex/viewer/controller.hpp"

namespace rex::viewer {

namespace {

constexpr int kDefaultWidth = 1280;
constexpr int kDefaultHeight = 800;
constexpr int kSelectionOutlineThickness = 3;

auto set_color(SDL_Renderer* renderer, const std::array<std::uint8_t, 4>& color) -> void {
  SDL_SetRenderDrawColor(renderer, color[0], color[1], color[2], color[3]);
}

auto color_for_body(rex::platform::EntityId id) -> std::array<std::uint8_t, 4> {
  const std::uint32_t seed = id.index * 2654435761u;
  return {
    static_cast<std::uint8_t>(90 + (seed % 120)),
    static_cast<std::uint8_t>(100 + ((seed >> 8) % 100)),
    static_cast<std::uint8_t>(120 + ((seed >> 16) % 100)),
    255,
  };
}

auto update_title(
  SDL_Window* window,
  const FrameSnapshot& frame,
  const ViewerState& state,
  const ReplayLog& replay,
  bool live_mode) -> void {
  std::ostringstream title{};
  title << "rex viewer | frame " << (state.current_frame + 1) << '/' << replay.size()
        << " | " << (state.playback == PlaybackMode::kPlaying ? "playing" : "paused")
        << " | contacts " << frame.trace.solver.contact_count;

  if (live_mode) {
    title << " | live";
  }

  if (state.selection.body_index.has_value() && *state.selection.body_index < frame.bodies.size()) {
    const SnapshotBody& body = frame.bodies[*state.selection.body_index];
    title << " | body "
          << body.id.index
          << " "
          << (body.shape == SnapshotShapeKind::kBox ? "box" : "sphere")
          << " pos=(" << body.translation.x << "," << body.translation.y << "," << body.translation.z << ")";
  } else if (state.selection.contact_index.has_value() && *state.selection.contact_index < frame.contacts.size()) {
    const SnapshotContact& contact = frame.contacts[*state.selection.contact_index];
    title << " | contact "
          << contact.body_a.index << "-" << contact.body_b.index
          << " pen=" << contact.penetration
          << " normal=(" << contact.normal.x << "," << contact.normal.y << "," << contact.normal.z << ")";
  }

  title << " | space play/pause, arrows step, wasd pan, +/- zoom, c/n overlays, r reset";
  SDL_SetWindowTitle(window, title.str().c_str());
}

auto draw_filled_circle(SDL_Renderer* renderer, int cx, int cy, int radius) -> void {
  for (int dy = -radius; dy <= radius; ++dy) {
    const int dx = static_cast<int>(std::sqrt((radius * radius) - (dy * dy)));
    SDL_RenderDrawLine(renderer, cx - dx, cy + dy, cx + dx, cy + dy);
  }
}

auto create_renderer(SDL_Window* window) -> SDL_Renderer* {
  constexpr std::array<Uint32, 3> kRendererFlags = {
    SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC,
    SDL_RENDERER_ACCELERATED,
    SDL_RENDERER_SOFTWARE,
  };

  for (Uint32 flags : kRendererFlags) {
    if (SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, flags)) {
      return renderer;
    }
  }

  return nullptr;
}

auto draw_rect_outline(SDL_Renderer* renderer, SDL_Rect rect, int thickness) -> void {
  for (int offset = 0; offset < thickness; ++offset) {
    SDL_RenderDrawRect(renderer, &rect);
    --rect.x;
    --rect.y;
    rect.w += 2;
    rect.h += 2;
  }
}

auto viewport_for_window(SDL_Window* window) -> FrameViewport {
  int width = kDefaultWidth;
  int height = kDefaultHeight;
  SDL_GetWindowSize(window, &width, &height);
  return {
    .width = static_cast<double>(width),
    .height = static_cast<double>(height),
    .margin = 60.0,
  };
}

auto handle_key(
  ViewerState& state,
  const ReplayLog& replay,
  const SDL_KeyboardEvent& key,
  const FrameViewport& viewport,
  bool& should_quit) -> void {
  if (key.repeat != 0) {
    return;
  }

  switch (key.keysym.sym) {
    case SDLK_ESCAPE:
      should_quit = true;
      return;
    case SDLK_SPACE:
      apply_command(state, replay, ViewerCommand::kTogglePlayPause, viewport);
      return;
    case SDLK_RIGHT:
      apply_command(state, replay, ViewerCommand::kStepForward, viewport);
      return;
    case SDLK_LEFT:
      apply_command(state, replay, ViewerCommand::kStepBackward, viewport);
      return;
    case SDLK_c:
      apply_command(state, replay, ViewerCommand::kToggleContacts, viewport);
      return;
    case SDLK_n:
      apply_command(state, replay, ViewerCommand::kToggleNormals, viewport);
      return;
    case SDLK_r:
      apply_command(state, replay, ViewerCommand::kResetCamera, viewport);
      return;
    case SDLK_EQUALS:
    case SDLK_PLUS:
    case SDLK_KP_PLUS:
      apply_command(state, replay, ViewerCommand::kZoomIn, viewport);
      return;
    case SDLK_MINUS:
    case SDLK_KP_MINUS:
      apply_command(state, replay, ViewerCommand::kZoomOut, viewport);
      return;
    case SDLK_a:
      apply_command(state, replay, ViewerCommand::kPanLeft, viewport);
      return;
    case SDLK_d:
      apply_command(state, replay, ViewerCommand::kPanRight, viewport);
      return;
    case SDLK_w:
      apply_command(state, replay, ViewerCommand::kPanUp, viewport);
      return;
    case SDLK_s:
      apply_command(state, replay, ViewerCommand::kPanDown, viewport);
      return;
    default:
      return;
  }
}

auto box_rect(const SnapshotBody& body, const ViewerState& state, const FrameViewport& viewport) -> SDL_Rect {
  const ScreenPoint center = project_point(state.camera, viewport, body.translation);
  const int width = static_cast<int>(std::round(body.dimensions.x * 2.0 * state.camera.zoom));
  const int height = static_cast<int>(std::round(body.dimensions.z * 2.0 * state.camera.zoom));
  return {
    .x = static_cast<int>(std::round(center.x - (width * 0.5))),
    .y = static_cast<int>(std::round(center.y - (height * 0.5))),
    .w = std::max(width, 1),
    .h = std::max(height, 1),
  };
}

auto draw_box(SDL_Renderer* renderer, const SnapshotBody& body, const ViewerState& state, const FrameViewport& viewport)
  -> void {
  SDL_Rect rect = box_rect(body, state, viewport);

  SDL_RenderFillRect(renderer, &rect);
  set_color(renderer, {34, 30, 28, 255});
  SDL_RenderDrawRect(renderer, &rect);
}

auto draw_sphere(SDL_Renderer* renderer, const SnapshotBody& body, const ViewerState& state, const FrameViewport& viewport)
  -> void {
  const ScreenPoint center = project_point(state.camera, viewport, body.translation);
  const int radius = std::max(static_cast<int>(std::round(body.dimensions.x * state.camera.zoom)), 2);
  draw_filled_circle(renderer, static_cast<int>(std::round(center.x)), static_cast<int>(std::round(center.y)), radius);
  set_color(renderer, {34, 30, 28, 255});
  draw_filled_circle(renderer, static_cast<int>(std::round(center.x)), static_cast<int>(std::round(center.y)), 1);
}

auto draw_timeline(SDL_Renderer* renderer, const ReplayLog& replay, const ViewerState& state, const FrameViewport& viewport)
  -> void {
  const TimelineRect timeline = timeline_rect(viewport);
  SDL_Rect background{
    .x = static_cast<int>(std::round(timeline.left)),
    .y = static_cast<int>(std::round(timeline.top)),
    .w = static_cast<int>(std::round(timeline.width)),
    .h = static_cast<int>(std::round(timeline.height)),
  };
  set_color(renderer, {228, 222, 203, 255});
  SDL_RenderFillRect(renderer, &background);
  set_color(renderer, {90, 87, 80, 255});
  SDL_RenderDrawRect(renderer, &background);

  if (replay.size() > 1) {
    const double alpha = static_cast<double>(state.current_frame) / static_cast<double>(replay.size() - 1);
    const int thumb_x = static_cast<int>(std::round(timeline.left + (alpha * timeline.width)));
    set_color(renderer, {57, 108, 176, 255});
    SDL_RenderDrawLine(
      renderer,
      thumb_x,
      static_cast<int>(std::round(timeline.top)) - 3,
      thumb_x,
      static_cast<int>(std::round(timeline.top + timeline.height)) + 3);

    const std::size_t tick_count = std::min<std::size_t>(replay.size(), 20);
    for (std::size_t tick_index = 0; tick_index < tick_count; ++tick_index) {
      const double tick_alpha = tick_count == 1 ? 0.0 : static_cast<double>(tick_index) / static_cast<double>(tick_count - 1);
      const int tick_x = static_cast<int>(std::round(timeline.left + (tick_alpha * timeline.width)));
      set_color(renderer, {150, 145, 134, 255});
      SDL_RenderDrawLine(
        renderer,
        tick_x,
        static_cast<int>(std::round(timeline.top)),
        tick_x,
        static_cast<int>(std::round(timeline.top + timeline.height)));
    }
  }
}

auto draw_frame(
  SDL_Renderer* renderer,
  const ReplayLog& replay,
  const FrameSnapshot& frame,
  const ViewerState& state,
  const FrameViewport& viewport) -> void {
  set_color(renderer, {247, 244, 234, 255});
  SDL_RenderClear(renderer);

  const ScreenPoint origin = project_point(state.camera, viewport, {0.0, 0.0, 0.0});
  set_color(renderer, {224, 216, 194, 255});
  SDL_RenderDrawLine(renderer, 0, static_cast<int>(std::round(origin.y)), static_cast<int>(std::round(viewport.width)), static_cast<int>(std::round(origin.y)));
  SDL_RenderDrawLine(renderer, static_cast<int>(std::round(origin.x)), 0, static_cast<int>(std::round(origin.x)), static_cast<int>(std::round(viewport.height)));

  for (const SnapshotBody& body : frame.bodies) {
    set_color(renderer, color_for_body(body.id));
    if (body.shape == SnapshotShapeKind::kBox) {
      draw_box(renderer, body, state, viewport);
    } else {
      draw_sphere(renderer, body, state, viewport);
    }
  }

  if (state.overlay.show_contacts) {
    for (const SnapshotContact& contact : frame.contacts) {
      const ScreenPoint point = project_point(state.camera, viewport, contact.position);
      SDL_Rect marker{
        .x = static_cast<int>(std::round(point.x)) - 3,
        .y = static_cast<int>(std::round(point.y)) - 3,
        .w = 6,
        .h = 6,
      };
      set_color(renderer, {200, 69, 69, 255});
      SDL_RenderFillRect(renderer, &marker);

      if (state.overlay.show_normals) {
        const rex::math::Vec3 end_world = {
          contact.position.x + (contact.normal.x * 0.35),
          contact.position.y,
          contact.position.z + (contact.normal.z * 0.35),
        };
        const ScreenPoint end = project_point(state.camera, viewport, end_world);
        SDL_RenderDrawLine(
          renderer,
          static_cast<int>(std::round(point.x)),
          static_cast<int>(std::round(point.y)),
          static_cast<int>(std::round(end.x)),
          static_cast<int>(std::round(end.y)));
      }
    }
  }

  if (state.selection.body_index.has_value() && *state.selection.body_index < frame.bodies.size()) {
    const SnapshotBody& body = frame.bodies[*state.selection.body_index];
    set_color(renderer, {33, 90, 166, 255});
    if (body.shape == SnapshotShapeKind::kBox) {
      draw_rect_outline(renderer, box_rect(body, state, viewport), kSelectionOutlineThickness);
    } else {
      const ScreenPoint center = project_point(state.camera, viewport, body.translation);
      const int radius = std::max(static_cast<int>(std::round(body.dimensions.x * state.camera.zoom)), 2);
      SDL_Rect highlight{
        .x = static_cast<int>(std::round(center.x)) - radius,
        .y = static_cast<int>(std::round(center.y)) - radius,
        .w = radius * 2,
        .h = radius * 2,
      };
      draw_rect_outline(renderer, highlight, kSelectionOutlineThickness);
    }
  }

  if (state.selection.contact_index.has_value() && *state.selection.contact_index < frame.contacts.size()) {
    const SnapshotContact& contact = frame.contacts[*state.selection.contact_index];
    const ScreenPoint point = project_point(state.camera, viewport, contact.position);
    set_color(renderer, {219, 122, 28, 255});
    SDL_RenderDrawLine(
      renderer,
      static_cast<int>(std::round(point.x)) - 8,
      static_cast<int>(std::round(point.y)),
      static_cast<int>(std::round(point.x)) + 8,
      static_cast<int>(std::round(point.y)));
    SDL_RenderDrawLine(
      renderer,
      static_cast<int>(std::round(point.x)),
      static_cast<int>(std::round(point.y)) - 8,
      static_cast<int>(std::round(point.x)),
      static_cast<int>(std::round(point.y)) + 8);
  }

  draw_timeline(renderer, replay, state, viewport);
  SDL_RenderPresent(renderer);
}

}  // namespace

auto run_windowed_viewer_impl(ReplayLog replay, LiveFramePump frame_pump, WindowRunOptions options) -> int {
  if (frame_pump && replay.empty()) {
    if (!frame_pump(replay) || replay.empty()) {
      throw std::runtime_error("live viewer could not produce an initial frame");
    }
  }

  if (replay.empty()) {
    throw std::runtime_error("cannot open a window for an empty replay");
  }

  if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) != 0) {
    throw std::runtime_error(std::string{"SDL_Init failed: "} + SDL_GetError());
  }

  SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "best");

  SDL_Window* window = SDL_CreateWindow(
    "rex viewer",
    SDL_WINDOWPOS_CENTERED,
    SDL_WINDOWPOS_CENTERED,
    kDefaultWidth,
    kDefaultHeight,
    SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE);
  if (window == nullptr) {
    SDL_Quit();
    throw std::runtime_error(std::string{"SDL_CreateWindow failed: "} + SDL_GetError());
  }

  SDL_Renderer* renderer = create_renderer(window);
  if (renderer == nullptr) {
    SDL_DestroyWindow(window);
    SDL_Quit();
    throw std::runtime_error(std::string{"SDL_CreateRenderer failed: "} + SDL_GetError());
  }

  ViewerState state = make_viewer_state(replay);
  if (frame_pump) {
    state.playback = PlaybackMode::kPlaying;
  }
  FrameViewport viewport = viewport_for_window(window);
  fit_camera_to_frame(state, replay.frames()[state.current_frame], viewport);

  bool should_quit = false;
  bool timeline_dragging = false;
  std::size_t rendered_frames = 0;
  std::uint32_t last_ticks = SDL_GetTicks();
  double live_accumulated_time = 0.0;

  while (!should_quit) {
    SDL_Event event{};
    while (SDL_PollEvent(&event) != 0) {
      switch (event.type) {
        case SDL_QUIT:
          should_quit = true;
          break;

        case SDL_KEYDOWN:
          handle_key(state, replay, event.key, viewport, should_quit);
          break;

        case SDL_MOUSEBUTTONDOWN:
          if (event.button.button == SDL_BUTTON_LEFT) {
            const ScreenPoint point{
              .x = static_cast<double>(event.button.x),
              .y = static_cast<double>(event.button.y),
            };
            if (point_in_timeline(timeline_rect(viewport), point)) {
              timeline_dragging = true;
              scrub_to_timeline(state, replay, viewport, point.x);
            } else {
              select_at_point(state, replay.frames()[state.current_frame], viewport, point);
            }
          }
          break;

        case SDL_MOUSEBUTTONUP:
          if (event.button.button == SDL_BUTTON_LEFT) {
            timeline_dragging = false;
          }
          break;

        case SDL_MOUSEMOTION:
          if (timeline_dragging) {
            scrub_to_timeline(state, replay, viewport, static_cast<double>(event.motion.x));
          }
          break;

        case SDL_WINDOWEVENT:
          if (event.window.event == SDL_WINDOWEVENT_SIZE_CHANGED) {
            viewport.width = static_cast<double>(event.window.data1);
            viewport.height = static_cast<double>(event.window.data2);
          }
          break;

        default:
          break;
      }
    }

    const std::uint32_t ticks = SDL_GetTicks();
    const double delta_seconds = static_cast<double>(ticks - last_ticks) / 1000.0;
    last_ticks = ticks;

    if (frame_pump && state.playback == PlaybackMode::kPlaying && state.playback_fps > 0.0) {
      live_accumulated_time += delta_seconds;
      const double frame_period = 1.0 / state.playback_fps;
      while (live_accumulated_time >= frame_period) {
        live_accumulated_time -= frame_period;
        if (!frame_pump(replay)) {
          break;
        }
      }
    }

    update_playback(state, replay, delta_seconds);
    const FrameSnapshot& frame = replay.frames()[state.current_frame];
    update_title(window, frame, state, replay, static_cast<bool>(frame_pump));
    draw_frame(renderer, replay, frame, state, viewport);

    ++rendered_frames;
    if (options.max_frames > 0 && rendered_frames >= options.max_frames) {
      should_quit = true;
    }
  }

  SDL_DestroyRenderer(renderer);
  SDL_DestroyWindow(window);
  SDL_Quit();
  return 0;
}

auto run_windowed_viewer(const ReplayLog& replay, WindowRunOptions options) -> int {
  return run_windowed_viewer_impl(replay, {}, options);
}

auto run_live_windowed_viewer(ReplayLog replay, LiveFramePump frame_pump, WindowRunOptions options) -> int {
  if (!frame_pump) {
    throw std::runtime_error("live viewer requires a valid frame pump");
  }

  return run_windowed_viewer_impl(std::move(replay), std::move(frame_pump), options);
}

}  // namespace rex::viewer
