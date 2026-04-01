#define SDL_MAIN_HANDLED
#include <SDL2/SDL.h>

#include "rex/viewer/window_app.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>

#include "rex/platform/profile.hpp"
#include "rex/viewer/controller.hpp"

namespace rex::viewer {

namespace {

constexpr int kDefaultWidth = 1280;
constexpr int kDefaultHeight = 800;
constexpr int kSelectionOutlineThickness = 3;
constexpr double kAverageAlpha = 0.15;

struct ViewerFrameProfile {
  double event_ms{0.0};
  double live_pump_ms{0.0};
  double cache_build_ms{0.0};
  double draw_ms{0.0};
  double frame_ms{0.0};
};

struct ViewerFrameAverages {
  double event_ms{0.0};
  double live_pump_ms{0.0};
  double cache_build_ms{0.0};
  double draw_ms{0.0};
  double frame_ms{0.0};
};

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

[[nodiscard]] auto make_title(
  const FrameSnapshot& frame,
  const ViewerState& state,
  const ReplayLog& replay,
  const ViewerFrameAverages& averages,
  bool live_mode) -> std::string {
  std::ostringstream title{};
  title << "rex viewer | frame " << (state.current_frame + 1) << '/' << replay.size()
        << " | " << (state.playback == PlaybackMode::kPlaying ? "playing" : "paused")
        << " | contacts " << frame.trace.solver.contact_count
        << " | sim "
        << frame.trace.profile.total_ms << "ms"
        << " (c " << frame.trace.profile.collision_ms
        << " s " << frame.trace.profile.solver_ms << ")"
        << " | draw " << averages.draw_ms << "ms"
        << " | cache " << averages.cache_build_ms << "ms"
        << " | frame " << averages.frame_ms << "ms";

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
  return title.str();
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

auto polygon_centroid(const std::vector<ScreenPoint>& polygon) -> ScreenPoint {
  ScreenPoint centroid{};
  if (polygon.empty()) {
    return centroid;
  }

  for (const ScreenPoint& point : polygon) {
    centroid.x += point.x;
    centroid.y += point.y;
  }
  centroid.x /= static_cast<double>(polygon.size());
  centroid.y /= static_cast<double>(polygon.size());
  return centroid;
}

void update_average(double sample, double& average) {
  average = average == 0.0
    ? sample
    : (average + ((sample - average) * kAverageAlpha));
}

void update_averages(const ViewerFrameProfile& sample, ViewerFrameAverages& averages) {
  update_average(sample.event_ms, averages.event_ms);
  update_average(sample.live_pump_ms, averages.live_pump_ms);
  update_average(sample.cache_build_ms, averages.cache_build_ms);
  update_average(sample.draw_ms, averages.draw_ms);
  update_average(sample.frame_ms, averages.frame_ms);
}

auto draw_polygon_outline(SDL_Renderer* renderer, const std::vector<ScreenPoint>& polygon, int thickness = 1) -> void {
  if (polygon.size() < 2) {
    return;
  }

  for (int offset = 0; offset < thickness; ++offset) {
    for (std::size_t index = 0; index < polygon.size(); ++index) {
      const ScreenPoint& start = polygon[index];
      const ScreenPoint& end = polygon[(index + 1) % polygon.size()];
      SDL_RenderDrawLine(
        renderer,
        static_cast<int>(std::round(start.x)),
        static_cast<int>(std::round(start.y)) + offset,
        static_cast<int>(std::round(end.x)),
        static_cast<int>(std::round(end.y)) + offset);
    }
  }
}

#if SDL_VERSION_ATLEAST(2, 0, 18)
auto draw_filled_polygon(
  SDL_Renderer* renderer,
  const std::vector<ScreenPoint>& polygon,
  const std::array<std::uint8_t, 4>& color) -> void {
  if (polygon.size() < 3) {
    draw_polygon_outline(renderer, polygon);
    return;
  }

  const ScreenPoint centroid = polygon_centroid(polygon);
  std::vector<SDL_Vertex> vertices{};
  vertices.reserve(polygon.size() + 1);
  vertices.push_back({
    .position = {static_cast<float>(centroid.x), static_cast<float>(centroid.y)},
    .color = {color[0], color[1], color[2], color[3]},
    .tex_coord = {0.0f, 0.0f},
  });
  for (const ScreenPoint& point : polygon) {
    vertices.push_back({
      .position = {static_cast<float>(point.x), static_cast<float>(point.y)},
      .color = {color[0], color[1], color[2], color[3]},
      .tex_coord = {0.0f, 0.0f},
    });
  }

  std::vector<int> indices{};
  indices.reserve(polygon.size() * 3);
  for (std::size_t index = 0; index < polygon.size(); ++index) {
    indices.push_back(0);
    indices.push_back(static_cast<int>(index + 1));
    indices.push_back(static_cast<int>((index + 1) % polygon.size()) + 1);
  }

  SDL_RenderGeometry(renderer, nullptr, vertices.data(), static_cast<int>(vertices.size()), indices.data(), static_cast<int>(indices.size()));
}
#else
auto draw_filled_polygon(
  SDL_Renderer* renderer,
  const std::vector<ScreenPoint>& polygon,
  const std::array<std::uint8_t, 4>&) -> void {
  draw_polygon_outline(renderer, polygon);
}
#endif

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

auto draw_box(
  SDL_Renderer* renderer,
  const ProjectedBody& body,
  const std::array<std::uint8_t, 4>& fill_color)
  -> void {
  draw_filled_polygon(renderer, body.outline, fill_color);
  set_color(renderer, {34, 30, 28, 255});
  draw_polygon_outline(renderer, body.outline);
}

auto draw_sphere(SDL_Renderer* renderer, const ProjectedBody& body)
  -> void {
  const int radius = std::max(static_cast<int>(std::round(body.radius_pixels)), 2);
  draw_filled_circle(
    renderer,
    static_cast<int>(std::round(body.center.x)),
    static_cast<int>(std::round(body.center.y)),
    radius);
  set_color(renderer, {34, 30, 28, 255});
  draw_filled_circle(
    renderer,
    static_cast<int>(std::round(body.center.x)),
    static_cast<int>(std::round(body.center.y)),
    1);
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
  const FrameViewport& viewport,
  const FrameProjectionCache& cache) -> void {
  REX_PROFILE_SCOPE("viewer::draw_frame");
  set_color(renderer, {247, 244, 234, 255});
  SDL_RenderClear(renderer);

  const ScreenPoint origin = project_point(state.camera, viewport, {0.0, 0.0, 0.0});
  set_color(renderer, {224, 216, 194, 255});
  SDL_RenderDrawLine(renderer, 0, static_cast<int>(std::round(origin.y)), static_cast<int>(std::round(viewport.width)), static_cast<int>(std::round(origin.y)));
  SDL_RenderDrawLine(renderer, static_cast<int>(std::round(origin.x)), 0, static_cast<int>(std::round(origin.x)), static_cast<int>(std::round(viewport.height)));

  for (std::size_t body_index = 0; body_index < frame.bodies.size(); ++body_index) {
    const SnapshotBody& body = frame.bodies[body_index];
    const ProjectedBody& projected = cache.bodies[body_index];
    const auto fill_color = color_for_body(body.id);
    set_color(renderer, fill_color);
    if (body.shape == SnapshotShapeKind::kBox) {
      draw_box(renderer, projected, fill_color);
    } else {
      draw_sphere(renderer, projected);
    }
  }

  if (state.overlay.show_contacts) {
    for (std::size_t contact_index = 0; contact_index < frame.contacts.size(); ++contact_index) {
      const SnapshotContact& contact = frame.contacts[contact_index];
      const ProjectedContact& projected = cache.contacts[contact_index];
      SDL_Rect marker{
        .x = static_cast<int>(std::round(projected.position.x)) - 3,
        .y = static_cast<int>(std::round(projected.position.y)) - 3,
        .w = 6,
        .h = 6,
      };
      set_color(renderer, {200, 69, 69, 255});
      SDL_RenderFillRect(renderer, &marker);

      if (state.overlay.show_normals) {
        SDL_RenderDrawLine(
          renderer,
          static_cast<int>(std::round(projected.position.x)),
          static_cast<int>(std::round(projected.position.y)),
          static_cast<int>(std::round(projected.normal_end.x)),
          static_cast<int>(std::round(projected.normal_end.y)));
      }
    }
  }

  if (state.selection.body_index.has_value() && *state.selection.body_index < frame.bodies.size()) {
    const ProjectedBody& projected = cache.bodies[*state.selection.body_index];
    const SnapshotBody& body = frame.bodies[*state.selection.body_index];
    set_color(renderer, {33, 90, 166, 255});
    if (body.shape == SnapshotShapeKind::kBox) {
      draw_polygon_outline(renderer, projected.outline, kSelectionOutlineThickness);
    } else {
      const int radius = std::max(static_cast<int>(std::round(projected.radius_pixels)), 2);
      SDL_Rect highlight{
        .x = static_cast<int>(std::round(projected.center.x)) - radius,
        .y = static_cast<int>(std::round(projected.center.y)) - radius,
        .w = radius * 2,
        .h = radius * 2,
      };
      draw_rect_outline(renderer, highlight, kSelectionOutlineThickness);
    }
  }

  if (state.selection.contact_index.has_value() && *state.selection.contact_index < frame.contacts.size()) {
    const ScreenPoint point = cache.contacts[*state.selection.contact_index].position;
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

  SDL_SetHint(SDL_HINT_RENDER_BATCHING, "1");
  SDL_SetHint(SDL_HINT_RENDER_VSYNC, "1");
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
  std::string current_title{};
  ViewerFrameAverages averages{};

  while (!should_quit) {
    REX_PROFILE_FRAME_MARK();
    ViewerFrameProfile frame_profile{};
    rex::platform::ScopedMilliseconds frame_timer(frame_profile.frame_ms);
    SDL_Event event{};
    const bool should_block_for_events =
      !timeline_dragging &&
      state.playback != PlaybackMode::kPlaying;
    std::optional<ScreenPoint> pending_selection{};
    {
      REX_PROFILE_SCOPE("viewer::events");
      rex::platform::ScopedMilliseconds event_timer(frame_profile.event_ms);
      bool has_event = should_block_for_events
        ? SDL_WaitEventTimeout(&event, 16) == 1
        : SDL_PollEvent(&event) != 0;
      while (has_event) {
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
                pending_selection = point;
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

        has_event = SDL_PollEvent(&event) != 0;
      }
    }

    const std::uint32_t ticks = SDL_GetTicks();
    const double delta_seconds = static_cast<double>(ticks - last_ticks) / 1000.0;
    last_ticks = ticks;

    if (frame_pump && state.playback == PlaybackMode::kPlaying && state.playback_fps > 0.0) {
      REX_PROFILE_SCOPE("viewer::live_pump");
      rex::platform::ScopedMilliseconds live_pump_timer(frame_profile.live_pump_ms);
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
    FrameProjectionCache cache{};
    {
      REX_PROFILE_SCOPE("viewer::cache");
      rex::platform::ScopedMilliseconds cache_timer(frame_profile.cache_build_ms);
      cache = build_frame_projection_cache(frame, state.camera, viewport);
    }
    if (pending_selection.has_value()) {
      select_at_point(state, frame, cache, *pending_selection);
    }
    {
      REX_PROFILE_SCOPE("viewer::draw");
      rex::platform::ScopedMilliseconds draw_timer(frame_profile.draw_ms);
      draw_frame(renderer, replay, frame, state, viewport, cache);
    }
    update_averages(frame_profile, averages);
    const std::string next_title = make_title(frame, state, replay, averages, static_cast<bool>(frame_pump));
    if (next_title != current_title) {
      SDL_SetWindowTitle(window, next_title.c_str());
      current_title = next_title;
    }

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
