#include "rex/viewer/controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace rex::viewer {

namespace {

constexpr double kPanDistance = 0.25;
constexpr double kZoomFactor = 1.15;
constexpr double kMinZoom = 20.0;
constexpr double kMaxZoom = 800.0;
constexpr double kTimelineHeight = 18.0;
constexpr double kTimelineBottomPadding = 22.0;
constexpr double kSelectionThresholdPixels = 10.0;

struct Bounds2D {
  double min_x{0.0};
  double max_x{0.0};
  double min_z{0.0};
  double max_z{0.0};
};

[[nodiscard]] auto distance_squared(const ScreenPoint& lhs, const ScreenPoint& rhs) -> double {
  const double dx = lhs.x - rhs.x;
  const double dy = lhs.y - rhs.y;
  return (dx * dx) + (dy * dy);
}

[[nodiscard]] auto frame_bounds(const FrameSnapshot& frame) -> Bounds2D {
  Bounds2D bounds{};
  bool initialized = false;

  auto extend = [&](double x, double z) {
    if (!initialized) {
      bounds.min_x = bounds.max_x = x;
      bounds.min_z = bounds.max_z = z;
      initialized = true;
      return;
    }

    bounds.min_x = std::min(bounds.min_x, x);
    bounds.max_x = std::max(bounds.max_x, x);
    bounds.min_z = std::min(bounds.min_z, z);
    bounds.max_z = std::max(bounds.max_z, z);
  };

  for (const auto& body : frame.bodies) {
    if (body.shape == SnapshotShapeKind::kSphere) {
      extend(body.translation.x - body.dimensions.x, body.translation.z - body.dimensions.x);
      extend(body.translation.x + body.dimensions.x, body.translation.z + body.dimensions.x);
    } else {
      extend(body.translation.x - body.dimensions.x, body.translation.z - body.dimensions.z);
      extend(body.translation.x + body.dimensions.x, body.translation.z + body.dimensions.z);
    }
  }

  for (const auto& contact : frame.contacts) {
    extend(contact.position.x, contact.position.z);
  }

  if (!initialized) {
    extend(-1.0, -1.0);
    extend(1.0, 1.0);
  }

  return bounds;
}

[[nodiscard]] auto clamped_frame_index(std::size_t index, const ReplayLog& replay) -> std::size_t {
  if (replay.empty()) {
    return 0;
  }

  return std::min(index, replay.size() - 1);
}

[[nodiscard]] auto nearest_body_distance_squared(
  const SnapshotBody& body,
  const Camera2D& camera,
  const FrameViewport& viewport,
  const ScreenPoint& point) -> double {
  const ScreenPoint center = project_point(camera, viewport, body.translation);
  if (body.shape == SnapshotShapeKind::kSphere) {
    const double radius = std::max(body.dimensions.x * camera.zoom, 2.0);
    const double squared = distance_squared(center, point);
    return squared <= (radius * radius) ? squared : std::numeric_limits<double>::infinity();
  }

  const double half_width = std::max(body.dimensions.x * camera.zoom, 0.5);
  const double half_height = std::max(body.dimensions.z * camera.zoom, 0.5);
  const bool inside = std::abs(point.x - center.x) <= half_width && std::abs(point.y - center.y) <= half_height;
  return inside ? distance_squared(center, point) : std::numeric_limits<double>::infinity();
}

}  // namespace

auto make_viewer_state(const ReplayLog& replay) -> ViewerState {
  ViewerState state{};
  state.current_frame = replay.empty() ? 0 : 0;
  return state;
}

void fit_camera_to_frame(ViewerState& state, const FrameSnapshot& frame, const FrameViewport& viewport) {
  const Bounds2D bounds = frame_bounds(frame);
  const double extent_x = std::max(bounds.max_x - bounds.min_x, 1.0);
  const double extent_z = std::max(bounds.max_z - bounds.min_z, 1.0);
  const double usable_width = std::max(viewport.width - (viewport.margin * 2.0), 1.0);
  const double usable_height = std::max(viewport.height - (viewport.margin * 2.0), 1.0);

  state.camera.center_x = (bounds.min_x + bounds.max_x) * 0.5;
  state.camera.center_z = (bounds.min_z + bounds.max_z) * 0.5;
  state.camera.zoom = std::clamp(
    std::min(usable_width / extent_x, usable_height / extent_z),
    kMinZoom,
    kMaxZoom);
}

void clear_selection(ViewerState& state) {
  state.selection.body_index.reset();
  state.selection.contact_index.reset();
}

void apply_command(ViewerState& state, const ReplayLog& replay, ViewerCommand command, const FrameViewport& viewport) {
  switch (command) {
    case ViewerCommand::kTogglePlayPause:
      state.playback = state.playback == PlaybackMode::kPlaying ? PlaybackMode::kPaused : PlaybackMode::kPlaying;
      break;

    case ViewerCommand::kStepForward:
      state.current_frame = clamped_frame_index(state.current_frame + 1, replay);
      state.accumulated_playback_time = 0.0;
      clear_selection(state);
      break;

    case ViewerCommand::kStepBackward:
      state.current_frame = state.current_frame == 0 ? 0 : state.current_frame - 1;
      state.accumulated_playback_time = 0.0;
      clear_selection(state);
      break;

    case ViewerCommand::kToggleContacts:
      state.overlay.show_contacts = !state.overlay.show_contacts;
      break;

    case ViewerCommand::kToggleNormals:
      state.overlay.show_normals = !state.overlay.show_normals;
      break;

    case ViewerCommand::kZoomIn:
      state.camera.zoom = std::clamp(state.camera.zoom * kZoomFactor, kMinZoom, kMaxZoom);
      break;

    case ViewerCommand::kZoomOut:
      state.camera.zoom = std::clamp(state.camera.zoom / kZoomFactor, kMinZoom, kMaxZoom);
      break;

    case ViewerCommand::kPanLeft:
      state.camera.center_x -= kPanDistance;
      break;

    case ViewerCommand::kPanRight:
      state.camera.center_x += kPanDistance;
      break;

    case ViewerCommand::kPanUp:
      state.camera.center_z += kPanDistance;
      break;

    case ViewerCommand::kPanDown:
      state.camera.center_z -= kPanDistance;
      break;

    case ViewerCommand::kResetCamera:
      if (!replay.empty()) {
        fit_camera_to_frame(state, replay.frames()[state.current_frame], viewport);
      }
      break;
  }
}

auto timeline_rect(const FrameViewport& viewport) -> TimelineRect {
  return {
    .left = viewport.margin,
    .top = std::max(viewport.height - kTimelineBottomPadding - kTimelineHeight, 0.0),
    .width = std::max(viewport.width - (viewport.margin * 2.0), 1.0),
    .height = kTimelineHeight,
  };
}

auto point_in_timeline(const TimelineRect& timeline, const ScreenPoint& point) -> bool {
  return point.x >= timeline.left &&
         point.x <= timeline.left + timeline.width &&
         point.y >= timeline.top &&
         point.y <= timeline.top + timeline.height;
}

auto frame_index_for_timeline(const ReplayLog& replay, const TimelineRect& timeline, double x) -> std::size_t {
  if (replay.empty()) {
    return 0;
  }

  const double clamped_x = std::clamp(x, timeline.left, timeline.left + timeline.width);
  const double alpha = timeline.width <= 0.0 ? 0.0 : (clamped_x - timeline.left) / timeline.width;
  const double scaled = alpha * static_cast<double>(replay.size() - 1);
  return static_cast<std::size_t>(std::llround(scaled));
}

void scrub_to_timeline(ViewerState& state, const ReplayLog& replay, const FrameViewport& viewport, double x) {
  state.current_frame = frame_index_for_timeline(replay, timeline_rect(viewport), x);
  state.playback = PlaybackMode::kPaused;
  state.accumulated_playback_time = 0.0;
  clear_selection(state);
}

auto pick_body(
  const FrameSnapshot& frame,
  const Camera2D& camera,
  const FrameViewport& viewport,
  const ScreenPoint& point) -> std::optional<std::size_t> {
  std::optional<std::size_t> best_index{};
  double best_distance_squared = std::numeric_limits<double>::infinity();

  for (std::size_t body_index = 0; body_index < frame.bodies.size(); ++body_index) {
    const double distance = nearest_body_distance_squared(frame.bodies[body_index], camera, viewport, point);
    if (distance < best_distance_squared) {
      best_distance_squared = distance;
      best_index = body_index;
    }
  }

  return best_index;
}

auto pick_contact(
  const FrameSnapshot& frame,
  const Camera2D& camera,
  const FrameViewport& viewport,
  const ScreenPoint& point) -> std::optional<std::size_t> {
  std::optional<std::size_t> best_index{};
  double best_distance_squared = kSelectionThresholdPixels * kSelectionThresholdPixels;

  for (std::size_t contact_index = 0; contact_index < frame.contacts.size(); ++contact_index) {
    const ScreenPoint projected = project_point(camera, viewport, frame.contacts[contact_index].position);
    const double distance = distance_squared(projected, point);
    if (distance <= best_distance_squared) {
      best_distance_squared = distance;
      best_index = contact_index;
    }
  }

  return best_index;
}

void select_at_point(
  ViewerState& state,
  const FrameSnapshot& frame,
  const FrameViewport& viewport,
  const ScreenPoint& point) {
  if (const auto contact_index = pick_contact(frame, state.camera, viewport, point)) {
    state.selection.contact_index = contact_index;
    state.selection.body_index.reset();
    return;
  }

  if (const auto body_index = pick_body(frame, state.camera, viewport, point)) {
    state.selection.body_index = body_index;
    state.selection.contact_index.reset();
    return;
  }

  clear_selection(state);
}

void update_playback(ViewerState& state, const ReplayLog& replay, double delta_seconds) {
  if (state.playback != PlaybackMode::kPlaying || replay.size() < 2 || state.playback_fps <= 0.0) {
    return;
  }

  state.accumulated_playback_time += delta_seconds;
  const double frame_period = 1.0 / state.playback_fps;
  while (state.accumulated_playback_time >= frame_period) {
    state.accumulated_playback_time -= frame_period;

    if (state.current_frame + 1 >= replay.size()) {
      state.current_frame = replay.size() - 1;
      state.playback = PlaybackMode::kPaused;
      state.accumulated_playback_time = 0.0;
      break;
    }

    ++state.current_frame;
  }
}

auto project_point(
  const Camera2D& camera,
  const FrameViewport& viewport,
  const rex::math::Vec3& world_point) -> ScreenPoint {
  return {
    .x = (viewport.width * 0.5) + ((world_point.x - camera.center_x) * camera.zoom),
    .y = (viewport.height * 0.5) - ((world_point.z - camera.center_z) * camera.zoom),
  };
}

}  // namespace rex::viewer
