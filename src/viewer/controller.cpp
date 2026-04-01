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
constexpr double kPolygonTolerance = 1.0e-6;

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

[[nodiscard]] auto cross_z(const ScreenPoint& origin, const ScreenPoint& lhs, const ScreenPoint& rhs) -> double {
  const double ax = lhs.x - origin.x;
  const double ay = lhs.y - origin.y;
  const double bx = rhs.x - origin.x;
  const double by = rhs.y - origin.y;
  return (ax * by) - (ay * bx);
}

[[nodiscard]] auto screen_point_less(const ScreenPoint& lhs, const ScreenPoint& rhs) -> bool {
  if (lhs.x < rhs.x) {
    return true;
  }

  if (rhs.x < lhs.x) {
    return false;
  }

  return lhs.y < rhs.y;
}

[[nodiscard]] auto body_box_corners(const SnapshotBody& body) -> std::vector<rex::math::Vec3> {
  const std::array<double, 2> signs = {-1.0, 1.0};
  std::vector<rex::math::Vec3> corners{};
  corners.reserve(8);
  const rex::math::Transform transform{
    .rotation = body.rotation,
    .translation = body.translation,
  };

  for (double sx : signs) {
    for (double sy : signs) {
      for (double sz : signs) {
        corners.push_back(rex::math::transform_point(
          transform,
          {
            sx * body.dimensions.x,
            sy * body.dimensions.y,
            sz * body.dimensions.z,
          }));
      }
    }
  }

  return corners;
}

[[nodiscard]] auto convex_hull(std::vector<ScreenPoint> points) -> std::vector<ScreenPoint> {
  if (points.size() <= 1) {
    return points;
  }

  std::sort(points.begin(), points.end(), screen_point_less);
  points.erase(
    std::unique(
      points.begin(),
      points.end(),
      [](const ScreenPoint& lhs, const ScreenPoint& rhs) {
        return std::abs(lhs.x - rhs.x) <= kPolygonTolerance &&
               std::abs(lhs.y - rhs.y) <= kPolygonTolerance;
      }),
    points.end());

  if (points.size() <= 2) {
    return points;
  }

  std::vector<ScreenPoint> lower{};
  for (const ScreenPoint& point : points) {
    while (lower.size() >= 2 &&
           cross_z(lower[lower.size() - 2], lower.back(), point) <= kPolygonTolerance) {
      lower.pop_back();
    }
    lower.push_back(point);
  }

  std::vector<ScreenPoint> upper{};
  for (auto it = points.rbegin(); it != points.rend(); ++it) {
    while (upper.size() >= 2 &&
           cross_z(upper[upper.size() - 2], upper.back(), *it) <= kPolygonTolerance) {
      upper.pop_back();
    }
    upper.push_back(*it);
  }

  lower.pop_back();
  upper.pop_back();
  lower.insert(lower.end(), upper.begin(), upper.end());
  return lower;
}

[[nodiscard]] auto point_in_convex_polygon(
  const std::vector<ScreenPoint>& polygon,
  const ScreenPoint& point) -> bool {
  if (polygon.empty()) {
    return false;
  }

  if (polygon.size() == 1) {
    return distance_squared(polygon.front(), point) <= (kSelectionThresholdPixels * kSelectionThresholdPixels);
  }

  if (polygon.size() == 2) {
    const double area = std::abs(cross_z(polygon[0], polygon[1], point));
    if (area > kSelectionThresholdPixels) {
      return false;
    }

    const double dot =
      ((point.x - polygon[0].x) * (point.x - polygon[1].x)) +
      ((point.y - polygon[0].y) * (point.y - polygon[1].y));
    return dot <= 0.0;
  }

  bool saw_positive = false;
  bool saw_negative = false;
  for (std::size_t index = 0; index < polygon.size(); ++index) {
    const ScreenPoint& current = polygon[index];
    const ScreenPoint& next = polygon[(index + 1) % polygon.size()];
    const double cross = cross_z(current, next, point);
    saw_positive = saw_positive || cross > kPolygonTolerance;
    saw_negative = saw_negative || cross < -kPolygonTolerance;
    if (saw_positive && saw_negative) {
      return false;
    }
  }

  return true;
}

[[nodiscard]] auto polygon_centroid(const std::vector<ScreenPoint>& polygon) -> ScreenPoint {
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
      for (const rex::math::Vec3& corner : body_box_corners(body)) {
        extend(corner.x, corner.z);
      }
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
  const ProjectedBody& body,
  const ScreenPoint& point) -> double {
  if (body.outline.empty()) {
    const double squared = distance_squared(body.center, point);
    return squared <= (body.radius_pixels * body.radius_pixels)
      ? squared
      : std::numeric_limits<double>::infinity();
  }

  if (!point_in_convex_polygon(body.outline, point)) {
    return std::numeric_limits<double>::infinity();
  }

  return distance_squared(body.selection_anchor, point);
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

auto build_frame_projection_cache(
  const FrameSnapshot& frame,
  const Camera2D& camera,
  const FrameViewport& viewport) -> FrameProjectionCache {
  FrameProjectionCache cache{};
  cache.bodies.reserve(frame.bodies.size());
  cache.contacts.reserve(frame.contacts.size());

  for (const SnapshotBody& body : frame.bodies) {
    ProjectedBody projected{};
    projected.center = project_point(camera, viewport, body.translation);
    if (body.shape == SnapshotShapeKind::kSphere) {
      projected.selection_anchor = projected.center;
      projected.radius_pixels = std::max(body.dimensions.x * camera.zoom, 2.0);
    } else {
      projected.outline = project_box_outline(body, camera, viewport);
      projected.selection_anchor = polygon_centroid(projected.outline);
    }

    cache.bodies.push_back(std::move(projected));
  }

  for (const SnapshotContact& contact : frame.contacts) {
    const ScreenPoint position = project_point(camera, viewport, contact.position);
    cache.contacts.push_back({
      .position = position,
      .normal_end = project_point(
        camera,
        viewport,
        {
          contact.position.x + (contact.normal.x * 0.35),
          contact.position.y,
          contact.position.z + (contact.normal.z * 0.35),
        }),
    });
  }

  return cache;
}

auto pick_body(const FrameProjectionCache& cache, const ScreenPoint& point) -> std::optional<std::size_t> {
  std::optional<std::size_t> best_index{};
  double best_distance_squared = std::numeric_limits<double>::infinity();

  for (std::size_t body_index = 0; body_index < cache.bodies.size(); ++body_index) {
    const double distance = nearest_body_distance_squared(cache.bodies[body_index], point);
    if (distance < best_distance_squared) {
      best_distance_squared = distance;
      best_index = body_index;
    }
  }

  return best_index;
}

auto pick_body(
  const FrameSnapshot& frame,
  const Camera2D& camera,
  const FrameViewport& viewport,
  const ScreenPoint& point) -> std::optional<std::size_t> {
  return pick_body(build_frame_projection_cache(frame, camera, viewport), point);
}

auto pick_contact(const FrameProjectionCache& cache, const ScreenPoint& point) -> std::optional<std::size_t> {
  std::optional<std::size_t> best_index{};
  double best_distance_squared = kSelectionThresholdPixels * kSelectionThresholdPixels;

  for (std::size_t contact_index = 0; contact_index < cache.contacts.size(); ++contact_index) {
    const double distance = distance_squared(cache.contacts[contact_index].position, point);
    if (distance <= best_distance_squared) {
      best_distance_squared = distance;
      best_index = contact_index;
    }
  }

  return best_index;
}

auto pick_contact(
  const FrameSnapshot& frame,
  const Camera2D& camera,
  const FrameViewport& viewport,
  const ScreenPoint& point) -> std::optional<std::size_t> {
  return pick_contact(build_frame_projection_cache(frame, camera, viewport), point);
}

void select_at_point(
  ViewerState& state,
  const FrameSnapshot& frame,
  const FrameProjectionCache& cache,
  const ScreenPoint& point) {
  if (const auto contact_index = pick_contact(cache, point)) {
    state.selection.contact_index = contact_index;
    state.selection.body_index.reset();
    return;
  }

  if (const auto body_index = pick_body(cache, point)) {
    state.selection.body_index = body_index;
    state.selection.contact_index.reset();
    return;
  }

  clear_selection(state);
}

void select_at_point(
  ViewerState& state,
  const FrameSnapshot& frame,
  const FrameViewport& viewport,
  const ScreenPoint& point) {
  select_at_point(
    state,
    frame,
    build_frame_projection_cache(frame, state.camera, viewport),
    point);
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

auto project_box_outline(
  const SnapshotBody& body,
  const Camera2D& camera,
  const FrameViewport& viewport) -> std::vector<ScreenPoint> {
  if (body.shape != SnapshotShapeKind::kBox) {
    return {};
  }

  std::vector<ScreenPoint> projected{};
  projected.reserve(8);
  for (const rex::math::Vec3& corner : body_box_corners(body)) {
    projected.push_back(project_point(camera, viewport, corner));
  }

  return convex_hull(std::move(projected));
}

}  // namespace rex::viewer
