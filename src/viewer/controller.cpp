#include "rex/viewer/controller.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace rex::viewer {

namespace {

constexpr double kPanDistance = 0.25;
constexpr double kZoomFactor = 1.15;
constexpr double kMinCameraDistance = 1.5;
constexpr double kMaxCameraDistance = 80.0;
constexpr double kMinPitchRadians = 0.15;
constexpr double kMaxPitchRadians = 1.45;
constexpr double kTimelineHeight = 18.0;
constexpr double kTimelineBottomPadding = 22.0;
constexpr double kSelectionThresholdPixels = 12.0;
constexpr double kProjectionTolerance = 1.0e-6;

struct Bounds3D {
  rex::math::Vec3 min{};
  rex::math::Vec3 max{};
  bool initialized{false};
};

struct BoxHit {
  double distance{std::numeric_limits<double>::infinity()};
  bool hit{false};
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
        return std::abs(lhs.x - rhs.x) <= kProjectionTolerance &&
               std::abs(lhs.y - rhs.y) <= kProjectionTolerance;
      }),
    points.end());

  if (points.size() <= 2) {
    return points;
  }

  std::vector<ScreenPoint> lower{};
  for (const ScreenPoint& point : points) {
    while (lower.size() >= 2 &&
           cross_z(lower[lower.size() - 2], lower.back(), point) <= kProjectionTolerance) {
      lower.pop_back();
    }
    lower.push_back(point);
  }

  std::vector<ScreenPoint> upper{};
  for (auto it = points.rbegin(); it != points.rend(); ++it) {
    while (upper.size() >= 2 &&
           cross_z(upper[upper.size() - 2], upper.back(), *it) <= kProjectionTolerance) {
      upper.pop_back();
    }
    upper.push_back(*it);
  }

  lower.pop_back();
  upper.pop_back();
  lower.insert(lower.end(), upper.begin(), upper.end());
  return lower;
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

[[nodiscard]] auto clamped_frame_index(std::size_t index, const ReplayLog& replay) -> std::size_t {
  if (replay.empty()) {
    return 0;
  }

  return std::min(index, replay.size() - 1);
}

void extend_bounds(Bounds3D& bounds, const rex::math::Vec3& point) {
  if (!bounds.initialized) {
    bounds.min = bounds.max = point;
    bounds.initialized = true;
    return;
  }

  bounds.min.x = std::min(bounds.min.x, point.x);
  bounds.min.y = std::min(bounds.min.y, point.y);
  bounds.min.z = std::min(bounds.min.z, point.z);
  bounds.max.x = std::max(bounds.max.x, point.x);
  bounds.max.y = std::max(bounds.max.y, point.y);
  bounds.max.z = std::max(bounds.max.z, point.z);
}

[[nodiscard]] auto frame_bounds(const FrameSnapshot& frame) -> Bounds3D {
  Bounds3D bounds{};

  for (const SnapshotBody& body : frame.bodies) {
    if (body.shape == SnapshotShapeKind::kSphere) {
      const double radius = body.dimensions.x;
      extend_bounds(bounds, body.translation + rex::math::Vec3{-radius, -radius, -radius});
      extend_bounds(bounds, body.translation + rex::math::Vec3{radius, radius, radius});
    } else {
      for (const rex::math::Vec3& corner : body_box_corners(body)) {
        extend_bounds(bounds, corner);
      }
    }
  }

  for (const SnapshotContact& contact : frame.contacts) {
    extend_bounds(bounds, contact.position);
  }

  if (!bounds.initialized) {
    extend_bounds(bounds, {-1.0, -1.0, -1.0});
    extend_bounds(bounds, {1.0, 1.0, 1.0});
  }

  return bounds;
}

[[nodiscard]] auto ray_sphere_distance(const Ray3& ray, const SnapshotBody& body) -> double {
  const rex::math::Vec3 to_center = ray.origin - body.translation;
  const double radius = body.dimensions.x;
  const double a = rex::math::dot(ray.direction, ray.direction);
  const double b = 2.0 * rex::math::dot(ray.direction, to_center);
  const double c = rex::math::dot(to_center, to_center) - (radius * radius);
  const double discriminant = (b * b) - (4.0 * a * c);
  if (discriminant < 0.0) {
    return std::numeric_limits<double>::infinity();
  }

  const double sqrt_discriminant = std::sqrt(discriminant);
  const double near_hit = (-b - sqrt_discriminant) / (2.0 * a);
  if (near_hit > 0.0) {
    return near_hit;
  }

  const double far_hit = (-b + sqrt_discriminant) / (2.0 * a);
  return far_hit > 0.0 ? far_hit : std::numeric_limits<double>::infinity();
}

[[nodiscard]] auto ray_box_distance(const Ray3& ray, const SnapshotBody& body) -> BoxHit {
  const rex::math::Transform transform{
    .rotation = body.rotation,
    .translation = body.translation,
  };
  const rex::math::Vec3 local_origin = rex::math::inverse_transform_point(transform, ray.origin);
  const rex::math::Vec3 local_direction = rex::math::inverse_rotate(body.rotation, ray.direction);

  double t_min = 0.0;
  double t_max = std::numeric_limits<double>::infinity();

  const auto update_axis = [&](double origin, double direction, double extent) -> bool {
    if (std::abs(direction) <= kProjectionTolerance) {
      return origin >= -extent && origin <= extent;
    }

    double axis_t0 = (-extent - origin) / direction;
    double axis_t1 = (extent - origin) / direction;
    if (axis_t0 > axis_t1) {
      std::swap(axis_t0, axis_t1);
    }
    t_min = std::max(t_min, axis_t0);
    t_max = std::min(t_max, axis_t1);
    return t_min <= t_max;
  };

  if (!update_axis(local_origin.x, local_direction.x, body.dimensions.x) ||
      !update_axis(local_origin.y, local_direction.y, body.dimensions.y) ||
      !update_axis(local_origin.z, local_direction.z, body.dimensions.z)) {
    return {};
  }

  const double distance = t_min > 0.0 ? t_min : t_max;
  return {
    .distance = distance > 0.0 ? distance : std::numeric_limits<double>::infinity(),
    .hit = distance > 0.0,
  };
}

[[nodiscard]] auto camera_aspect(const FrameViewport& viewport) -> double {
  const double safe_height = std::max(viewport.height, 1.0);
  return std::max(viewport.width / safe_height, 0.1);
}

[[nodiscard]] auto horizontal_fov(const Camera3D& camera, const FrameViewport& viewport) -> double {
  return 2.0 * std::atan(std::tan(camera.vertical_fov_radians * 0.5) * camera_aspect(viewport));
}

}  // namespace

auto make_viewer_state(const ReplayLog& replay) -> ViewerState {
  ViewerState state{};
  state.current_frame = replay.empty() ? 0 : 0;
  return state;
}

void fit_camera_to_frame(ViewerState& state, const FrameSnapshot& frame, const FrameViewport& viewport) {
  const Bounds3D bounds = frame_bounds(frame);
  state.camera.target = (bounds.min + bounds.max) * 0.5;
  state.camera.yaw_radians = 0.95;
  state.camera.pitch_radians = 0.55;

  double radius = 0.0;
  const auto extend_radius = [&](const rex::math::Vec3& point) {
    radius = std::max(radius, rex::math::distance(point, state.camera.target));
  };

  for (const SnapshotBody& body : frame.bodies) {
    if (body.shape == SnapshotShapeKind::kSphere) {
      extend_radius(body.translation + rex::math::Vec3{body.dimensions.x, body.dimensions.x, body.dimensions.x});
      extend_radius(body.translation - rex::math::Vec3{body.dimensions.x, body.dimensions.x, body.dimensions.x});
    } else {
      for (const rex::math::Vec3& corner : body_box_corners(body)) {
        extend_radius(corner);
      }
    }
  }
  for (const SnapshotContact& contact : frame.contacts) {
    extend_radius(contact.position);
  }

  radius = std::max(radius, 1.0);
  const double half_vertical = state.camera.vertical_fov_radians * 0.5;
  const double half_horizontal = horizontal_fov(state.camera, viewport) * 0.5;
  const double limiting_half_angle = std::max(std::min(half_vertical, half_horizontal), 0.2);
  const double fitted_distance = (radius / std::sin(limiting_half_angle)) + (viewport.margin / 75.0);
  state.camera.distance = std::clamp(fitted_distance, kMinCameraDistance, kMaxCameraDistance);
}

void clear_selection(ViewerState& state) {
  state.selection.body_index.reset();
  state.selection.contact_index.reset();
}

auto camera_forward(const Camera3D& camera) -> rex::math::Vec3 {
  const double cos_pitch = std::cos(camera.pitch_radians);
  return rex::math::normalized_or(rex::math::Vec3{
    std::cos(camera.yaw_radians) * cos_pitch,
    std::sin(camera.yaw_radians) * cos_pitch,
    std::sin(camera.pitch_radians),
  });
}

auto camera_right(const Camera3D& camera) -> rex::math::Vec3 {
  const rex::math::Vec3 world_up{0.0, 0.0, 1.0};
  return rex::math::normalized_or(rex::math::cross(camera_forward(camera), world_up), {1.0, 0.0, 0.0});
}

auto camera_up(const Camera3D& camera) -> rex::math::Vec3 {
  return rex::math::normalized_or(rex::math::cross(camera_right(camera), camera_forward(camera)), {0.0, 0.0, 1.0});
}

auto camera_position(const Camera3D& camera) -> rex::math::Vec3 {
  return camera.target - (camera_forward(camera) * camera.distance);
}

void orbit_camera(Camera3D& camera, double delta_yaw_radians, double delta_pitch_radians) {
  camera.yaw_radians += delta_yaw_radians;
  camera.pitch_radians = std::clamp(
    camera.pitch_radians + delta_pitch_radians,
    kMinPitchRadians,
    kMaxPitchRadians);
}

void pan_camera(Camera3D& camera, double delta_right, double delta_up) {
  const rex::math::Vec3 right = camera_right(camera);
  const rex::math::Vec3 up = camera_up(camera);
  camera.target += (right * delta_right) + (up * delta_up);
}

void dolly_camera(Camera3D& camera, double zoom_factor) {
  camera.distance = std::clamp(camera.distance / zoom_factor, kMinCameraDistance, kMaxCameraDistance);
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

    case ViewerCommand::kToggleTrails:
      state.overlay.show_trails = !state.overlay.show_trails;
      break;

    case ViewerCommand::kToggleGrid:
      state.overlay.show_grid = !state.overlay.show_grid;
      break;

    case ViewerCommand::kZoomIn:
      dolly_camera(state.camera, kZoomFactor);
      break;

    case ViewerCommand::kZoomOut:
      dolly_camera(state.camera, 1.0 / kZoomFactor);
      break;

    case ViewerCommand::kPanLeft:
      pan_camera(state.camera, -kPanDistance, 0.0);
      break;

    case ViewerCommand::kPanRight:
      pan_camera(state.camera, kPanDistance, 0.0);
      break;

    case ViewerCommand::kPanUp:
      pan_camera(state.camera, 0.0, kPanDistance);
      break;

    case ViewerCommand::kPanDown:
      pan_camera(state.camera, 0.0, -kPanDistance);
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

auto project_to_screen(
  const Camera3D& camera,
  const FrameViewport& viewport,
  const rex::math::Vec3& world_point) -> ProjectedPoint {
  const rex::math::Vec3 position = camera_position(camera);
  const rex::math::Vec3 forward = camera_forward(camera);
  const rex::math::Vec3 right = camera_right(camera);
  const rex::math::Vec3 up = camera_up(camera);
  const rex::math::Vec3 relative = world_point - position;

  const double view_x = rex::math::dot(relative, right);
  const double view_y = rex::math::dot(relative, up);
  const double view_z = rex::math::dot(relative, forward);
  if (view_z <= 0.01) {
    return {};
  }

  const double tan_half_fov = std::tan(camera.vertical_fov_radians * 0.5);
  const double aspect = camera_aspect(viewport);
  const double ndc_x = view_x / (view_z * tan_half_fov * aspect);
  const double ndc_y = view_y / (view_z * tan_half_fov);

  return {
    .point = {
      .x = (ndc_x + 1.0) * 0.5 * viewport.width,
      .y = (1.0 - ndc_y) * 0.5 * viewport.height,
    },
    .depth = view_z,
    .visible = true,
  };
}

auto project_point(
  const Camera3D& camera,
  const FrameViewport& viewport,
  const rex::math::Vec3& world_point) -> ScreenPoint {
  return project_to_screen(camera, viewport, world_point).point;
}

auto screen_ray(
  const Camera3D& camera,
  const FrameViewport& viewport,
  const ScreenPoint& point) -> Ray3 {
  const double aspect = camera_aspect(viewport);
  const double tan_half_fov = std::tan(camera.vertical_fov_radians * 0.5);
  const double ndc_x = ((point.x / std::max(viewport.width, 1.0)) * 2.0) - 1.0;
  const double ndc_y = 1.0 - ((point.y / std::max(viewport.height, 1.0)) * 2.0);

  const rex::math::Vec3 forward = camera_forward(camera);
  const rex::math::Vec3 right = camera_right(camera);
  const rex::math::Vec3 up = camera_up(camera);

  return {
    .origin = camera_position(camera),
    .direction = rex::math::normalized_or(
      forward + (right * (ndc_x * tan_half_fov * aspect)) + (up * (ndc_y * tan_half_fov)),
      forward),
  };
}

auto build_frame_projection_cache(
  const FrameSnapshot& frame,
  const Camera3D& camera,
  const FrameViewport& viewport) -> FrameProjectionCache {
  FrameProjectionCache cache{};
  cache.bodies.reserve(frame.bodies.size());
  cache.contacts.reserve(frame.contacts.size());

  for (const SnapshotBody& body : frame.bodies) {
    ProjectedBody projected{};
    const ProjectedPoint center = project_to_screen(camera, viewport, body.translation);
    projected.center = center.point;
    projected.selection_anchor = center.point;
    projected.depth = center.depth;
    projected.visible = center.visible;

    if (body.shape == SnapshotShapeKind::kSphere) {
      const ProjectedPoint edge = project_to_screen(
        camera,
        viewport,
        body.translation + (camera_right(camera) * body.dimensions.x));
      projected.radius_pixels =
        edge.visible && center.visible
        ? std::max(std::sqrt(distance_squared(center.point, edge.point)), 2.0)
        : 0.0;
    } else {
      projected.outline = project_box_outline(body, camera, viewport);
      projected.selection_anchor = polygon_centroid(projected.outline);
    }

    cache.bodies.push_back(std::move(projected));
  }

  for (const SnapshotContact& contact : frame.contacts) {
    const ProjectedPoint position = project_to_screen(camera, viewport, contact.position);
    const ProjectedPoint normal_end = project_to_screen(
      camera,
      viewport,
      contact.position + (contact.normal * 0.35));
    cache.contacts.push_back({
      .position = position.point,
      .normal_end = normal_end.point,
      .depth = position.depth,
      .visible = position.visible,
    });
  }

  return cache;
}

auto pick_body(const FrameProjectionCache& cache, const ScreenPoint& point) -> std::optional<std::size_t> {
  std::optional<std::size_t> best_index{};
  double best_depth = std::numeric_limits<double>::infinity();

  for (std::size_t body_index = 0; body_index < cache.bodies.size(); ++body_index) {
    const ProjectedBody& body = cache.bodies[body_index];
    if (!body.visible) {
      continue;
    }

    bool hit = false;
    if (body.outline.empty()) {
      const double radius_squared = body.radius_pixels * body.radius_pixels;
      hit = radius_squared > 0.0 && distance_squared(body.center, point) <= radius_squared;
    } else {
      hit = !body.outline.empty();
      if (body.outline.size() >= 3) {
        bool saw_positive = false;
        bool saw_negative = false;
        for (std::size_t index = 0; index < body.outline.size(); ++index) {
          const ScreenPoint& current = body.outline[index];
          const ScreenPoint& next = body.outline[(index + 1) % body.outline.size()];
          const double cross = cross_z(current, next, point);
          saw_positive = saw_positive || cross > kProjectionTolerance;
          saw_negative = saw_negative || cross < -kProjectionTolerance;
          if (saw_positive && saw_negative) {
            hit = false;
            break;
          }
        }
      } else {
        hit = false;
      }
    }

    if (hit && body.depth < best_depth) {
      best_depth = body.depth;
      best_index = body_index;
    }
  }

  return best_index;
}

auto pick_body(
  const FrameSnapshot& frame,
  const Camera3D& camera,
  const FrameViewport& viewport,
  const ScreenPoint& point) -> std::optional<std::size_t> {
  const Ray3 ray = screen_ray(camera, viewport, point);
  std::optional<std::size_t> best_index{};
  double best_distance = std::numeric_limits<double>::infinity();

  for (std::size_t body_index = 0; body_index < frame.bodies.size(); ++body_index) {
    const SnapshotBody& body = frame.bodies[body_index];
    const double distance =
      body.shape == SnapshotShapeKind::kSphere
      ? ray_sphere_distance(ray, body)
      : ray_box_distance(ray, body).distance;
    if (distance < best_distance) {
      best_distance = distance;
      best_index = body_index;
    }
  }

  return best_index;
}

auto pick_contact(const FrameProjectionCache& cache, const ScreenPoint& point) -> std::optional<std::size_t> {
  std::optional<std::size_t> best_index{};
  double best_depth = std::numeric_limits<double>::infinity();
  double best_distance_squared = kSelectionThresholdPixels * kSelectionThresholdPixels;

  for (std::size_t contact_index = 0; contact_index < cache.contacts.size(); ++contact_index) {
    const ProjectedContact& contact = cache.contacts[contact_index];
    if (!contact.visible) {
      continue;
    }

    const double distance = distance_squared(contact.position, point);
    if (distance <= best_distance_squared && contact.depth < best_depth) {
      best_distance_squared = distance;
      best_depth = contact.depth;
      best_index = contact_index;
    }
  }

  return best_index;
}

auto pick_contact(
  const FrameSnapshot& frame,
  const Camera3D& camera,
  const FrameViewport& viewport,
  const ScreenPoint& point) -> std::optional<std::size_t> {
  return pick_contact(build_frame_projection_cache(frame, camera, viewport), point);
}

void select_at_point(
  ViewerState& state,
  const FrameSnapshot&,
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

auto project_box_outline(
  const SnapshotBody& body,
  const Camera3D& camera,
  const FrameViewport& viewport) -> std::vector<ScreenPoint> {
  if (body.shape != SnapshotShapeKind::kBox) {
    return {};
  }

  std::vector<ScreenPoint> projected{};
  projected.reserve(8);
  for (const rex::math::Vec3& corner : body_box_corners(body)) {
    const ProjectedPoint point = project_to_screen(camera, viewport, corner);
    if (point.visible) {
      projected.push_back(point.point);
    }
  }

  return convex_hull(std::move(projected));
}

}  // namespace rex::viewer
