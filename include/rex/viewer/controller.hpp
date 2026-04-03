#pragma once

#include <cstddef>
#include <optional>
#include <vector>

#include "rex/viewer/replay.hpp"

namespace rex::viewer {

enum class PlaybackMode {
  kPaused,
  kPlaying,
};

enum class ViewerCommand {
  kTogglePlayPause,
  kStepForward,
  kStepBackward,
  kToggleContacts,
  kToggleNormals,
  kToggleTrails,
  kToggleGrid,
  kZoomIn,
  kZoomOut,
  kPanLeft,
  kPanRight,
  kPanUp,
  kPanDown,
  kResetCamera,
};

struct Camera3D {
  rex::math::Vec3 target{};
  double distance{7.5};
  double yaw_radians{0.9};
  double pitch_radians{0.55};
  double vertical_fov_radians{0.95};
};

struct OverlayState {
  bool show_contacts{true};
  bool show_normals{true};
  bool show_trails{true};
  bool show_grid{true};
};

struct FrameViewport {
  double width{960.0};
  double height{720.0};
  double margin{60.0};
};

struct ScreenPoint {
  double x{0.0};
  double y{0.0};
};

struct TimelineRect {
  double left{0.0};
  double top{0.0};
  double width{0.0};
  double height{0.0};
};

struct Ray3 {
  rex::math::Vec3 origin{};
  rex::math::Vec3 direction{0.0, 1.0, 0.0};
};

struct ProjectedPoint {
  ScreenPoint point{};
  double depth{0.0};
  bool visible{false};
};

struct ProjectedBody {
  ScreenPoint center{};
  ScreenPoint selection_anchor{};
  double radius_pixels{0.0};
  double depth{0.0};
  bool visible{false};
  std::vector<ScreenPoint> outline{};
};

struct ProjectedContact {
  ScreenPoint position{};
  ScreenPoint normal_end{};
  double depth{0.0};
  bool visible{false};
};

struct FrameProjectionCache {
  std::vector<ProjectedBody> bodies{};
  std::vector<ProjectedContact> contacts{};
};

struct SelectionState {
  std::optional<std::size_t> body_index{};
  std::optional<std::size_t> contact_index{};
};

struct ViewerState {
  PlaybackMode playback{PlaybackMode::kPaused};
  std::size_t current_frame{0};
  double accumulated_playback_time{0.0};
  double playback_fps{30.0};
  Camera3D camera{};
  OverlayState overlay{};
  SelectionState selection{};
};

[[nodiscard]] auto make_viewer_state(const ReplayLog& replay) -> ViewerState;
void fit_camera_to_frame(ViewerState& state, const FrameSnapshot& frame, const FrameViewport& viewport);
void apply_command(ViewerState& state, const ReplayLog& replay, ViewerCommand command, const FrameViewport& viewport);
void update_playback(ViewerState& state, const ReplayLog& replay, double delta_seconds);
void clear_selection(ViewerState& state);

[[nodiscard]] auto timeline_rect(const FrameViewport& viewport) -> TimelineRect;
[[nodiscard]] auto point_in_timeline(const TimelineRect& timeline, const ScreenPoint& point) -> bool;
[[nodiscard]] auto frame_index_for_timeline(const ReplayLog& replay, const TimelineRect& timeline, double x)
  -> std::size_t;
void scrub_to_timeline(ViewerState& state, const ReplayLog& replay, const FrameViewport& viewport, double x);

[[nodiscard]] auto camera_position(const Camera3D& camera) -> rex::math::Vec3;
[[nodiscard]] auto camera_forward(const Camera3D& camera) -> rex::math::Vec3;
[[nodiscard]] auto camera_right(const Camera3D& camera) -> rex::math::Vec3;
[[nodiscard]] auto camera_up(const Camera3D& camera) -> rex::math::Vec3;
void orbit_camera(Camera3D& camera, double delta_yaw_radians, double delta_pitch_radians);
void pan_camera(Camera3D& camera, double delta_right, double delta_up);
void dolly_camera(Camera3D& camera, double zoom_factor);

[[nodiscard]] auto project_to_screen(
  const Camera3D& camera,
  const FrameViewport& viewport,
  const rex::math::Vec3& world_point) -> ProjectedPoint;
[[nodiscard]] auto project_point(
  const Camera3D& camera,
  const FrameViewport& viewport,
  const rex::math::Vec3& world_point) -> ScreenPoint;
[[nodiscard]] auto screen_ray(
  const Camera3D& camera,
  const FrameViewport& viewport,
  const ScreenPoint& point) -> Ray3;

[[nodiscard]] auto build_frame_projection_cache(
  const FrameSnapshot& frame,
  const Camera3D& camera,
  const FrameViewport& viewport) -> FrameProjectionCache;

[[nodiscard]] auto pick_body(
  const FrameProjectionCache& cache,
  const ScreenPoint& point) -> std::optional<std::size_t>;
[[nodiscard]] auto pick_body(
  const FrameSnapshot& frame,
  const Camera3D& camera,
  const FrameViewport& viewport,
  const ScreenPoint& point) -> std::optional<std::size_t>;
[[nodiscard]] auto pick_contact(
  const FrameProjectionCache& cache,
  const ScreenPoint& point) -> std::optional<std::size_t>;
[[nodiscard]] auto pick_contact(
  const FrameSnapshot& frame,
  const Camera3D& camera,
  const FrameViewport& viewport,
  const ScreenPoint& point) -> std::optional<std::size_t>;
void select_at_point(
  ViewerState& state,
  const FrameSnapshot& frame,
  const FrameProjectionCache& cache,
  const ScreenPoint& point);
void select_at_point(
  ViewerState& state,
  const FrameSnapshot& frame,
  const FrameViewport& viewport,
  const ScreenPoint& point);

[[nodiscard]] auto project_box_outline(
  const SnapshotBody& body,
  const Camera3D& camera,
  const FrameViewport& viewport) -> std::vector<ScreenPoint>;

}  // namespace rex::viewer
