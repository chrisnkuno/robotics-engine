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
  kZoomIn,
  kZoomOut,
  kPanLeft,
  kPanRight,
  kPanUp,
  kPanDown,
  kResetCamera,
};

struct Camera2D {
  double center_x{0.0};
  double center_z{0.0};
  double zoom{140.0};
};

struct OverlayState {
  bool show_contacts{true};
  bool show_normals{true};
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

struct ProjectedBody {
  ScreenPoint center{};
  ScreenPoint selection_anchor{};
  double radius_pixels{0.0};
  std::vector<ScreenPoint> outline{};
};

struct ProjectedContact {
  ScreenPoint position{};
  ScreenPoint normal_end{};
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
  Camera2D camera{};
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

[[nodiscard]] auto build_frame_projection_cache(
  const FrameSnapshot& frame,
  const Camera2D& camera,
  const FrameViewport& viewport) -> FrameProjectionCache;

[[nodiscard]] auto pick_body(
  const FrameProjectionCache& cache,
  const ScreenPoint& point) -> std::optional<std::size_t>;
[[nodiscard]] auto pick_body(
  const FrameSnapshot& frame,
  const Camera2D& camera,
  const FrameViewport& viewport,
  const ScreenPoint& point) -> std::optional<std::size_t>;
[[nodiscard]] auto pick_contact(
  const FrameProjectionCache& cache,
  const ScreenPoint& point) -> std::optional<std::size_t>;
[[nodiscard]] auto pick_contact(
  const FrameSnapshot& frame,
  const Camera2D& camera,
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

[[nodiscard]] auto project_point(
  const Camera2D& camera,
  const FrameViewport& viewport,
  const rex::math::Vec3& world_point) -> ScreenPoint;
[[nodiscard]] auto project_box_outline(
  const SnapshotBody& body,
  const Camera2D& camera,
  const FrameViewport& viewport) -> std::vector<ScreenPoint>;

}  // namespace rex::viewer
