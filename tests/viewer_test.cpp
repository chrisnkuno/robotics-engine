#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "rex/geometry/shapes.hpp"
#include "rex/sim/engine.hpp"
#include "rex/viewer/controller.hpp"
#include "rex/viewer/demo.hpp"
#include "rex/viewer/replay.hpp"
#include "rex/viewer/svg_renderer.hpp"

namespace {

void expect(bool condition, const std::string& message) {
  if (!condition) {
    std::cerr << "test failure: " << message << '\n';
    std::exit(1);
  }
}

auto nearly_equal(double lhs, double rhs, double tolerance = 1.0e-9) -> bool {
  return std::abs(lhs - rhs) <= tolerance;
}

void expect_between(double value, double lower, double upper, const std::string& message) {
  expect(value >= lower - 1.0e-9 && value <= upper + 1.0e-9, message);
}

void expect_quat_equivalent(
  const rex::math::Quat& actual,
  const rex::math::Quat& expected,
  double tolerance,
  const std::string& message) {
  const double alignment =
    std::abs((actual.w * expected.w) + (actual.x * expected.x) + (actual.y * expected.y) + (actual.z * expected.z));
  expect(nearly_equal(alignment, 1.0, tolerance), message);
}

auto make_sphere_body(
  std::uint32_t index,
  const rex::math::Vec3& translation,
  double radius,
  double inverse_mass = 1.0) -> rex::dynamics::BodyState {
  return {
    .id = rex::platform::EntityId{.index = index, .generation = 1},
    .pose = rex::math::Transform{.translation = translation},
    .inverse_mass = inverse_mass,
    .shape = rex::geometry::Shape{.data = rex::geometry::Sphere{.radius = radius}},
  };
}

auto make_box_body(
  std::uint32_t index,
  const rex::math::Vec3& translation,
  const rex::math::Vec3& half_extents,
  double inverse_mass = 1.0,
  const rex::math::Quat& rotation = {}) -> rex::dynamics::BodyState {
  return {
    .id = rex::platform::EntityId{.index = index, .generation = 1},
    .pose = rex::math::Transform{.rotation = rotation, .translation = translation},
    .inverse_mass = inverse_mass,
    .shape = rex::geometry::Shape{.data = rex::geometry::Box{.half_extents = half_extents}},
  };
}

void test_capture_frame_extracts_shapes_and_contacts() {
  rex::sim::EngineConfig config{};
  config.simulation.gravity = {0.0, 0.0, 0.0};
  rex::sim::Engine engine{config};

  rex::dynamics::WorldState world{};
  const rex::math::Quat box_rotation = rex::math::quat_from_axis_angle({0.0, 1.0, 0.0}, 0.35);
  const std::size_t box_index =
    world.bodies.add_body(make_box_body(9, {0.0, 0.0, 0.0}, {0.5, 0.5, 0.5}, 0.0, box_rotation));
  const std::size_t sphere_index =
    world.bodies.add_body(make_sphere_body(4, {1.2, 0.0, 0.0}, 0.8));

  const rex::sim::StepTrace trace = engine.step(world);
  const rex::viewer::FrameSnapshot frame = rex::viewer::capture_frame(world, trace, 7, 0.125);

  expect(box_index == 0, "box index should be stable");
  expect(sphere_index == 1, "sphere index should be stable");
  expect(frame.frame_index == 7, "frame index should be preserved");
  expect(nearly_equal(frame.sim_time, 0.125), "sim time should be preserved");
  expect(frame.bodies.size() == 2, "frame should capture two bodies");
  expect(frame.contacts.size() == 1, "frame should capture one contact");
  expect(frame.bodies[0].shape == rex::viewer::SnapshotShapeKind::kBox, "first body should be a box");
  expect(frame.bodies[1].shape == rex::viewer::SnapshotShapeKind::kSphere, "second body should be a sphere");
  expect_quat_equivalent(frame.bodies[0].rotation, box_rotation, 1.0e-9, "capture should preserve box rotation");
  expect(frame.trace.manifold_count == 1, "frame trace should keep solver metadata");
}

void test_replay_round_trip() {
  rex::viewer::ReplayLog replay{};
  rex::viewer::FrameSnapshot frame{};
  frame.frame_index = 3;
  frame.sim_time = 0.05;
  frame.trace.body_count = 1;
  frame.trace.solver.contact_count = 1;
  frame.trace.profile.integrate_ms = 0.1;
  frame.trace.profile.collision_ms = 0.2;
  frame.trace.profile.solver_ms = 0.3;
  frame.trace.profile.total_ms = 0.7;
  frame.bodies.push_back({
      .id = rex::platform::EntityId{.index = 4, .generation = 2},
      .shape = rex::viewer::SnapshotShapeKind::kSphere,
      .rotation = rex::math::quat_from_axis_angle({0.0, 1.0, 0.0}, 0.2),
      .translation = {1.0, 0.0, 2.0},
      .dimensions = {0.6, 0.0, 0.0},
    });
  frame.contacts.push_back({
    .body_a = rex::platform::EntityId{.index = 4, .generation = 2},
    .body_b = rex::platform::EntityId{.index = 9, .generation = 1},
    .position = {0.5, 0.0, 1.5},
    .normal = {-1.0, 0.0, 0.0},
    .penetration = 0.25,
  });
  replay.add_frame(frame);

  const std::filesystem::path replay_path = std::filesystem::temp_directory_path() / "rex-viewer-test.rex";
  replay.save(replay_path);
  const rex::viewer::ReplayLog loaded = rex::viewer::ReplayLog::load(replay_path);

  expect(loaded.size() == 1, "loaded replay should contain one frame");
  expect(loaded.frames()[0].bodies.size() == 1, "loaded replay should keep body records");
  expect(loaded.frames()[0].contacts.size() == 1, "loaded replay should keep contact records");
  expect(loaded.frames()[0].bodies[0].shape == rex::viewer::SnapshotShapeKind::kSphere, "shape kind should round-trip");
  expect_quat_equivalent(
    loaded.frames()[0].bodies[0].rotation,
    frame.bodies[0].rotation,
    1.0e-9,
    "body rotation should round-trip through replay serialization");
  expect(nearly_equal(loaded.frames()[0].trace.profile.total_ms, 0.7), "step profile should round-trip through replay serialization");
  expect(nearly_equal(loaded.frames()[0].contacts[0].penetration, 0.25), "penetration should round-trip");
}

void test_svg_render_includes_shapes_and_contacts() {
  rex::viewer::FrameSnapshot frame{};
  frame.frame_index = 1;
  frame.sim_time = 0.0;
  frame.trace.body_count = 2;
  frame.trace.solver.contact_count = 1;
  frame.bodies.push_back({
    .id = rex::platform::EntityId{.index = 9, .generation = 1},
    .shape = rex::viewer::SnapshotShapeKind::kBox,
    .rotation = rex::math::quat_from_axis_angle({0.0, 1.0, 0.0}, 0.5),
    .translation = {0.0, 0.0, 0.0},
    .dimensions = {0.5, 0.5, 0.5},
  });
  frame.bodies.push_back({
    .id = rex::platform::EntityId{.index = 4, .generation = 1},
    .shape = rex::viewer::SnapshotShapeKind::kSphere,
    .translation = {1.2, 0.0, 0.0},
    .dimensions = {0.8, 0.0, 0.0},
  });
  frame.contacts.push_back({
    .body_a = rex::platform::EntityId{.index = 4, .generation = 1},
    .body_b = rex::platform::EntityId{.index = 9, .generation = 1},
    .position = {0.5, 0.0, 0.0},
    .normal = {-1.0, 0.0, 0.0},
    .penetration = 0.1,
  });

  const std::string svg = rex::viewer::render_frame_svg(frame);
  expect(svg.find("<svg") != std::string::npos, "svg output should contain root element");
  expect(svg.find("<polygon") != std::string::npos, "svg output should contain a rotated box polygon");
  expect(svg.find("<circle") != std::string::npos, "svg output should contain circle primitives");
  expect(svg.find("frame=1") != std::string::npos, "svg output should include frame metadata");
}

void test_demo_replay_has_frames_and_contacts() {
  const rex::viewer::ReplayLog replay = rex::viewer::build_demo_replay(4);

  expect(replay.size() == 4, "demo replay should generate the requested number of frames");
  expect(!replay.frames().front().bodies.empty(), "demo replay should include bodies");

  bool found_contact = false;
  for (const auto& frame : replay.frames()) {
    found_contact = found_contact || !frame.contacts.empty();
  }

  expect(found_contact, "demo replay should include at least one visible contact");
}

void test_viewer_controller_playback_and_step_commands() {
  const rex::viewer::ReplayLog replay = rex::viewer::build_demo_replay(5);
  rex::viewer::ViewerState state = rex::viewer::make_viewer_state(replay);
  const rex::viewer::FrameViewport viewport{};

  expect(state.current_frame == 0, "viewer should start on the first frame");
  expect(state.playback == rex::viewer::PlaybackMode::kPaused, "viewer should start paused");

  rex::viewer::apply_command(state, replay, rex::viewer::ViewerCommand::kStepForward, viewport);
  expect(state.current_frame == 1, "step forward should advance one frame");

  rex::viewer::apply_command(state, replay, rex::viewer::ViewerCommand::kStepBackward, viewport);
  expect(state.current_frame == 0, "step backward should rewind one frame");

  rex::viewer::apply_command(state, replay, rex::viewer::ViewerCommand::kTogglePlayPause, viewport);
  expect(state.playback == rex::viewer::PlaybackMode::kPlaying, "toggle should enter playback");

  rex::viewer::update_playback(state, replay, 1.0 / 15.0);
  expect(state.current_frame >= 1, "playback update should advance frames over time");

  rex::viewer::apply_command(state, replay, rex::viewer::ViewerCommand::kTogglePlayPause, viewport);
  expect(state.playback == rex::viewer::PlaybackMode::kPaused, "toggle should pause playback");
}

void test_viewer_controller_camera_fit_and_projection() {
  rex::viewer::FrameSnapshot frame{};
  frame.bodies.push_back({
    .id = rex::platform::EntityId{.index = 1, .generation = 1},
    .shape = rex::viewer::SnapshotShapeKind::kBox,
    .translation = {-1.0, 0.0, 0.0},
    .dimensions = {0.5, 0.5, 0.5},
  });
  frame.bodies.push_back({
    .id = rex::platform::EntityId{.index = 2, .generation = 1},
    .shape = rex::viewer::SnapshotShapeKind::kSphere,
    .translation = {1.0, 0.0, 1.0},
    .dimensions = {0.5, 0.0, 0.0},
  });

  rex::viewer::ViewerState state{};
  const rex::viewer::FrameViewport viewport{.width = 1000.0, .height = 800.0, .margin = 100.0};
  rex::viewer::fit_camera_to_frame(state, frame, viewport);

  expect(state.camera.distance > 0.0, "camera fit should produce a positive camera distance");
  expect(nearly_equal(state.camera.target.x, 0.0), "camera fit should center the x range");
  expect(nearly_equal(state.camera.target.z, 0.5), "camera fit should center the z range");

  const rex::viewer::ScreenPoint projected =
    rex::viewer::project_point(state.camera, viewport, state.camera.target);
  expect(nearly_equal(projected.x, viewport.width * 0.5), "projection should map center x to viewport center");
  expect(nearly_equal(projected.y, viewport.height * 0.5), "projection should map center z to viewport center");
}

void test_viewer_controller_overlay_toggles() {
  const rex::viewer::ReplayLog replay = rex::viewer::build_demo_replay(2);
  rex::viewer::ViewerState state = rex::viewer::make_viewer_state(replay);
  const rex::viewer::FrameViewport viewport{};

  rex::viewer::apply_command(state, replay, rex::viewer::ViewerCommand::kToggleContacts, viewport);
  expect(!state.overlay.show_contacts, "contact toggle should hide contacts");

  rex::viewer::apply_command(state, replay, rex::viewer::ViewerCommand::kToggleNormals, viewport);
  expect(!state.overlay.show_normals, "normal toggle should hide normals");

  const double initial_distance = state.camera.distance;
  rex::viewer::apply_command(state, replay, rex::viewer::ViewerCommand::kZoomIn, viewport);
  expect(state.camera.distance < initial_distance, "zoom in should dolly the camera closer");

  rex::viewer::apply_command(state, replay, rex::viewer::ViewerCommand::kPanRight, viewport);
  expect(state.camera.target.x > 0.0, "pan right should move the camera target");
}

void test_viewer_camera_fit_keeps_geometry_inside_margin() {
  rex::viewer::FrameSnapshot frame{};
  frame.bodies.push_back({
    .id = rex::platform::EntityId{.index = 11, .generation = 1},
    .shape = rex::viewer::SnapshotShapeKind::kBox,
    .translation = {-1.6, 0.0, -0.2},
    .dimensions = {0.7, 0.5, 0.4},
  });
  frame.bodies.push_back({
    .id = rex::platform::EntityId{.index = 14, .generation = 1},
    .shape = rex::viewer::SnapshotShapeKind::kSphere,
    .translation = {1.4, 0.0, 1.1},
    .dimensions = {0.6, 0.0, 0.0},
  });

  rex::viewer::ViewerState state{};
  const rex::viewer::FrameViewport viewport{.width = 1200.0, .height = 900.0, .margin = 120.0};
  rex::viewer::fit_camera_to_frame(state, frame, viewport);

  const std::vector<rex::math::Vec3> extremal_points = {
    {-2.3, 0.0, -0.6},
    {-0.9, 0.0, 0.2},
    {0.8, 0.0, 0.5},
    {2.0, 0.0, 1.7},
  };

  for (const auto& point : extremal_points) {
    const rex::viewer::ScreenPoint projected = rex::viewer::project_point(state.camera, viewport, point);
    expect_between(projected.x, viewport.margin, viewport.width - viewport.margin, "camera fit should keep x within margins");
    expect_between(projected.y, viewport.margin, viewport.height - viewport.margin, "camera fit should keep y within margins");
  }
}

void test_viewer_timeline_mapping_and_scrubbing() {
  const rex::viewer::ReplayLog replay = rex::viewer::build_demo_replay(5);
  rex::viewer::ViewerState state = rex::viewer::make_viewer_state(replay);
  const rex::viewer::FrameViewport viewport{.width = 1000.0, .height = 800.0, .margin = 80.0};
  const rex::viewer::TimelineRect timeline = rex::viewer::timeline_rect(viewport);

  expect(rex::viewer::frame_index_for_timeline(replay, timeline, timeline.left) == 0, "timeline left edge should map to the first frame");
  expect(
    rex::viewer::frame_index_for_timeline(replay, timeline, timeline.left + timeline.width) == replay.size() - 1,
    "timeline right edge should map to the last frame");
  expect(
    rex::viewer::frame_index_for_timeline(replay, timeline, timeline.left + (timeline.width * 0.5)) == 2,
    "timeline midpoint should map to the middle frame");

  state.playback = rex::viewer::PlaybackMode::kPlaying;
  state.selection.body_index = 0;
  rex::viewer::scrub_to_timeline(state, replay, viewport, timeline.left + timeline.width);
  expect(state.current_frame == replay.size() - 1, "scrubbing should update the current frame");
  expect(state.playback == rex::viewer::PlaybackMode::kPaused, "scrubbing should pause playback for inspection");
  expect(!state.selection.body_index.has_value(), "scrubbing should clear stale selection state");
}

void test_viewer_selection_prefers_contacts_then_bodies() {
  rex::viewer::FrameSnapshot frame{};
  frame.bodies.push_back({
    .id = rex::platform::EntityId{.index = 21, .generation = 1},
    .shape = rex::viewer::SnapshotShapeKind::kBox,
    .translation = {0.0, 0.0, 0.0},
    .dimensions = {0.5, 0.5, 0.5},
  });
  frame.bodies.push_back({
    .id = rex::platform::EntityId{.index = 22, .generation = 1},
    .shape = rex::viewer::SnapshotShapeKind::kSphere,
    .translation = {1.2, 0.0, 0.0},
    .dimensions = {0.8, 0.0, 0.0},
  });
  frame.contacts.push_back({
    .body_a = rex::platform::EntityId{.index = 21, .generation = 1},
    .body_b = rex::platform::EntityId{.index = 22, .generation = 1},
    .position = {0.5, 0.0, 0.0},
    .normal = {1.0, 0.0, 0.0},
    .penetration = 0.1,
  });

  rex::viewer::ViewerState state{};
  const rex::viewer::FrameViewport viewport{.width = 1000.0, .height = 800.0, .margin = 80.0};
  rex::viewer::fit_camera_to_frame(state, frame, viewport);

  const rex::viewer::ScreenPoint contact_point =
    rex::viewer::project_point(state.camera, viewport, frame.contacts[0].position);
  rex::viewer::select_at_point(state, frame, viewport, contact_point);
  expect(state.selection.contact_index == std::optional<std::size_t>{0}, "contact picks should win when the click hits a contact");
  expect(!state.selection.body_index.has_value(), "contact selection should clear body selection");

  const rex::viewer::ScreenPoint body_point =
    rex::viewer::project_point(state.camera, viewport, frame.bodies[0].translation);
  rex::viewer::select_at_point(state, frame, viewport, body_point);
  expect(state.selection.body_index == std::optional<std::size_t>{0}, "body picks should identify the hit body");
  expect(!state.selection.contact_index.has_value(), "body selection should clear contact selection");

  rex::viewer::select_at_point(state, frame, viewport, {.x = 20.0, .y = 20.0});
  expect(!state.selection.body_index.has_value(), "clicking empty space should clear body selection");
  expect(!state.selection.contact_index.has_value(), "clicking empty space should clear contact selection");
}

void test_rotated_box_projection_and_selection_use_quaternion_pose() {
  rex::viewer::FrameSnapshot frame{};
  frame.bodies.push_back({
    .id = rex::platform::EntityId{.index = 31, .generation = 1},
    .shape = rex::viewer::SnapshotShapeKind::kBox,
    .rotation = rex::math::quat_from_axis_angle({0.0, 1.0, 0.0}, 0.6),
    .translation = {0.4, 0.0, 0.2},
    .dimensions = {0.6, 0.4, 0.3},
  });

  rex::viewer::ViewerState state{};
  const rex::viewer::FrameViewport viewport{.width = 1000.0, .height = 800.0, .margin = 80.0};
  rex::viewer::fit_camera_to_frame(state, frame, viewport);

  const std::vector<rex::viewer::ScreenPoint> outline =
    rex::viewer::project_box_outline(frame.bodies[0], state.camera, viewport);
  expect(outline.size() >= 4, "rotated box projection should produce a polygon outline");

  const rex::viewer::ScreenPoint center =
    rex::viewer::project_point(state.camera, viewport, frame.bodies[0].translation);
  rex::viewer::select_at_point(state, frame, viewport, center);
  expect(state.selection.body_index == std::optional<std::size_t>{0}, "rotated box selection should use the projected polygon");
}

void test_projection_cache_matches_direct_projection_and_selection() {
  rex::viewer::FrameSnapshot frame{};
  frame.bodies.push_back({
    .id = rex::platform::EntityId{.index = 41, .generation = 1},
    .shape = rex::viewer::SnapshotShapeKind::kBox,
    .rotation = rex::math::quat_from_axis_angle({0.0, 1.0, 0.0}, 0.45),
    .translation = {0.3, 0.0, -0.2},
    .dimensions = {0.7, 0.3, 0.4},
  });
  frame.bodies.push_back({
    .id = rex::platform::EntityId{.index = 42, .generation = 1},
    .shape = rex::viewer::SnapshotShapeKind::kSphere,
    .translation = {1.3, 0.0, 0.2},
    .dimensions = {0.5, 0.0, 0.0},
  });
  frame.contacts.push_back({
    .body_a = rex::platform::EntityId{.index = 41, .generation = 1},
    .body_b = rex::platform::EntityId{.index = 42, .generation = 1},
    .position = {0.9, 0.0, 0.0},
    .normal = {1.0, 0.0, 0.0},
    .penetration = 0.08,
  });

  rex::viewer::ViewerState state{};
  const rex::viewer::FrameViewport viewport{.width = 1000.0, .height = 800.0, .margin = 80.0};
  rex::viewer::fit_camera_to_frame(state, frame, viewport);

  const rex::viewer::FrameProjectionCache cache =
    rex::viewer::build_frame_projection_cache(frame, state.camera, viewport);
  const std::vector<rex::viewer::ScreenPoint> direct_outline =
    rex::viewer::project_box_outline(frame.bodies[0], state.camera, viewport);

  expect(cache.bodies.size() == frame.bodies.size(), "projection cache should keep one entry per body");
  expect(cache.contacts.size() == frame.contacts.size(), "projection cache should keep one entry per contact");
  expect(cache.bodies[0].outline.size() == direct_outline.size(), "cached box outline should match direct projection");
  for (std::size_t index = 0; index < direct_outline.size(); ++index) {
    expect(nearly_equal(cache.bodies[0].outline[index].x, direct_outline[index].x, 1.0e-9), "cached box outline x should match direct projection");
    expect(nearly_equal(cache.bodies[0].outline[index].y, direct_outline[index].y, 1.0e-9), "cached box outline y should match direct projection");
  }

  const rex::viewer::ScreenPoint box_point =
    rex::viewer::project_point(state.camera, viewport, frame.bodies[0].translation);
  const rex::viewer::ScreenPoint contact_point =
    rex::viewer::project_point(state.camera, viewport, frame.contacts[0].position);
  expect(
    rex::viewer::pick_body(cache, box_point) ==
      rex::viewer::pick_body(frame, state.camera, viewport, box_point),
    "cached body picking should match direct picking");
  expect(
    rex::viewer::pick_contact(cache, contact_point) ==
      rex::viewer::pick_contact(frame, state.camera, viewport, contact_point),
    "cached contact picking should match direct picking");
}

void test_demo_scene_runner_produces_monotonic_frames() {
  rex::viewer::DemoSceneRunner runner{};
  const rex::viewer::FrameSnapshot first = runner.step_frame();
  const rex::viewer::FrameSnapshot second = runner.step_frame();

  expect(first.frame_index == 0, "demo runner should start at frame zero");
  expect(second.frame_index == 1, "demo runner should advance frame indices monotonically");
  expect(second.sim_time > first.sim_time, "demo runner should advance simulation time");
  expect(first.bodies.size() == second.bodies.size(), "demo runner should keep body count stable");
}

}  // namespace

int main() {
  test_capture_frame_extracts_shapes_and_contacts();
  test_replay_round_trip();
  test_svg_render_includes_shapes_and_contacts();
  test_demo_replay_has_frames_and_contacts();
  test_viewer_controller_playback_and_step_commands();
  test_viewer_controller_camera_fit_and_projection();
  test_viewer_controller_overlay_toggles();
  test_viewer_camera_fit_keeps_geometry_inside_margin();
  test_viewer_timeline_mapping_and_scrubbing();
  test_viewer_selection_prefers_contacts_then_bodies();
  test_rotated_box_projection_and_selection_use_quaternion_pose();
  test_projection_cache_matches_direct_projection_and_selection();
  test_demo_scene_runner_produces_monotonic_frames();
  std::cout << "all viewer tests passed\n";
  return 0;
}
