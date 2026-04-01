#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

#include "rex/geometry/shapes.hpp"
#include "rex/sim/engine.hpp"
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
  double inverse_mass = 1.0) -> rex::dynamics::BodyState {
  return {
    .id = rex::platform::EntityId{.index = index, .generation = 1},
    .pose = rex::math::Transform{.translation = translation},
    .inverse_mass = inverse_mass,
    .shape = rex::geometry::Shape{.data = rex::geometry::Box{.half_extents = half_extents}},
  };
}

void test_capture_frame_extracts_shapes_and_contacts() {
  rex::sim::EngineConfig config{};
  config.simulation.gravity = {0.0, 0.0, 0.0};
  rex::sim::Engine engine{config};

  rex::dynamics::WorldState world{};
  const std::size_t box_index =
    world.bodies.add_body(make_box_body(9, {0.0, 0.0, 0.0}, {0.5, 0.5, 0.5}, 0.0));
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
  expect(frame.trace.manifold_count == 1, "frame trace should keep solver metadata");
}

void test_replay_round_trip() {
  rex::viewer::ReplayLog replay{};
  rex::viewer::FrameSnapshot frame{};
  frame.frame_index = 3;
  frame.sim_time = 0.05;
  frame.trace.body_count = 1;
  frame.trace.solver.contact_count = 1;
  frame.bodies.push_back({
    .id = rex::platform::EntityId{.index = 4, .generation = 2},
    .shape = rex::viewer::SnapshotShapeKind::kSphere,
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
  expect(svg.find("<rect") != std::string::npos, "svg output should contain a box primitive");
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

}  // namespace

int main() {
  test_capture_frame_extracts_shapes_and_contacts();
  test_replay_round_trip();
  test_svg_render_includes_shapes_and_contacts();
  test_demo_replay_has_frames_and_contacts();
  std::cout << "all viewer tests passed\n";
  return 0;
}
