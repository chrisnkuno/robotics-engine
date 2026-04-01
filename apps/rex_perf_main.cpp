#include <chrono>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <string_view>

#include "rex/geometry/shapes.hpp"
#include "rex/sim/engine.hpp"
#include "rex/viewer/controller.hpp"
#include "rex/viewer/demo.hpp"

namespace {

auto make_sphere_body(
  std::uint32_t index,
  const rex::math::Vec3& translation,
  double radius,
  const rex::math::Vec3& linear_velocity = {}) -> rex::dynamics::BodyState {
  return {
    .id = rex::platform::EntityId{.index = index, .generation = 1},
    .pose = rex::math::Transform{.translation = translation},
    .linear_velocity = linear_velocity,
    .inverse_mass = 1.0,
    .shape = rex::geometry::Shape{.data = rex::geometry::Sphere{.radius = radius}},
  };
}

auto make_box_body(
  std::uint32_t index,
  const rex::math::Vec3& translation,
  const rex::math::Vec3& half_extents,
  double inverse_mass = 0.0) -> rex::dynamics::BodyState {
  return {
    .id = rex::platform::EntityId{.index = index, .generation = 1},
    .pose = rex::math::Transform{.translation = translation},
    .inverse_mass = inverse_mass,
    .shape = rex::geometry::Shape{.data = rex::geometry::Box{.half_extents = half_extents}},
  };
}

auto build_benchmark_world(std::size_t dynamic_body_count) -> rex::dynamics::WorldState {
  rex::dynamics::WorldState world{};
  world.bodies.reserve(dynamic_body_count + 1);
  const std::size_t ground_index =
    world.bodies.add_body(make_box_body(1, {0.0, 0.0, -0.6}, {6.0, 6.0, 0.5}, 0.0));
  (void)ground_index;

  constexpr double kRadius = 0.35;
  const std::size_t side = std::max<std::size_t>(1, static_cast<std::size_t>(std::ceil(std::sqrt(dynamic_body_count))));
  for (std::size_t body_index = 0; body_index < dynamic_body_count; ++body_index) {
    const std::size_t x_index = body_index % side;
    const std::size_t z_index = body_index / side;
    const double x = (static_cast<double>(x_index) - (static_cast<double>(side) * 0.5)) * 0.72;
    const double y = (body_index % 3 == 0) ? 0.15 : ((body_index % 3 == 1) ? -0.15 : 0.0);
    const double z = 0.2 + (static_cast<double>(z_index) * 0.72);
    const std::size_t inserted_index = world.bodies.add_body(make_sphere_body(
      static_cast<std::uint32_t>(body_index + 2),
      {x, y, z},
      kRadius,
      {(body_index % 2 == 0) ? 0.05 : -0.05, 0.0, 0.0}));
    (void)inserted_index;
  }

  return world;
}

auto benchmark_step_throughput(std::size_t body_count, std::size_t warmup_steps, std::size_t timed_steps) -> void {
  rex::sim::EngineConfig config{};
  config.simulation.gravity = {0.0, 0.0, -9.81};
  config.simulation.step.dt = 1.0 / 240.0;
  config.simulation.solver.friction_coefficient = 0.4;

  rex::sim::Engine engine{config};
  rex::dynamics::WorldState world = build_benchmark_world(body_count);

  for (std::size_t step_index = 0; step_index < warmup_steps; ++step_index) {
    (void)engine.step(world);
  }

  double integrate_ms = 0.0;
  double collision_ms = 0.0;
  double solver_ms = 0.0;
  double traced_total_ms = 0.0;
  const auto start = std::chrono::steady_clock::now();
  for (std::size_t step_index = 0; step_index < timed_steps; ++step_index) {
    const rex::sim::StepTrace trace = engine.step(world);
    integrate_ms += trace.profile.integrate_ms;
    collision_ms += trace.profile.collision_ms;
    solver_ms += trace.profile.solver_ms;
    traced_total_ms += trace.profile.total_ms;
  }
  const auto end = std::chrono::steady_clock::now();
  const std::chrono::duration<double> elapsed = end - start;
  const double total_seconds = elapsed.count();
  const double milliseconds_per_step = (total_seconds * 1000.0) / static_cast<double>(timed_steps);
  const double steps_per_second = static_cast<double>(timed_steps) / total_seconds;

  std::cout << "step throughput\n";
  std::cout << "  dynamic bodies: " << body_count << '\n';
  std::cout << "  timed steps: " << timed_steps << '\n';
  std::cout << "  total seconds: " << total_seconds << '\n';
  std::cout << "  ms/step: " << milliseconds_per_step << '\n';
  std::cout << "  steps/sec: " << steps_per_second << '\n';
  std::cout << "  avg integrate ms: " << (integrate_ms / static_cast<double>(timed_steps)) << '\n';
  std::cout << "  avg collision ms: " << (collision_ms / static_cast<double>(timed_steps)) << '\n';
  std::cout << "  avg solver ms: " << (solver_ms / static_cast<double>(timed_steps)) << '\n';
  std::cout << "  avg traced total ms: " << (traced_total_ms / static_cast<double>(timed_steps)) << '\n';
}

auto benchmark_viewer_projection(std::size_t iterations) -> void {
  const rex::viewer::ReplayLog replay = rex::viewer::build_demo_replay(120);
  rex::viewer::ViewerState state = rex::viewer::make_viewer_state(replay);
  const rex::viewer::FrameViewport viewport{.width = 1280.0, .height = 800.0, .margin = 60.0};

  const auto start = std::chrono::steady_clock::now();
  std::size_t polygon_count = 0;
  for (std::size_t iteration = 0; iteration < iterations; ++iteration) {
    const auto& frame = replay.frames()[iteration % replay.size()];
    rex::viewer::fit_camera_to_frame(state, frame, viewport);
    const rex::viewer::FrameProjectionCache cache =
      rex::viewer::build_frame_projection_cache(frame, state.camera, viewport);
    for (const auto& body : cache.bodies) {
      polygon_count += body.outline.size();
    }
  }
  const auto end = std::chrono::steady_clock::now();
  const std::chrono::duration<double> elapsed = end - start;
  const double iterations_per_second = static_cast<double>(iterations) / elapsed.count();

  std::cout << "viewer projection\n";
  std::cout << "  iterations: " << iterations << '\n';
  std::cout << "  polygons projected: " << polygon_count << '\n';
  std::cout << "  iterations/sec: " << iterations_per_second << '\n';
}

}  // namespace

int main(int argc, char** argv) {
  std::size_t body_count = 256;
  std::size_t warmup_steps = 60;
  std::size_t timed_steps = 600;
  std::size_t projection_iterations = 5000;

  for (int arg_index = 1; arg_index < argc; ++arg_index) {
    const std::string_view arg = argv[arg_index];
    if (arg == "--bodies" && arg_index + 1 < argc) {
      body_count = static_cast<std::size_t>(std::stoul(argv[++arg_index]));
    } else if (arg == "--warmup" && arg_index + 1 < argc) {
      warmup_steps = static_cast<std::size_t>(std::stoul(argv[++arg_index]));
    } else if (arg == "--steps" && arg_index + 1 < argc) {
      timed_steps = static_cast<std::size_t>(std::stoul(argv[++arg_index]));
    } else if (arg == "--projection-iters" && arg_index + 1 < argc) {
      projection_iterations = static_cast<std::size_t>(std::stoul(argv[++arg_index]));
    } else {
      std::cerr << "usage: rex_perf_app [--bodies N] [--warmup N] [--steps N] [--projection-iters N]\n";
      return 1;
    }
  }

  benchmark_step_throughput(body_count, warmup_steps, timed_steps);
  benchmark_viewer_projection(projection_iterations);
  return 0;
}
