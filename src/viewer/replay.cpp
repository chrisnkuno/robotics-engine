#include "rex/viewer/replay.hpp"

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <variant>

#include "rex/geometry/shapes.hpp"

namespace rex::viewer {

namespace {

[[nodiscard]] auto shape_name(SnapshotShapeKind shape) -> const char* {
  switch (shape) {
    case SnapshotShapeKind::kSphere:
      return "sphere";
    case SnapshotShapeKind::kBox:
      return "box";
  }

  return "unknown";
}

[[nodiscard]] auto parse_shape_name(const std::string& value) -> SnapshotShapeKind {
  if (value == "sphere") {
    return SnapshotShapeKind::kSphere;
  }
  if (value == "box") {
    return SnapshotShapeKind::kBox;
  }

  throw std::runtime_error("unknown snapshot shape: " + value);
}

}  // namespace

void ReplayLog::add_frame(FrameSnapshot frame) {
  frames_.push_back(std::move(frame));
}

auto ReplayLog::frames() const noexcept -> const std::vector<FrameSnapshot>& {
  return frames_;
}

auto ReplayLog::empty() const noexcept -> bool {
  return frames_.empty();
}

auto ReplayLog::size() const noexcept -> std::size_t {
  return frames_.size();
}

void ReplayLog::save(const std::filesystem::path& path) const {
  std::ofstream output(path);
  if (!output) {
    throw std::runtime_error("failed to open replay file for writing: " + path.string());
  }

  output << "REX_REPLAY 1\n";
  for (const FrameSnapshot& frame : frames_) {
    output << "frame "
           << frame.frame_index << ' '
           << frame.sim_time << ' '
           << frame.trace.body_count << ' '
           << frame.trace.articulation_count << ' '
           << frame.trace.broadphase_pair_count << ' '
           << frame.trace.manifold_count << ' '
           << frame.trace.solver.contact_count << ' '
           << frame.trace.solver.constraint_count << ' '
           << frame.trace.solver.max_penetration << ' '
           << frame.bodies.size() << ' '
           << frame.contacts.size() << '\n';

    for (const SnapshotBody& body : frame.bodies) {
      output << "body "
             << body.id.index << ' '
             << body.id.generation << ' '
             << shape_name(body.shape) << ' '
             << body.translation.x << ' '
             << body.translation.y << ' '
             << body.translation.z << ' '
             << body.dimensions.x << ' '
             << body.dimensions.y << ' '
             << body.dimensions.z << '\n';
    }

    for (const SnapshotContact& contact : frame.contacts) {
      output << "contact "
             << contact.body_a.index << ' '
             << contact.body_a.generation << ' '
             << contact.body_b.index << ' '
             << contact.body_b.generation << ' '
             << contact.position.x << ' '
             << contact.position.y << ' '
             << contact.position.z << ' '
             << contact.normal.x << ' '
             << contact.normal.y << ' '
             << contact.normal.z << ' '
             << contact.penetration << '\n';
    }

    output << "endframe\n";
  }
}

auto ReplayLog::load(const std::filesystem::path& path) -> ReplayLog {
  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("failed to open replay file for reading: " + path.string());
  }

  std::string header{};
  int version = 0;
  input >> header >> version;
  if (header != "REX_REPLAY" || version != 1) {
    throw std::runtime_error("unsupported replay file: " + path.string());
  }

  ReplayLog replay{};
  std::string token{};
  while (input >> token) {
    if (token != "frame") {
      throw std::runtime_error("expected frame record in replay: " + path.string());
    }

    FrameSnapshot frame{};
    std::size_t body_record_count = 0;
    std::size_t contact_record_count = 0;

    input >> frame.frame_index
          >> frame.sim_time
          >> frame.trace.body_count
          >> frame.trace.articulation_count
          >> frame.trace.broadphase_pair_count
          >> frame.trace.manifold_count
          >> frame.trace.solver.contact_count
          >> frame.trace.solver.constraint_count
          >> frame.trace.solver.max_penetration
          >> body_record_count
          >> contact_record_count;

    frame.bodies.reserve(body_record_count);
    frame.contacts.reserve(contact_record_count);

    for (std::size_t body_index = 0; body_index < body_record_count; ++body_index) {
      std::string body_token{};
      std::string shape_token{};
      SnapshotBody body{};
      input >> body_token
            >> body.id.index
            >> body.id.generation
            >> shape_token
            >> body.translation.x
            >> body.translation.y
            >> body.translation.z
            >> body.dimensions.x
            >> body.dimensions.y
            >> body.dimensions.z;

      if (body_token != "body") {
        throw std::runtime_error("expected body record in replay: " + path.string());
      }

      body.shape = parse_shape_name(shape_token);
      frame.bodies.push_back(body);
    }

    for (std::size_t contact_index = 0; contact_index < contact_record_count; ++contact_index) {
      std::string contact_token{};
      SnapshotContact contact{};
      input >> contact_token
            >> contact.body_a.index
            >> contact.body_a.generation
            >> contact.body_b.index
            >> contact.body_b.generation
            >> contact.position.x
            >> contact.position.y
            >> contact.position.z
            >> contact.normal.x
            >> contact.normal.y
            >> contact.normal.z
            >> contact.penetration;

      if (contact_token != "contact") {
        throw std::runtime_error("expected contact record in replay: " + path.string());
      }

      frame.contacts.push_back(contact);
    }

    std::string endframe_token{};
    input >> endframe_token;
    if (endframe_token != "endframe") {
      throw std::runtime_error("expected endframe marker in replay: " + path.string());
    }

    replay.add_frame(std::move(frame));
  }

  return replay;
}

auto capture_frame(
  const rex::dynamics::WorldState& world,
  const rex::sim::StepTrace& trace,
  std::uint64_t frame_index,
  double sim_time) -> FrameSnapshot {
  FrameSnapshot frame{};
  frame.frame_index = frame_index;
  frame.sim_time = sim_time;
  frame.trace = trace;
  frame.bodies.reserve(world.bodies.size());

  for (std::size_t body_index = 0; body_index < world.bodies.size(); ++body_index) {
    const rex::dynamics::BodyState state = world.bodies.state(body_index);

    SnapshotBody body{};
    body.id = state.id;
    body.translation = state.pose.translation;
    body.shape = std::holds_alternative<rex::geometry::Sphere>(state.shape.data)
      ? SnapshotShapeKind::kSphere
      : SnapshotShapeKind::kBox;

    if (const auto* sphere = std::get_if<rex::geometry::Sphere>(&state.shape.data)) {
      body.dimensions = {sphere->radius, 0.0, 0.0};
    } else if (const auto* box = std::get_if<rex::geometry::Box>(&state.shape.data)) {
      body.dimensions = box->half_extents;
    }

    frame.bodies.push_back(body);
  }

  for (const auto& manifold : world.contact_manifolds) {
    for (std::size_t point_index = 0; point_index < manifold.point_count; ++point_index) {
      frame.contacts.push_back({
        .body_a = manifold.body_a,
        .body_b = manifold.body_b,
        .position = manifold.points[point_index].position,
        .normal = manifold.points[point_index].normal,
        .penetration = manifold.points[point_index].penetration,
      });
    }
  }

  return frame;
}

}  // namespace rex::viewer

