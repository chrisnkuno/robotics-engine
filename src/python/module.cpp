#include <cstdint>
#include <stdexcept>
#include <string>
#include <variant>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "rex/collision/contact.hpp"
#include "rex/dynamics/world.hpp"
#include "rex/geometry/shapes.hpp"
#include "rex/sim/engine.hpp"
#include "rex/viewer/demo.hpp"
#include "rex/viewer/replay.hpp"

namespace py = pybind11;

namespace {

class PyWorld {
 public:
  void reserve_bodies(std::size_t capacity) {
    state_.bodies.reserve(capacity);
  }

  [[nodiscard]] auto body_count() const noexcept -> std::size_t {
    return state_.bodies.size();
  }

  [[nodiscard]] auto contact_count() const noexcept -> std::size_t {
    std::size_t count = 0;
    for (const auto& manifold : state_.contact_manifolds) {
      count += manifold.point_count;
    }

    return count;
  }

  [[nodiscard]] auto add_sphere(
    const rex::math::Vec3& position,
    double radius,
    double inverse_mass,
    const rex::math::Vec3& linear_velocity,
    const rex::math::Quat& rotation,
    const rex::math::Vec3& angular_velocity) -> std::size_t {
    return add_body({
      .id = next_entity_id(),
      .pose = rex::math::Transform{.rotation = rotation, .translation = position},
      .linear_velocity = linear_velocity,
      .angular_velocity = angular_velocity,
      .inverse_mass = inverse_mass,
      .shape = rex::geometry::Shape{.data = rex::geometry::Sphere{.radius = radius}},
    });
  }

  [[nodiscard]] auto add_box(
    const rex::math::Vec3& position,
    const rex::math::Vec3& half_extents,
    double inverse_mass,
    const rex::math::Vec3& linear_velocity,
    const rex::math::Quat& rotation,
    const rex::math::Vec3& angular_velocity) -> std::size_t {
    return add_body({
      .id = next_entity_id(),
      .pose = rex::math::Transform{.rotation = rotation, .translation = position},
      .linear_velocity = linear_velocity,
      .angular_velocity = angular_velocity,
      .inverse_mass = inverse_mass,
      .shape = rex::geometry::Shape{.data = rex::geometry::Box{.half_extents = half_extents}},
    });
  }

  [[nodiscard]] auto body(std::size_t index) const -> py::dict {
    const rex::dynamics::BodyState state = state_.bodies.state(index);

    py::dict record{};
    record["id"] = state.id;
    record["rotation"] = state.pose.rotation;
    record["translation"] = state.pose.translation;
    record["linear_velocity"] = state.linear_velocity;
    record["angular_velocity"] = state.angular_velocity;
    record["inverse_mass"] = state.inverse_mass;

    if (const auto* sphere = std::get_if<rex::geometry::Sphere>(&state.shape.data)) {
      record["shape"] = "sphere";
      record["radius"] = sphere->radius;
    } else if (const auto* box = std::get_if<rex::geometry::Box>(&state.shape.data)) {
      record["shape"] = "box";
      record["half_extents"] = box->half_extents;
    } else {
      throw std::runtime_error("unsupported shape in Python body view");
    }

    return record;
  }

  [[nodiscard]] auto state() noexcept -> rex::dynamics::WorldState& {
    return state_;
  }

  [[nodiscard]] auto state() const noexcept -> const rex::dynamics::WorldState& {
    return state_;
  }

 private:
  [[nodiscard]] auto add_body(const rex::dynamics::BodyState& body) -> std::size_t {
    return state_.bodies.add_body(body);
  }

  [[nodiscard]] auto next_entity_id() noexcept -> rex::platform::EntityId {
    return {
      .index = next_entity_index_++,
      .generation = 1,
    };
  }

  rex::dynamics::WorldState state_{};
  std::uint32_t next_entity_index_{1};
};

}  // namespace

PYBIND11_MODULE(rex_py, module) {
  py::class_<rex::platform::EntityId>(module, "EntityId")
    .def(py::init<>())
    .def_readwrite("index", &rex::platform::EntityId::index)
    .def_readwrite("generation", &rex::platform::EntityId::generation)
    .def("valid", &rex::platform::EntityId::valid);

  py::class_<rex::math::Vec3>(module, "Vec3")
    .def(py::init<double, double, double>(), py::arg("x") = 0.0, py::arg("y") = 0.0, py::arg("z") = 0.0)
    .def_readwrite("x", &rex::math::Vec3::x)
    .def_readwrite("y", &rex::math::Vec3::y)
    .def_readwrite("z", &rex::math::Vec3::z);

  py::class_<rex::math::Quat>(module, "Quat")
    .def(py::init<double, double, double, double>(), py::arg("w") = 1.0, py::arg("x") = 0.0, py::arg("y") = 0.0, py::arg("z") = 0.0)
    .def_readwrite("w", &rex::math::Quat::w)
    .def_readwrite("x", &rex::math::Quat::x)
    .def_readwrite("y", &rex::math::Quat::y)
    .def_readwrite("z", &rex::math::Quat::z);
  module.def("quat_from_axis_angle", &rex::math::quat_from_axis_angle, py::arg("axis"), py::arg("radians"));
  module.def(
    "integrate_rotation",
    &rex::math::integrate_rotation,
    py::arg("rotation"),
    py::arg("angular_velocity"),
    py::arg("dt"));

  py::enum_<rex::dynamics::Integrator>(module, "Integrator")
    .value("SEMI_IMPLICIT_EULER", rex::dynamics::Integrator::kSemiImplicitEuler);

  py::class_<rex::dynamics::StepConfig>(module, "StepConfig")
    .def(py::init<>())
    .def_readwrite("dt", &rex::dynamics::StepConfig::dt)
    .def_readwrite("integrator", &rex::dynamics::StepConfig::integrator);

  py::enum_<rex::solver::SolverType>(module, "SolverType")
    .value("SEQUENTIAL_IMPULSE_PGS", rex::solver::SolverType::kSequentialImpulsePgs);

  py::enum_<rex::solver::FrictionModel>(module, "FrictionModel")
    .value("PYRAMID", rex::solver::FrictionModel::kPyramid);

  py::enum_<rex::solver::StabilizationMode>(module, "StabilizationMode")
    .value("SPLIT_IMPULSE", rex::solver::StabilizationMode::kSplitImpulse);

  py::class_<rex::solver::SolverConfig>(module, "SolverConfig")
    .def(py::init<>())
    .def_readwrite("type", &rex::solver::SolverConfig::type)
    .def_readwrite("friction_model", &rex::solver::SolverConfig::friction_model)
    .def_readwrite("stabilization", &rex::solver::SolverConfig::stabilization)
    .def_readwrite("velocity_iterations", &rex::solver::SolverConfig::velocity_iterations)
    .def_readwrite("position_iterations", &rex::solver::SolverConfig::position_iterations)
    .def_readwrite("warm_start", &rex::solver::SolverConfig::warm_start)
    .def_readwrite("deterministic_ordering", &rex::solver::SolverConfig::deterministic_ordering)
    .def_readwrite("restitution", &rex::solver::SolverConfig::restitution)
    .def_readwrite("friction_coefficient", &rex::solver::SolverConfig::friction_coefficient)
    .def_readwrite("penetration_slop", &rex::solver::SolverConfig::penetration_slop)
    .def_readwrite("position_correction_factor", &rex::solver::SolverConfig::position_correction_factor);

  py::class_<rex::collision::CollisionPipelineConfig>(module, "CollisionPipelineConfig")
    .def(py::init<>())
    .def_readwrite("max_points_per_manifold", &rex::collision::CollisionPipelineConfig::max_points_per_manifold)
    .def_readwrite("enable_persistent_manifolds", &rex::collision::CollisionPipelineConfig::enable_persistent_manifolds);

  py::class_<rex::dynamics::SimulationConfig>(module, "SimulationConfig")
    .def(py::init<>())
    .def_readwrite("gravity", &rex::dynamics::SimulationConfig::gravity)
    .def_readwrite("step", &rex::dynamics::SimulationConfig::step)
    .def_readwrite("collision", &rex::dynamics::SimulationConfig::collision)
    .def_readwrite("solver", &rex::dynamics::SimulationConfig::solver);

  py::class_<rex::sim::EngineConfig>(module, "EngineConfig")
    .def(py::init<>())
    .def_readwrite("simulation", &rex::sim::EngineConfig::simulation);

  py::class_<rex::solver::SolverResult>(module, "SolverResult")
    .def_readonly("contact_count", &rex::solver::SolverResult::contact_count)
    .def_readonly("constraint_count", &rex::solver::SolverResult::constraint_count)
    .def_readonly("max_penetration", &rex::solver::SolverResult::max_penetration);

  py::class_<rex::sim::StepTrace>(module, "StepTrace")
    .def_readonly("body_count", &rex::sim::StepTrace::body_count)
    .def_readonly("articulation_count", &rex::sim::StepTrace::articulation_count)
    .def_readonly("broadphase_pair_count", &rex::sim::StepTrace::broadphase_pair_count)
    .def_readonly("manifold_count", &rex::sim::StepTrace::manifold_count)
    .def_readonly("solver", &rex::sim::StepTrace::solver)
    .def_readonly("pipeline_summary", &rex::sim::StepTrace::pipeline_summary);

  py::enum_<rex::viewer::SnapshotShapeKind>(module, "SnapshotShapeKind")
    .value("SPHERE", rex::viewer::SnapshotShapeKind::kSphere)
    .value("BOX", rex::viewer::SnapshotShapeKind::kBox);

  py::class_<rex::viewer::SnapshotBody>(module, "SnapshotBody")
    .def(py::init<>())
    .def_readonly("id", &rex::viewer::SnapshotBody::id)
    .def_readonly("shape", &rex::viewer::SnapshotBody::shape)
    .def_readonly("rotation", &rex::viewer::SnapshotBody::rotation)
    .def_readonly("translation", &rex::viewer::SnapshotBody::translation)
    .def_readonly("dimensions", &rex::viewer::SnapshotBody::dimensions);

  py::class_<rex::viewer::SnapshotContact>(module, "SnapshotContact")
    .def(py::init<>())
    .def_readonly("body_a", &rex::viewer::SnapshotContact::body_a)
    .def_readonly("body_b", &rex::viewer::SnapshotContact::body_b)
    .def_readonly("position", &rex::viewer::SnapshotContact::position)
    .def_readonly("normal", &rex::viewer::SnapshotContact::normal)
    .def_readonly("penetration", &rex::viewer::SnapshotContact::penetration);

  py::class_<rex::viewer::FrameSnapshot>(module, "FrameSnapshot")
    .def(py::init<>())
    .def_readonly("frame_index", &rex::viewer::FrameSnapshot::frame_index)
    .def_readonly("sim_time", &rex::viewer::FrameSnapshot::sim_time)
    .def_readonly("trace", &rex::viewer::FrameSnapshot::trace)
    .def_readonly("bodies", &rex::viewer::FrameSnapshot::bodies)
    .def_readonly("contacts", &rex::viewer::FrameSnapshot::contacts);

  py::class_<rex::viewer::ReplayLog>(module, "ReplayLog")
    .def(py::init<>())
    .def("add_frame", &rex::viewer::ReplayLog::add_frame)
    .def("frames", [](const rex::viewer::ReplayLog& replay) { return replay.frames(); })
    .def("save", [](const rex::viewer::ReplayLog& replay, const std::string& path) { replay.save(path); })
    .def("size", &rex::viewer::ReplayLog::size)
    .def_static("load", [](const std::string& path) { return rex::viewer::ReplayLog::load(path); });

  py::class_<PyWorld>(module, "World")
    .def(py::init<>())
    .def("reserve_bodies", &PyWorld::reserve_bodies)
    .def(
      "add_sphere",
      &PyWorld::add_sphere,
      py::arg("position"),
      py::arg("radius"),
      py::arg("inverse_mass") = 1.0,
      py::arg("linear_velocity") = rex::math::Vec3{},
      py::arg("rotation") = rex::math::Quat{},
      py::arg("angular_velocity") = rex::math::Vec3{})
    .def(
      "add_box",
      &PyWorld::add_box,
      py::arg("position"),
      py::arg("half_extents"),
      py::arg("inverse_mass") = 1.0,
      py::arg("linear_velocity") = rex::math::Vec3{},
      py::arg("rotation") = rex::math::Quat{},
      py::arg("angular_velocity") = rex::math::Vec3{})
    .def("body", &PyWorld::body)
    .def_property_readonly("body_count", &PyWorld::body_count)
    .def_property_readonly("contact_count", &PyWorld::contact_count);

  py::class_<rex::sim::Engine>(module, "Engine")
    .def(py::init<rex::sim::EngineConfig>(), py::arg("config") = rex::sim::EngineConfig{})
    .def("config", &rex::sim::Engine::config, py::return_value_policy::reference_internal)
    .def("step", [](const rex::sim::Engine& engine, PyWorld& world) { return engine.step(world.state()); })
    .def("describe", [](const rex::sim::Engine& engine) {
      rex::dynamics::WorldState world{};
      return engine.step(world).pipeline_summary;
    });

  module.def(
    "capture_frame",
    [](const PyWorld& world, const rex::sim::StepTrace& trace, std::uint64_t frame_index, double sim_time) {
      return rex::viewer::capture_frame(world.state(), trace, frame_index, sim_time);
    },
    py::arg("world"),
    py::arg("trace"),
    py::arg("frame_index"),
    py::arg("sim_time"));
  module.def("build_demo_replay", &rex::viewer::build_demo_replay, py::arg("frame_count") = 8);
}
