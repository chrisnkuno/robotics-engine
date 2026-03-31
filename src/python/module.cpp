#include <pybind11/pybind11.h>

#include "rex/sim/engine.hpp"

namespace py = pybind11;

PYBIND11_MODULE(rex_py, module) {
  py::class_<rex::solver::SolverResult>(module, "SolverResult")
    .def_readonly("contact_count", &rex::solver::SolverResult::contact_count)
    .def_readonly("constraint_count", &rex::solver::SolverResult::constraint_count)
    .def_readonly("max_penetration", &rex::solver::SolverResult::max_penetration);

  py::class_<rex::sim::StepTrace>(module, "StepTrace")
    .def_readonly("body_count", &rex::sim::StepTrace::body_count)
    .def_readonly("articulation_count", &rex::sim::StepTrace::articulation_count)
    .def_readonly("solver", &rex::sim::StepTrace::solver)
    .def_readonly("pipeline_summary", &rex::sim::StepTrace::pipeline_summary);

  py::class_<rex::sim::Engine>(module, "Engine")
    .def(py::init<>())
    .def("describe", [](const rex::sim::Engine& engine) {
      rex::dynamics::WorldState world{};
      return engine.step(world).pipeline_summary;
    });
}

