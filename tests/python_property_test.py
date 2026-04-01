import importlib.util
import math
import pathlib
import sys

import numpy as np
from hypothesis import given, settings
from hypothesis import strategies as st


def load_module(module_path: pathlib.Path):
    spec = importlib.util.spec_from_file_location("rex_py", module_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"failed to load rex_py from {module_path}")

    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def make_engine(module, *, dt: float, gravity_z: float):
    config = module.EngineConfig()
    config.simulation.gravity = module.Vec3(0.0, 0.0, gravity_z)
    config.simulation.step.dt = dt
    return module.Engine(config)


def make_world(module, *, position_z: float, velocity_z: float):
    world = module.World()
    world.add_sphere(
        position=module.Vec3(0.0, 0.0, position_z),
        radius=0.25,
        linear_velocity=module.Vec3(0.0, 0.0, velocity_z),
    )
    return world


def body_translation_z(world):
    return world.body(0)["translation"].z


def body_velocity_z(world):
    return world.body(0)["linear_velocity"].z


def quat_alignment(lhs, rhs):
    return abs(lhs.w * rhs.w + lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z)


def quat_norm(value):
    return math.sqrt(value.w * value.w + value.x * value.x + value.y * value.y + value.z * value.z)


@settings(max_examples=30, deadline=None)
@given(
    dt=st.floats(min_value=0.01, max_value=0.2, allow_nan=False, allow_infinity=False),
    gravity_z=st.floats(min_value=-20.0, max_value=-0.1, allow_nan=False, allow_infinity=False),
    position_z=st.floats(min_value=-2.0, max_value=3.0, allow_nan=False, allow_infinity=False),
    velocity_z=st.floats(min_value=-5.0, max_value=5.0, allow_nan=False, allow_infinity=False),
)
def test_free_fall_matches_closed_form(module, dt, gravity_z, position_z, velocity_z):
    engine = make_engine(module, dt=dt, gravity_z=gravity_z)
    world = make_world(module, position_z=position_z, velocity_z=velocity_z)
    engine.step(world)

    expected_velocity = velocity_z + gravity_z * dt
    expected_position = position_z + expected_velocity * dt
    actual = np.array([body_translation_z(world), body_velocity_z(world)])
    expected = np.array([expected_position, expected_velocity])
    assert np.allclose(actual, expected, atol=1.0e-9)


@settings(max_examples=20, deadline=None)
@given(
    incoming_speed=st.floats(min_value=0.05, max_value=3.0, allow_nan=False, allow_infinity=False),
)
def test_elastic_contact_conserves_momentum_and_energy(module, incoming_speed):
    config = module.EngineConfig()
    config.simulation.gravity = module.Vec3(0.0, 0.0, 0.0)
    config.simulation.step.dt = 0.0
    config.simulation.solver.restitution = 1.0
    config.simulation.solver.position_correction_factor = 0.0
    config.simulation.solver.position_iterations = 0
    engine = module.Engine(config)

    world = module.World()
    world.add_sphere(
        position=module.Vec3(0.0, 0.0, 0.0),
        radius=0.5,
        linear_velocity=module.Vec3(incoming_speed, 0.0, 0.0),
    )
    world.add_sphere(
        position=module.Vec3(0.9, 0.0, 0.0),
        radius=0.5,
        linear_velocity=module.Vec3(0.0, 0.0, 0.0),
    )

    before = np.array([
        world.body(0)["linear_velocity"].x,
        world.body(1)["linear_velocity"].x,
    ])
    engine.step(world)
    after = np.array([
        world.body(0)["linear_velocity"].x,
        world.body(1)["linear_velocity"].x,
    ])

    assert np.allclose(before.sum(), after.sum(), atol=1.0e-9)
    assert np.allclose(np.dot(before, before), np.dot(after, after), atol=1.0e-9)


@settings(max_examples=30, deadline=None)
@given(
    dt=st.floats(min_value=0.01, max_value=0.2, allow_nan=False, allow_infinity=False),
    initial_angle=st.floats(min_value=-1.0, max_value=1.0, allow_nan=False, allow_infinity=False),
    angular_speed=st.floats(min_value=-4.0, max_value=4.0, allow_nan=False, allow_infinity=False),
)
def test_rotation_integration_preserves_unit_quaternion(module, dt, initial_angle, angular_speed):
    initial = module.quat_from_axis_angle(module.Vec3(0.0, 1.0, 0.0), initial_angle)
    actual = module.integrate_rotation(initial, module.Vec3(0.0, angular_speed, 0.0), dt)
    expected = module.quat_from_axis_angle(module.Vec3(0.0, 1.0, 0.0), initial_angle + angular_speed * dt)
    assert math.isclose(quat_norm(actual), 1.0, abs_tol=1.0e-9)
    assert math.isclose(quat_alignment(actual, expected), 1.0, abs_tol=1.0e-9)


@settings(max_examples=20, deadline=None)
@given(
    incoming_normal_speed=st.floats(min_value=0.1, max_value=3.0, allow_nan=False, allow_infinity=False),
    incoming_tangent_speed=st.floats(min_value=-2.0, max_value=2.0, allow_nan=False, allow_infinity=False),
    friction=st.floats(min_value=0.1, max_value=1.0, allow_nan=False, allow_infinity=False),
)
def test_friction_does_not_increase_tangential_speed(module, incoming_normal_speed, incoming_tangent_speed, friction):
    config = module.EngineConfig()
    config.simulation.gravity = module.Vec3(0.0, 0.0, 0.0)
    config.simulation.step.dt = 0.0
    config.simulation.solver.restitution = 0.0
    config.simulation.solver.friction_coefficient = friction
    config.simulation.solver.position_correction_factor = 0.0
    config.simulation.solver.position_iterations = 0
    engine = module.Engine(config)

    world = module.World()
    world.add_box(
        position=module.Vec3(0.0, 0.0, 0.0),
        half_extents=module.Vec3(0.5, 0.5, 0.5),
        inverse_mass=0.0,
    )
    world.add_sphere(
        position=module.Vec3(0.8, 0.0, 0.0),
        radius=0.4,
        linear_velocity=module.Vec3(-incoming_normal_speed, 0.0, incoming_tangent_speed),
    )

    engine.step(world)
    sphere = world.body(1)
    assert abs(sphere["linear_velocity"].z) <= abs(incoming_tangent_speed) + 1.0e-9
    assert sphere["linear_velocity"].x >= -1.0e-9


def main() -> int:
    if len(sys.argv) != 2:
        raise RuntimeError("usage: python_property_test.py <rex_py-module-path>")

    module = load_module(pathlib.Path(sys.argv[1]))
    test_free_fall_matches_closed_form(module)
    test_elastic_contact_conserves_momentum_and_energy(module)
    test_rotation_integration_preserves_unit_quaternion(module)
    test_friction_does_not_increase_tangential_speed(module)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
