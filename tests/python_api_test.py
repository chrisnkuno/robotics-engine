import importlib.util
import pathlib
import sys
import tempfile


def load_module(module_path: pathlib.Path):
    spec = importlib.util.spec_from_file_location("rex_py", module_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"failed to load rex_py from {module_path}")

    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main() -> int:
    if len(sys.argv) != 2:
        raise RuntimeError("usage: python_api_test.py <rex_py-module-path>")

    rex_py = load_module(pathlib.Path(sys.argv[1]))

    config = rex_py.EngineConfig()
    config.simulation.gravity = rex_py.Vec3(0.0, 0.0, 0.0)
    config.simulation.step.dt = 0.1
    engine = rex_py.Engine(config)

    world = rex_py.World()
    world.reserve_bodies(2)

    box_index = world.add_box(
        position=rex_py.Vec3(0.0, 0.0, 0.0),
        half_extents=rex_py.Vec3(0.5, 0.5, 0.5),
        inverse_mass=0.0,
    )
    sphere_index = world.add_sphere(
        position=rex_py.Vec3(0.9, 0.0, 0.0),
        radius=0.6,
        linear_velocity=rex_py.Vec3(0.1, 0.0, 0.0),
    )

    assert box_index == 0
    assert sphere_index == 1
    assert world.body_count == 2

    body = world.body(1)
    assert body["shape"] == "sphere"
    assert abs(body["radius"] - 0.6) < 1.0e-9
    assert body["id"].valid()

    trace = engine.step(world)
    assert trace.body_count == 2
    assert trace.manifold_count == 1
    assert trace.solver.contact_count == 1
    assert trace.solver.constraint_count == 3
    assert "broadphase" in trace.pipeline_summary
    assert world.contact_count == 1

    frame = rex_py.capture_frame(world, trace, 4, 0.4)
    assert frame.frame_index == 4
    assert abs(frame.sim_time - 0.4) < 1.0e-9
    assert len(frame.bodies) == 2
    assert len(frame.contacts) == 1
    assert frame.bodies[0].shape == rex_py.SnapshotShapeKind.BOX

    replay = rex_py.ReplayLog()
    replay.add_frame(frame)

    with tempfile.TemporaryDirectory() as temp_dir:
        replay_path = pathlib.Path(temp_dir) / "python-test.rex"
        replay.save(str(replay_path))
        loaded = rex_py.ReplayLog.load(str(replay_path))
        assert loaded.size() == 1
        assert len(loaded.frames()[0].contacts) == 1

    demo = rex_py.build_demo_replay(3)
    assert demo.size() == 3

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
