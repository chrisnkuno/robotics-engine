# Robotics Engine

Greenfield scaffold for a layered robotics simulation engine with a C++20 core, optional CUDA and Python frontends, and a module layout aimed at deterministic batched dynamics.

The current repo deliberately starts small:

- a buildable CMake workspace
- module boundaries that match the engine architecture
- a concrete first-pass solver/contact design in [docs/architecture/module-graph.md](/home/chris/robotics-engine/docs/architecture/module-graph.md)
- a smoke test that exercises the public stepping surface
- a native replay viewer scaffold that exports SVG debug frames
- invariant-heavy validation across collision, integration, transforms, viewer math, Python bindings, and dataset export

## Build

```bash
cmake -S . -B build
cmake --build build
ctest --test-dir build
```

## Benchmark

You can measure the current headless step throughput and cached viewer-projection workload with:

```bash
./build/rex_perf_app --bodies 256 --warmup 30 --steps 300 --projection-iters 2000
```

The benchmark now reports average per-step `integrate`, `collision`, `solver`, and traced `total` times so you can see where changes are moving cost, not just the final wall-clock throughput.

The current test matrix mixes:

- deterministic regression fixtures in C++
- finite-difference checks for discrete dynamics and kinematic Jacobians
- randomized property tests in Python via `Hypothesis` and `NumPy`
- replay-to-Parquet export checks via `pyarrow`

## Visualize A Demo

After building, you can generate a demo replay and SVG frames with:

```bash
./build/rex_viewer_app --demo build/viewer-demo
```

You can also open a real desktop replay window immediately:

```bash
./build/rex_viewer_app --demo-window
```

Or stream frames from a running simulation directly into the window:

```bash
./build/rex_viewer_app --live-demo-window
```

That writes:

- `build/viewer-demo/demo.rex`: replay capture
- `build/viewer-demo/frame-*.svg`: debug-rendered frames you can open directly

You can also export SVGs from an existing replay:

```bash
./build/rex_viewer_app path/to/run.rex build/viewer-export
```

Or open an existing replay in the native viewer:

```bash
./build/rex_viewer_app --window path/to/run.rex
```

Viewer controls:

- `Space`: play/pause
- `Left` / `Right`: step backward/forward
- `Left mouse drag`: orbit camera
- `Right mouse drag` or `Middle mouse drag`: pan camera
- `Mouse wheel` or `+` / `-`: zoom / dolly
- `W A S D`: pan camera
- `C`: toggle contact markers
- `N`: toggle contact normals
- `G`: toggle grid
- `T`: toggle motion trails
- `R`: reset camera fit
- `Left mouse click`: inspect a body or contact
- bottom scrub bar: drag to any captured frame
- `Esc`: quit

The desktop viewer now renders the replay as a simple real-time 3D scene with perspective, flat-shaded boxes/spheres, a ground grid, trails, and depth-sorted geometry.

Optional mesh import:

```bash
REX_VIEWER_MESH_DIR=path/to/meshes ./build/rex_viewer_app --window run.rex
```

If `REX_VIEWER_MESH_DIR` is set, the viewer will look for per-body OBJ files named `<body-id>.obj` and use them in place of the built-in primitive mesh.

## Optional Features

- `-DREX_ENABLE_CUDA=ON` enables a placeholder CUDA backend target when a CUDA compiler is available.
- `-DREX_ENABLE_PYTHON=ON` builds `rex_py` when `pybind11` is installed.
- `-DREX_ENABLE_TRACY=ON` enables Tracy-compatible profiling zones when Tracy headers are available on the include path.

## Python Research API

The first research-facing surface is a Python module built with `pybind11`. Configure it by pointing CMake at the installed `pybind11` package:

```bash
cmake -S . -B build \
  -DREX_ENABLE_PYTHON=ON \
  -Dpybind11_DIR="$(python -m pybind11 --cmakedir)"
cmake --build build
ctest --test-dir build --output-on-failure
```

That module currently supports:

- `EngineConfig` / `SimulationConfig` / solver and collision config structs
- `Quat` utilities plus `quat_from_axis_angle(...)` / `integrate_rotation(...)`
- a `World` wrapper for adding spheres and boxes from Python, including initial rotation and angular velocity
- direct body-state mutation through `set_pose(...)`, `set_translation(...)`, `set_rotation(...)`, `set_linear_velocity(...)`, and `set_angular_velocity(...)`
- `Engine.step(world)` for headless stepping
- `capture_frame(...)` and `ReplayLog` for saving `.rex` runs that the desktop viewer can open
- `python/rex_data.py` for exporting replay logs to Arrow/Parquet tables
- `python/rex_research.py` for paper metadata, rollout harnesses, action application, and latent-world-model planners

Minimal example:

```python
import rex_py

config = rex_py.EngineConfig()
config.simulation.gravity = rex_py.Vec3(0.0, 0.0, 0.0)

engine = rex_py.Engine(config)
world = rex_py.World()
world.add_box(rex_py.Vec3(0.0, 0.0, 0.0), rex_py.Vec3(0.5, 0.5, 0.5), inverse_mass=0.0)
world.add_sphere(rex_py.Vec3(0.9, 0.0, 0.0), radius=0.6)

trace = engine.step(world)
frame = rex_py.capture_frame(world, trace, frame_index=0, sim_time=0.0)

replay = rex_py.ReplayLog()
replay.add_frame(frame)
replay.save("run.rex")
```

Then open the result in the desktop viewer:

```bash
./build/rex_viewer_app --window run.rex
```

You can also export a replay to Parquet for notebook analysis:

```python
import importlib.util
from pathlib import Path

spec = importlib.util.spec_from_file_location("rex_data", Path("python/rex_data.py"))
rex_data = importlib.util.module_from_spec(spec)
spec.loader.exec_module(rex_data)

rex_data.write_replay_dataset(replay, "run-dataset")
```

That writes:

- `run-dataset/frames.parquet`
- `run-dataset/bodies.parquet`
- `run-dataset/contacts.parquet`

## Research Interface

The new high-level research surface lives in [python/rex_research.py](/home/chris/robotics-engine/python/rex_research.py).
It is designed so researchers can keep their paper code in Python and only treat the engine as a fast simulator backend.

It currently includes:

- `PaperSpec` for paper/model metadata
- `BodyCommand` for direct world edits per step
- `ResearchHarness` for stepping, replay capture, and rollout collection
- `CallableWorldModel` for wrapping arbitrary research code
- `RandomShootingPlanner` for latent-space planning
- `load_vjepa2_torch_hub(...)`, `load_lewm_stable_worldmodel(...)`, and `latest_jepa_world_model_specs()` for the current JEPA / LeWorldModel path

The JEPA-specific integration notes are in [world-model-interface.md](/home/chris/robotics-engine/docs/research/world-model-interface.md).

## Repo Layout

- `include/rex`: public module interfaces
- `src`: minimal implementations and backend stubs
- `docs/architecture`: module graph and solver/contact decisions
- `docs/research`: research-facing integration notes
- `apps`: native tools such as the replay viewer scaffold
- `python`: notebook/data/research helpers on top of `rex_py`
- `tests`: smoke coverage for the initial stepping API
