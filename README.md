# Robotics Engine

Greenfield scaffold for a layered robotics simulation engine with a C++20 core, optional CUDA and Python frontends, and a module layout aimed at deterministic batched dynamics.

The current repo deliberately starts small:

- a buildable CMake workspace
- module boundaries that match the engine architecture
- a concrete first-pass solver/contact design in [docs/architecture/module-graph.md](/home/chris/robotics-engine/docs/architecture/module-graph.md)
- a smoke test that exercises the public stepping surface
- a native replay viewer scaffold that exports SVG debug frames

## Build

```bash
cmake -S . -B build
cmake --build build
ctest --test-dir build
```

## Visualize A Demo

After building, you can generate a demo replay and SVG frames with:

```bash
./build/rex_viewer_app --demo build/viewer-demo
```

That writes:

- `build/viewer-demo/demo.rex`: replay capture
- `build/viewer-demo/frame-*.svg`: debug-rendered frames you can open directly

You can also export SVGs from an existing replay:

```bash
./build/rex_viewer_app path/to/run.rex build/viewer-export
```

## Optional Features

- `-DREX_ENABLE_CUDA=ON` enables a placeholder CUDA backend target when a CUDA compiler is available.
- `-DREX_ENABLE_PYTHON=ON` builds `rex_py` when `pybind11` is installed.

## Repo Layout

- `include/rex`: public module interfaces
- `src`: minimal implementations and backend stubs
- `docs/architecture`: module graph and solver/contact decisions
- `apps`: native tools such as the replay viewer scaffold
- `tests`: smoke coverage for the initial stepping API
