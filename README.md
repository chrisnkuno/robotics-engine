# Robotics Engine

Greenfield scaffold for a layered robotics simulation engine with a C++20 core, optional CUDA and Python frontends, and a module layout aimed at deterministic batched dynamics.

The current repo deliberately starts small:

- a buildable CMake workspace
- module boundaries that match the engine architecture
- a concrete first-pass solver/contact design in [docs/architecture/module-graph.md](/home/chris/robotics-engine/docs/architecture/module-graph.md)
- a smoke test that exercises the public stepping surface

## Build

```bash
cmake -S . -B build
cmake --build build
ctest --test-dir build
```

## Optional Features

- `-DREX_ENABLE_CUDA=ON` enables a placeholder CUDA backend target when a CUDA compiler is available.
- `-DREX_ENABLE_PYTHON=ON` builds `rex_py` when `pybind11` is installed.

## Repo Layout

- `include/rex`: public module interfaces
- `src`: minimal implementations and backend stubs
- `docs/architecture`: module graph and solver/contact decisions
- `tests`: smoke coverage for the initial stepping API

