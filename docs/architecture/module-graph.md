# Module Graph

This repo is structured around one constraint: the research-facing surface must stay replaceable without destabilizing the numerical core.

## Design Goals

- deterministic CPU stepping as the reference path
- batch-friendly data layout from the first implementation
- isolated interfaces between articulation, collision, and solving
- an explicit migration path to CUDA kernels without redesigning the scene model

## Layered Graph

```text
platform
  ├─ handles, deterministic ids, serialization seams
  └─ memory/tasking hooks

math
  ├─ vectors, quaternions, transforms, spatial algebra
  └─ fixed-size types that can survive CPU/GPU migration

geometry
  ├─ primitive shapes
  └─ mesh and acceleration-structure inputs later

kinematics
  ├─ articulated trees
  ├─ joint models
  └─ jacobian / FK / IK entry points

collision
  ├─ broadphase pairs
  ├─ narrowphase contacts
  └─ persistent manifolds

solver
  ├─ joint constraints
  ├─ contact/friction constraints
  └─ warm-started iterative solve

dynamics
  ├─ body state
  ├─ articulated-body forward dynamics
  └─ integration policy

sim
  ├─ frame pipeline
  ├─ stepping API
  └─ trace/debug outputs

python / gpu / renderer
  └─ replaceable frontends and accelerators
```

## Dependency Rules

- `platform` has no simulation dependencies.
- `math` depends only on `platform`.
- `geometry` depends on `math`.
- `kinematics` depends on `platform` and `math`.
- `collision` depends on `geometry`, `math`, and `platform`, but not on the solver.
- `solver` depends on collision outputs and math primitives, but not on Python or rendering.
- `dynamics` owns world state and calls into collision and solver through typed data, not scene-graph objects.
- `sim` orchestrates the step and emits traces; it should be the only layer that knows the full frame pipeline.

That keeps the research layer thin. Python bindings, render tooling, and batched rollout systems should speak to `sim` and `dynamics`, not reach down into internal storage.

## First Solver / Contact Architecture

The first implementation should optimize for robustness and determinism, not for perfect generality.

### Chosen Baseline

- integration: semi-implicit Euler
- articulated dynamics: Featherstone-style articulated-body forward dynamics on CPU
- contact representation: persistent contact manifolds with up to 4 points per pair
- broadphase: deterministic sweep-and-prune on the x-axis using bounding volumes
- friction model: 2 tangent directions per point using a friction pyramid
- solver: warm-started sequential impulse / projected Gauss-Seidel at the velocity level with cached normal and tangential impulses
- stabilization: split impulse for penetration correction, with a small Baumgarte bias reserved for joints
- ordering: stable body-id and manifold-id ordering on CPU to preserve replayability

### Why This Baseline

- It supports robotics use cases without forcing a full nonlinear optimizer into the first milestone.
- Sequential impulse / PGS is iterative but simple enough to batch later.
- Persistent manifolds reduce solver noise and are a practical stepping stone to GPU contact batching.
- Semi-implicit Euler is not the final word, but it is a reliable baseline while contact and articulated dynamics are still maturing.

### Frame Pipeline

1. Build active islands from articulated trees and broadphase overlap pairs.
2. Run unconstrained articulated-body forward dynamics per island.
3. Predict velocities and generate broadphase candidates.
4. Run narrowphase and update persistent manifolds.
5. Convert joints and contacts into solver rows.
6. Warm-start cached impulses.
7. Run velocity solve with deterministic iteration order.
8. Apply split impulse position correction.
9. Integrate poses and update traces.

## CPU / GPU Split

The CPU path remains the truth path for debugging and regression tests.

Current regression policy:

- closed-form checks for semi-implicit free-fall and constant-angular-velocity rotation
- finite-difference checks for discrete transitions and kinematic Jacobians
- algebraic transform invariants such as `T * inverse(T) = I`
- randomized property tests for free-fall, elastic impacts, friction dissipation, and quaternion normalization
- replay and Parquet export round-trips so the research/debug surfaces stay consistent with the engine core

The first CUDA migration candidates are:

- broadphase pair generation
- contact candidate compaction
- batched articulated forward dynamics for homogeneous robot sets
- constraint row assembly

The interface rule is simple: GPU kernels operate on packed arrays owned by `dynamics` and `collision`; they do not own the high-level world model.

## Immediate Milestones

1. Flesh out SoA body/articulation storage in `dynamics`.
2. Add broadphase and manifold persistence to `collision`.
3. Replace the solver stub with real row assembly and warm-start caches.
4. Add a Python binding that returns batched observations and step traces.
5. Introduce a CUDA backend for one hot path only after CPU parity tests exist.
