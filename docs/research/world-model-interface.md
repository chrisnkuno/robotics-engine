# World-Model Interface

The research-facing path in this repo is intentionally Python-first:

- `rex_py`: low-level simulation state and stepping
- `python/rex_research.py`: paper metadata, action application, rollout harnesses, and planner/model adapters
- `python/rex_data.py`: replay export for notebook analysis

## Why This Layer Exists

Researchers should not need to edit the engine core to try:

- a new planner
- a latent dynamics model
- an action-conditioned predictor
- a paper-specific observation encoder

The intended workflow is:

1. Build a world and engine through `rex_py`
2. Wrap your model with `CallableWorldModel`
3. Drive rollouts with `ResearchHarness`
4. Run a planner such as `RandomShootingPlanner`
5. Save replays and export Parquet datasets for analysis

## Latest JEPA References

The current public Meta JEPA world-model stack that matters for this engine is:

- `V-JEPA 2`
  - official launch date: `2025-06-11`
  - official newsroom post: `https://about.fb.com/news/2025/06/our-new-model-helps-ai-think-before-it-acts/`
  - focus: understanding, prediction, and planning in the physical world

- `V-JEPA 2-AC`
  - official public code path: `https://github.com/facebookresearch/vjepa2`
  - focus: latent action-conditioned world model post-trained on a small amount of robot interaction data
  - relevant use case: planning from image goals

- `V-JEPA 2.1`
  - official public repo release note date: `2026-03-16`
  - repo: `https://github.com/facebookresearch/vjepa2`
  - focus: higher-quality and temporally consistent dense features

This matters because the engine interface should support:

- video/image-conditioned encoders
- latent rollouts
- action-conditioned prediction
- goal-conditioned planning
- replay export for offline analysis

## Minimal JEPA Workflow

```python
import rex_py
from rex_research import (
    BodyCommand,
    CallableWorldModel,
    RandomShootingPlanner,
    ResearchHarness,
    latest_jepa_world_model_specs,
    load_vjepa2_torch_hub,
)

specs = latest_jepa_world_model_specs()
bundle = load_vjepa2_torch_hub()  # requires local torch + cached/downloadable repo

engine = rex_py.Engine(rex_py.EngineConfig())
world = rex_py.World()

harness = ResearchHarness(rex_py, engine, world)

world_model = CallableWorldModel(
    paper=specs["vjepa2_ac"],
    encode_observation_fn=lambda obs: ...,
    predict_next_latent_fn=lambda latent, action: ...,
    score_goal_fn=lambda latent, goal: ...,
)

planner = RandomShootingPlanner(
    horizon=4,
    candidates=64,
    sample_action_sequences=lambda obs, horizon, candidates: ...,
)
```

The deliberate separation is:

- `bundle`: external paper/model assets
- `CallableWorldModel`: engine-facing adapter
- `ResearchHarness`: simulation loop
- `RandomShootingPlanner`: planner over latent dynamics

## Current Limits

This repo still exposes a deliberately simple action surface:

- direct body pose updates
- direct body linear/angular velocity updates

That is enough for fast idea iteration, latent rollouts, and planner bring-up.
The next research-side extension should be actuator/task abstractions once articulated control lands.
