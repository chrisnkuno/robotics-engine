import importlib.util
import pathlib
import sys


def load_module(module_name: str, module_path: pathlib.Path):
    spec = importlib.util.spec_from_file_location(module_name, module_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"failed to load {module_name} from {module_path}")

    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module


class FakeHub:
    def __init__(self):
        self.calls = []

    def load(self, repo, name):
        self.calls.append((repo, name))
        if name == "vjepa2_ac_vit_giant":
            return "ac-encoder", "ac-predictor"
        return f"loaded:{name}"


class FakeTorch:
    def __init__(self):
        self.hub = FakeHub()


def main() -> int:
    if len(sys.argv) != 3:
        raise RuntimeError(
            "usage: python_research_test.py <rex_py-module-path> <rex_research-module-path>"
        )

    rex_py = load_module("rex_py", pathlib.Path(sys.argv[1]))
    rex_research = load_module("rex_research", pathlib.Path(sys.argv[2]))

    specs = rex_research.latest_jepa_world_model_specs()
    assert specs["vjepa2"].release_date == "2025-06-11"
    assert specs["vjepa2_ac"].key == "vjepa2_ac"
    assert specs["vjepa2_1"].release_date == "2026-03-16"

    fake_torch = FakeTorch()
    bundle = rex_research.load_vjepa2_torch_hub(fake_torch)
    assert bundle.processor == "loaded:vjepa2_preprocessor"
    assert bundle.encoder == "loaded:vjepa2_1_vit_large_384"
    assert bundle.action_conditioned_encoder == "ac-encoder"
    assert bundle.action_conditioned_predictor == "ac-predictor"
    assert fake_torch.hub.calls == [
        ("facebookresearch/vjepa2", "vjepa2_preprocessor"),
        ("facebookresearch/vjepa2", "vjepa2_1_vit_large_384"),
        ("facebookresearch/vjepa2", "vjepa2_ac_vit_giant"),
    ]

    config = rex_py.EngineConfig()
    config.simulation.gravity = rex_py.Vec3(0.0, 0.0, 0.0)
    config.simulation.step.dt = 0.05
    engine = rex_py.Engine(config)

    world = rex_py.World()
    world.add_box(
        position=rex_py.Vec3(0.0, 0.0, 0.0),
        half_extents=rex_py.Vec3(0.5, 0.5, 0.5),
        inverse_mass=0.0,
    )
    world.add_sphere(
        position=rex_py.Vec3(1.1, 0.0, 0.0),
        radius=0.4,
        linear_velocity=rex_py.Vec3(-0.2, 0.0, 0.0),
    )

    harness = rex_research.ResearchHarness(rex_py, engine, world, capture_replay=True)

    def controller(previous_observation, harness_instance):
        if previous_observation is None:
            return [
                rex_research.BodyCommand(
                    body_index=1,
                    linear_velocity=rex_py.Vec3(-0.3, 0.0, 0.1),
                )
            ]

        return [
            rex_research.BodyCommand(
                body_index=1,
                linear_velocity=rex_py.Vec3(-0.1, 0.0, 0.0),
            )
        ]

    rollout = harness.rollout(controller, steps=2)
    assert len(rollout.observations) == 2
    assert len(rollout.actions) == 2
    assert rollout.replay is not None
    assert rollout.replay.size() == 2
    assert rollout.observations[0].trace.profile.total_ms >= 0.0
    assert rollout.observations[1].bodies[1]["shape"] == "sphere"

    model = rex_research.CallableWorldModel(
        paper=specs["vjepa2_ac"],
        encode_observation_fn=lambda observation: observation.step_index,
        predict_next_latent_fn=lambda latent, action: latent + len(action),
        score_goal_fn=lambda latent, goal: -abs(goal - latent),
    )
    planner = rex_research.RandomShootingPlanner(
        horizon=2,
        candidates=2,
        sample_action_sequences=lambda observation, horizon, candidates: [
            [
                [
                    rex_research.BodyCommand(
                        body_index=1,
                        linear_velocity=rex_py.Vec3(-0.2, 0.0, 0.0),
                    )
                ]
                for _ in range(horizon)
            ],
            [[] for _ in range(horizon)],
        ],
    )
    best_sequence = planner.plan(model, rollout.observations[-1], goal=3)
    assert len(best_sequence) == 2
    assert len(best_sequence[0]) == 1

    rex_research.apply_body_commands(
        world,
        [
            rex_research.BodyCommand(
                body_index=1,
                translation=rex_py.Vec3(1.4, 0.0, 0.0),
                rotation=rex_py.Quat(),
                linear_velocity=rex_py.Vec3(0.0, 0.0, 0.0),
                angular_velocity=rex_py.Vec3(0.0, 0.0, 0.0),
            )
        ],
    )
    moved_body = world.body(1)
    assert abs(moved_body["translation"].x - 1.4) < 1.0e-9

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
