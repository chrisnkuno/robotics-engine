from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable, Iterable, Protocol, Sequence


@dataclass(frozen=True)
class PaperSpec:
    key: str
    title: str
    release_date: str
    url: str
    code_url: str
    summary: str
    tags: tuple[str, ...] = ()


@dataclass(frozen=True)
class BodyCommand:
    body_index: int
    translation: Any | None = None
    rotation: Any | None = None
    linear_velocity: Any | None = None
    angular_velocity: Any | None = None

    def empty(self) -> bool:
        return (
            self.translation is None
            and self.rotation is None
            and self.linear_velocity is None
            and self.angular_velocity is None
        )


@dataclass
class ResearchObservation:
    step_index: int
    sim_time: float
    trace: Any
    frame: Any
    bodies: list[dict[str, Any]]
    contacts: list[Any]


@dataclass
class RolloutResult:
    observations: list[ResearchObservation] = field(default_factory=list)
    actions: list[list[BodyCommand]] = field(default_factory=list)
    replay: Any | None = None


class WorldModelProtocol(Protocol):
    paper: PaperSpec

    def encode_observation(self, observation: ResearchObservation) -> Any:
        ...

    def predict_next_latent(self, latent: Any, action: Sequence[BodyCommand]) -> Any:
        ...

    def score_goal(self, latent: Any, goal: Any) -> float:
        ...


@dataclass
class CallableWorldModel:
    paper: PaperSpec
    encode_observation_fn: Callable[[ResearchObservation], Any]
    predict_next_latent_fn: Callable[[Any, Sequence[BodyCommand]], Any]
    score_goal_fn: Callable[[Any, Any], float]

    def encode_observation(self, observation: ResearchObservation) -> Any:
        return self.encode_observation_fn(observation)

    def predict_next_latent(self, latent: Any, action: Sequence[BodyCommand]) -> Any:
        return self.predict_next_latent_fn(latent, action)

    def score_goal(self, latent: Any, goal: Any) -> float:
        return float(self.score_goal_fn(latent, goal))


@dataclass
class RandomShootingPlanner:
    horizon: int
    candidates: int
    sample_action_sequences: Callable[
        [ResearchObservation, int, int],
        Iterable[Sequence[Sequence[BodyCommand]]],
    ]
    objective_fn: Callable[[Any, Any], float] | None = None

    def plan(
        self,
        world_model: WorldModelProtocol,
        observation: ResearchObservation,
        goal: Any,
    ) -> list[list[BodyCommand]]:
        best_sequence: list[list[BodyCommand]] = []
        best_score = float("-inf")
        initial_latent = world_model.encode_observation(observation)

        for candidate in self.sample_action_sequences(
            observation,
            self.horizon,
            self.candidates,
        ):
            latent = initial_latent
            for action in candidate:
                latent = world_model.predict_next_latent(latent, action)

            objective = self.objective_fn or world_model.score_goal
            score = float(objective(latent, goal))
            if score > best_score:
                best_score = score
                best_sequence = [list(step_actions) for step_actions in candidate]

        return best_sequence


@dataclass(frozen=True)
class JEPATorchHubBundle:
    paper: PaperSpec
    processor: Any
    encoder: Any
    action_conditioned_encoder: Any
    action_conditioned_predictor: Any
    repo: str = "facebookresearch/vjepa2"


def latest_jepa_world_model_specs() -> dict[str, PaperSpec]:
    return {
        "vjepa2": PaperSpec(
            key="vjepa2",
            title="V-JEPA 2",
            release_date="2025-06-11",
            url="https://about.fb.com/news/2025/06/our-new-model-helps-ai-think-before-it-acts/",
            code_url="https://github.com/facebookresearch/vjepa2",
            summary=(
                "Meta's official world-model release focused on understanding, prediction "
                "and planning in the physical world."
            ),
            tags=("video", "world-model", "prediction", "planning"),
        ),
        "vjepa2_ac": PaperSpec(
            key="vjepa2_ac",
            title="V-JEPA 2-AC",
            release_date="2025-06-25",
            url="https://github.com/facebookresearch/vjepa2",
            code_url="https://github.com/facebookresearch/vjepa2",
            summary=(
                "Meta's latent action-conditioned world model post-trained on a small amount "
                "of robot interaction data and used for planning from image goals."
            ),
            tags=("video", "world-model", "action-conditioned", "robotics", "planning"),
        ),
        "vjepa2_1": PaperSpec(
            key="vjepa2_1",
            title="V-JEPA 2.1",
            release_date="2026-03-16",
            url="https://github.com/facebookresearch/vjepa2",
            code_url="https://github.com/facebookresearch/vjepa2",
            summary=(
                "The latest official V-JEPA family release in the public codebase, focused "
                "on higher-quality and temporally consistent dense features."
            ),
            tags=("video", "representation-learning", "dense-features", "temporal-consistency"),
        ),
    }


def load_vjepa2_torch_hub(
    torch_module: Any | None = None,
    *,
    repo: str = "facebookresearch/vjepa2",
    encoder_name: str = "vjepa2_1_vit_large_384",
    action_conditioned_name: str = "vjepa2_ac_vit_giant",
) -> JEPATorchHubBundle:
    if torch_module is None:
        import torch as torch_module  # type: ignore[no-redef]

    processor = torch_module.hub.load(repo, "vjepa2_preprocessor")
    encoder = torch_module.hub.load(repo, encoder_name)
    action_conditioned_encoder, action_conditioned_predictor = torch_module.hub.load(
        repo,
        action_conditioned_name,
    )
    return JEPATorchHubBundle(
        paper=latest_jepa_world_model_specs()["vjepa2_ac"],
        processor=processor,
        encoder=encoder,
        action_conditioned_encoder=action_conditioned_encoder,
        action_conditioned_predictor=action_conditioned_predictor,
        repo=repo,
    )


def apply_body_commands(world: Any, commands: Sequence[BodyCommand]) -> None:
    for command in commands:
        if command.translation is not None and command.rotation is not None:
            world.set_pose(command.body_index, command.translation, command.rotation)
        elif command.translation is not None:
            world.set_translation(command.body_index, command.translation)
        elif command.rotation is not None:
            world.set_rotation(command.body_index, command.rotation)

        if command.linear_velocity is not None:
            world.set_linear_velocity(command.body_index, command.linear_velocity)
        if command.angular_velocity is not None:
            world.set_angular_velocity(command.body_index, command.angular_velocity)


class ResearchHarness:
    def __init__(self, rex_py: Any, engine: Any, world: Any, *, capture_replay: bool = True):
        self.rex_py = rex_py
        self.engine = engine
        self.world = world
        self.capture_replay = capture_replay
        self.replay = rex_py.ReplayLog() if capture_replay else None
        self.step_index = 0
        self.sim_time = 0.0
        self.dt = float(engine.config().simulation.step.dt)

    def _capture_observation(self, trace: Any) -> ResearchObservation:
        frame = self.rex_py.capture_frame(
            self.world,
            trace,
            frame_index=self.step_index,
            sim_time=self.sim_time,
        )
        if self.replay is not None:
            self.replay.add_frame(frame)

        observation = ResearchObservation(
            step_index=self.step_index,
            sim_time=self.sim_time,
            trace=trace,
            frame=frame,
            bodies=[self.world.body(index) for index in range(self.world.body_count)],
            contacts=list(frame.contacts),
        )
        self.step_index += 1
        self.sim_time += self.dt
        return observation

    def step(self, commands: Sequence[BodyCommand] = ()) -> ResearchObservation:
        apply_body_commands(self.world, commands)
        trace = self.engine.step(self.world)
        return self._capture_observation(trace)

    def rollout(
        self,
        controller: Callable[[ResearchObservation | None, "ResearchHarness"], Sequence[BodyCommand]],
        steps: int,
    ) -> RolloutResult:
        result = RolloutResult(replay=self.replay)
        previous_observation: ResearchObservation | None = None
        for _ in range(steps):
            commands = list(controller(previous_observation, self))
            observation = self.step(commands)
            result.actions.append(commands)
            result.observations.append(observation)
            previous_observation = observation

        return result
