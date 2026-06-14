from dataclasses import dataclass, field


@dataclass(slots=True)
class KeyframeModel:
    frame: int
    value: float


@dataclass(slots=True)
class AnimationClipModel:
    name: str = "Take001"
    frame_count: int = 0
    keyframes: list[KeyframeModel] = field(default_factory=list)
