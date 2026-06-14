from dataclasses import dataclass


@dataclass(slots=True)
class SceneModel:
    name: str = "Scene"
    unit_name: str = "centimeter"
