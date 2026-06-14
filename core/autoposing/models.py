from dataclasses import dataclass, field


@dataclass(slots=True)
class AutoPosingControllerModel:
    identifier: str
    name: str
    controller_type: str
    joint_name: str
    world_position: tuple[float, float, float]
    default_position: tuple[float, float, float]
    visible: bool = True
    active: bool = False
    fixed: bool = False
    always_active: bool = False
    selected: bool = False


@dataclass(slots=True)
class AutoPosingRigModel:
    controllers: list[AutoPosingControllerModel] = field(default_factory=list)
    current_mode: str = "view"
    selected_controller_id: str | None = None
