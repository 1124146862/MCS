from __future__ import annotations

from dataclasses import dataclass, field

from core.math import add, distance, length, normalize, scale, subtract
from core.skeleton.models import BoneModel, SkeletonModel, Vec3

from .controller_layout import direction_endpoint


@dataclass(frozen=True, slots=True)
class AutoPosingControllerDefinition:
    identifier: str
    name: str
    role: str
    joint_name: str
    world_position: Vec3
    visible: bool = True
    always_active: bool = False


@dataclass(frozen=True, slots=True)
class AutoPosingControllerEdgeDefinition:
    start_controller_id: str
    end_controller_id: str
    edge_type: str = "topology"
    visible: bool = True


@dataclass(slots=True)
class AutoPosingControllerGraphDefinition:
    controllers: list[AutoPosingControllerDefinition] = field(default_factory=list)
    edges: list[AutoPosingControllerEdgeDefinition] = field(default_factory=list)


JOINT_ALIASES = {
    "pelvis": ("pelvis",),
    "stomach": ("spine_02", "spine_03", "spine_01"),
    "chest": ("spine_05", "spine_04", "spine_03", "spine_02"),
    "neck": ("neck_01", "neck_02", "spine_05"),
    "head": ("head",),
    "shoulder_l": ("upperarm_l", "clavicle_l"),
    "shoulder_r": ("upperarm_r", "clavicle_r"),
    "elbow_l": ("lowerarm_l",),
    "elbow_r": ("lowerarm_r",),
    "wrist_l": ("hand_l",),
    "wrist_r": ("hand_r",),
    "hip_l": ("thigh_l",),
    "hip_r": ("thigh_r",),
    "knee_l": ("calf_l",),
    "knee_r": ("calf_r",),
    "ankle_l": ("foot_l",),
    "ankle_r": ("foot_r",),
    "ball_l": ("ball_l",),
    "ball_r": ("ball_r",),
}

ALWAYS_ACTIVE_IDS = {"ankle_l_main", "ankle_r_main"}


def build_cascadeur_like_controller_schema(skeleton: SkeletonModel) -> AutoPosingControllerGraphDefinition:
    rest_by_name = {bone.name: bone for bone in skeleton.bones}
    graph = AutoPosingControllerGraphDefinition()

    def add_controller(identifier: str, name: str, role: str, semantic: str, position: Vec3 | None = None) -> None:
        joint = _resolve(rest_by_name, semantic)
        if joint is None:
            return
        graph.controllers.append(
            AutoPosingControllerDefinition(
                identifier=identifier,
                name=name,
                role=role,
                joint_name=joint.name,
                world_position=position if position is not None else joint.rest_world_position,
                always_active=identifier in ALWAYS_ACTIVE_IDS,
            )
        )

    def add_edge(start: str, end: str) -> None:
        graph.edges.append(AutoPosingControllerEdgeDefinition(start_controller_id=start, end_controller_id=end))

    side_axis = _side_axis(rest_by_name)
    pelvis = _position(rest_by_name, "pelvis")
    chest = _position(rest_by_name, "chest")
    head = _position(rest_by_name, "head")
    stomach = _position(rest_by_name, "stomach")
    neck = _position(rest_by_name, "neck")

    add_controller("pelvis_main", "Pelvis", "main", "pelvis")
    add_controller("pelvis_additional", "Pelvis Additional", "additional", "pelvis", add(pelvis, scale(side_axis, 26.0)))
    add_controller("stomach_secondary", "Stomach", "secondary", "stomach")
    add_controller("chest_secondary", "Chest", "secondary", "chest")
    add_controller("chest_additional", "Chest Additional", "additional", "chest", add(chest, scale(side_axis, 34.0)))
    add_controller("neck_lesser", "Neck", "lesser", "neck", neck)
    add_controller("head_main", "Head", "main", "head")
    add_controller("head_additional", "Head Additional", "additional", "head", add(head, scale(side_axis, 28.0)))

    add_edge("pelvis_main", "stomach_secondary")
    add_edge("stomach_secondary", "chest_secondary")
    add_edge("chest_secondary", "neck_lesser")
    add_edge("neck_lesser", "head_main")
    add_edge("pelvis_main", "pelvis_additional")
    add_edge("stomach_secondary", "pelvis_additional")
    add_edge("chest_secondary", "chest_additional")
    add_edge("head_main", "head_additional")

    for side in ("l", "r"):
        _add_arm(graph, rest_by_name, side)
        _add_leg(graph, rest_by_name, side)

    for identifier, name, semantic in (
        ("pelvis_dir", "Pelvis Direction", "pelvis"),
        ("chest_dir", "Chest Direction", "chest"),
        ("head_dir", "Head Direction", "head"),
    ):
        joint = _resolve(rest_by_name, semantic)
        if joint is None:
            continue
        graph.controllers.append(
            AutoPosingControllerDefinition(
                identifier=identifier,
                name=name,
                role="direction",
                joint_name=joint.name,
                world_position=direction_endpoint(identifier, joint.rest_world_position, add(joint.rest_world_position, (0.0, 0.0, 20.0))),
            )
        )

    return _filter_graph(graph)


def _add_arm(graph: AutoPosingControllerGraphDefinition, rest_by_name: dict[str, BoneModel], side: str) -> None:
    suffix = "L" if side == "l" else "R"
    shoulder = _position(rest_by_name, f"shoulder_{side}")
    elbow = _position(rest_by_name, f"elbow_{side}")
    wrist = _position(rest_by_name, f"wrist_{side}")
    distal = _direction(elbow, wrist, (0.0, 0.0, 1.0))
    outward = _outward_direction(rest_by_name, wrist)
    hand = add(wrist, scale(distal, 12.0))
    hand_additional = add(hand, scale(outward, 10.0))

    _append_controller(graph, f"shoulder_{side}_lesser", f"Shoulder {suffix}", "lesser", rest_by_name, f"shoulder_{side}", shoulder)
    _append_controller(graph, f"elbow_{side}_secondary", f"Elbow {suffix}", "lesser", rest_by_name, f"elbow_{side}", elbow)
    _append_controller(graph, f"wrist_{side}_main", f"Wrist {suffix}", "main", rest_by_name, f"wrist_{side}", wrist)
    _append_controller(graph, f"hand_{side}_secondary", f"Hand {suffix}", "secondary", rest_by_name, f"wrist_{side}", hand)
    _append_controller(graph, f"hand_{side}_additional", f"Hand Additional {suffix}", "additional", rest_by_name, f"wrist_{side}", hand_additional)

    _append_edges(
        graph,
        (
            (f"chest_secondary", f"shoulder_{side}_lesser"),
            (f"shoulder_{side}_lesser", f"elbow_{side}_secondary"),
            (f"elbow_{side}_secondary", f"wrist_{side}_main"),
            (f"wrist_{side}_main", f"hand_{side}_secondary"),
            (f"wrist_{side}_main", f"hand_{side}_additional"),
            (f"hand_{side}_secondary", f"hand_{side}_additional"),
        ),
    )


def _add_leg(graph: AutoPosingControllerGraphDefinition, rest_by_name: dict[str, BoneModel], side: str) -> None:
    suffix = "L" if side == "l" else "R"
    hip = _position(rest_by_name, f"hip_{side}")
    knee = _position(rest_by_name, f"knee_{side}")
    ankle = _position(rest_by_name, f"ankle_{side}")
    ball_joint = _resolve(rest_by_name, f"ball_{side}")
    foot_direction = _direction(ankle, ball_joint.rest_world_position if ball_joint else add(ankle, (0.0, 0.0, 1.0)), (0.0, 0.0, 1.0))
    foot = ball_joint.rest_world_position if ball_joint else add(ankle, scale(foot_direction, 14.0))
    ball = add(foot, scale(foot_direction, 10.0))
    outward = _outward_direction(rest_by_name, ankle)
    foot_additional = add(foot, scale(outward, 9.0))

    _append_controller(graph, f"hip_{side}_lesser", f"Hip {suffix}", "lesser", rest_by_name, f"hip_{side}", hip)
    _append_controller(graph, f"knee_{side}_secondary", f"Knee {suffix}", "lesser", rest_by_name, f"knee_{side}", knee)
    _append_controller(graph, f"ankle_{side}_main", f"Ankle {suffix}", "main", rest_by_name, f"ankle_{side}", ankle)
    _append_controller(graph, f"foot_{side}_secondary", f"Foot {suffix}", "secondary", rest_by_name, f"ball_{side}" if ball_joint else f"ankle_{side}", foot)
    if ball_joint is not None:
        _append_controller(graph, f"ball_{side}_lesser", f"Ball {suffix}", "lesser", rest_by_name, f"ball_{side}", ball)
    _append_controller(graph, f"foot_{side}_additional", f"Foot Additional {suffix}", "additional", rest_by_name, f"ball_{side}" if ball_joint else f"ankle_{side}", foot_additional)

    _append_edges(
        graph,
        (
            ("pelvis_main", f"hip_{side}_lesser"),
            (f"hip_{side}_lesser", f"knee_{side}_secondary"),
            (f"knee_{side}_secondary", f"ankle_{side}_main"),
            (f"ankle_{side}_main", f"foot_{side}_secondary"),
            (f"foot_{side}_secondary", f"ball_{side}_lesser"),
            (f"foot_{side}_secondary", f"foot_{side}_additional"),
            (f"ankle_{side}_main", f"foot_{side}_additional"),
        ),
    )


def _append_controller(
    graph: AutoPosingControllerGraphDefinition,
    identifier: str,
    name: str,
    role: str,
    rest_by_name: dict[str, BoneModel],
    semantic: str,
    position: Vec3,
) -> None:
    joint = _resolve(rest_by_name, semantic)
    if joint is None:
        return
    graph.controllers.append(
        AutoPosingControllerDefinition(
            identifier=identifier,
            name=name,
            role=role,
            joint_name=joint.name,
            world_position=position,
            always_active=identifier in ALWAYS_ACTIVE_IDS,
        )
    )


def _append_edges(graph: AutoPosingControllerGraphDefinition, edges: tuple[tuple[str, str], ...]) -> None:
    for start, end in edges:
        graph.edges.append(AutoPosingControllerEdgeDefinition(start, end))


def _filter_graph(graph: AutoPosingControllerGraphDefinition) -> AutoPosingControllerGraphDefinition:
    controller_ids = {controller.identifier for controller in graph.controllers}
    graph.edges = [
        edge
        for edge in graph.edges
        if edge.start_controller_id in controller_ids and edge.end_controller_id in controller_ids
    ]
    return graph


def _resolve(rest_by_name: dict[str, BoneModel], semantic: str) -> BoneModel | None:
    for alias in JOINT_ALIASES.get(semantic, (semantic,)):
        joint = rest_by_name.get(alias)
        if joint is not None:
            return joint
    return None


def _position(rest_by_name: dict[str, BoneModel], semantic: str) -> Vec3:
    joint = _resolve(rest_by_name, semantic)
    return joint.rest_world_position if joint is not None else (0.0, 0.0, 0.0)


def _side_axis(rest_by_name: dict[str, BoneModel]) -> Vec3:
    left = _resolve(rest_by_name, "hip_l") or _resolve(rest_by_name, "shoulder_l")
    right = _resolve(rest_by_name, "hip_r") or _resolve(rest_by_name, "shoulder_r")
    if left is not None and right is not None:
        axis = normalize(subtract(right.rest_world_position, left.rest_world_position))
        if length(axis) > 0.0:
            return axis
    return (1.0, 0.0, 0.0)


def _outward_direction(rest_by_name: dict[str, BoneModel], anchor: Vec3) -> Vec3:
    center = _position(rest_by_name, "chest")
    if distance(anchor, center) <= 0.001:
        center = _position(rest_by_name, "pelvis")
    direction = normalize(subtract(anchor, center))
    if length(direction) > 0.0:
        return direction
    return _side_axis(rest_by_name)


def _direction(start: Vec3, end: Vec3, fallback: Vec3) -> Vec3:
    direction = normalize(subtract(end, start))
    if length(direction) > 0.0:
        return direction
    fallback_direction = normalize(fallback)
    return fallback_direction if length(fallback_direction) > 0.0 else (0.0, 0.0, 1.0)
