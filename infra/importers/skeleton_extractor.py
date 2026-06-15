import math
import re
from dataclasses import dataclass, field
from pathlib import Path

from core.math import length, normalize
from core.skeleton.models import BoneModel, SkeletonModel


_JOINTS_PATTERN = re.compile(r"Skin\s*\{.*?joints:\s*\[(.*?)\]", re.DOTALL)
_IDENTIFIER_PATTERN = re.compile(r"\b([A-Za-z_][A-Za-z0-9_]*)\b")
_BLOCK_OPEN_PATTERN = re.compile(r"([A-Za-z_][A-Za-z0-9_]*)\s*\{")
_ID_PATTERN = re.compile(r"id:\s*([A-Za-z_][A-Za-z0-9_]*)")
_OBJECT_NAME_PATTERN = re.compile(r'objectName:\s*"([^"]+)"')
_VECTOR_PATTERN = re.compile(r"Qt\.vector3d\(([^)]+)\)")
_QUATERNION_PATTERN = re.compile(r"Qt\.quaternion\(([^)]+)\)")
_HIDDEN_JOINT_NAMES = {
    "root",
    "ik_foot_root",
    "ik_foot_l",
    "ik_foot_r",
    "ik_hand_root",
    "ik_hand_gun",
    "ik_hand_l",
    "ik_hand_r",
    "interaction",
    "center_of_mass",
}


@dataclass(slots=True)
class _ParsedNode:
    node_id: str | None = None
    object_name: str | None = None
    position: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rotation: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)
    children: list["_ParsedNode"] = field(default_factory=list)


def _parse_float_tuple(raw_values: str, expected_size: int) -> tuple[float, ...] | None:
    parts = [part.strip() for part in raw_values.split(",")]
    if len(parts) != expected_size:
        return None

    try:
        return tuple(float(part) for part in parts)
    except ValueError:
        return None


def _parse_joint_ids(qml_text: str) -> list[str]:
    match = _JOINTS_PATTERN.search(qml_text)
    if not match:
        return []
    return _IDENTIFIER_PATTERN.findall(match.group(1))


def _parse_node_tree(qml_text: str) -> list[_ParsedNode]:
    root_nodes: list[_ParsedNode] = []
    block_stack: list[tuple[str, _ParsedNode | None]] = []

    for raw_line in qml_text.splitlines():
        stripped = raw_line.strip()
        if not stripped:
            continue

        block_match = _BLOCK_OPEN_PATTERN.match(stripped)
        if block_match:
            block_type = block_match.group(1)
            if block_type == "Node":
                node = _ParsedNode()
                parent_node = next(
                    (stack_node for stack_type, stack_node in reversed(block_stack) if stack_type == "Node"),
                    None,
                )
                if parent_node is None:
                    root_nodes.append(node)
                else:
                    parent_node.children.append(node)
                block_stack.append((block_type, node))
            else:
                block_stack.append((block_type, None))
            continue

        if stripped == "}":
            if block_stack:
                block_stack.pop()
            continue

        if not block_stack or block_stack[-1][0] != "Node":
            continue

        current_node = block_stack[-1][1]
        if current_node is None:
            continue

        id_match = _ID_PATTERN.match(stripped)
        if id_match:
            current_node.node_id = id_match.group(1)
            continue

        object_name_match = _OBJECT_NAME_PATTERN.match(stripped)
        if object_name_match:
            current_node.object_name = object_name_match.group(1)
            continue

        if stripped.startswith("position:"):
            vector_match = _VECTOR_PATTERN.search(stripped)
            if vector_match:
                parsed = _parse_float_tuple(vector_match.group(1), 3)
                if parsed is not None:
                    current_node.position = parsed  # type: ignore[assignment]
            continue

        if stripped.startswith("rotation:"):
            quaternion_match = _QUATERNION_PATTERN.search(stripped)
            if quaternion_match:
                parsed = _parse_float_tuple(quaternion_match.group(1), 4)
                if parsed is not None:
                    current_node.rotation = parsed  # type: ignore[assignment]

    return root_nodes


def _quaternion_multiply(
    left: tuple[float, float, float, float],
    right: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    lw, lx, ly, lz = left
    rw, rx, ry, rz = right
    return (
        lw * rw - lx * rx - ly * ry - lz * rz,
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
    )


def _quaternion_conjugate(quaternion: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    w, x, y, z = quaternion
    return (w, -x, -y, -z)


def _rotate_vector(
    quaternion: tuple[float, float, float, float],
    vector: tuple[float, float, float],
) -> tuple[float, float, float]:
    pure_vector = (0.0, vector[0], vector[1], vector[2])
    rotated = _quaternion_multiply(_quaternion_multiply(quaternion, pure_vector), _quaternion_conjugate(quaternion))
    return (rotated[1], rotated[2], rotated[3])


def _add_vectors(
    left: tuple[float, float, float],
    right: tuple[float, float, float],
) -> tuple[float, float, float]:
    return (left[0] + right[0], left[1] + right[1], left[2] + right[2])


def _normalize_quaternion(quaternion: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    w, x, y, z = quaternion
    length = math.sqrt(w * w + x * x + y * y + z * z)
    if length <= 1e-8:
        return (1.0, 0.0, 0.0, 0.0)
    return (w / length, x / length, y / length, z / length)


def _build_skeleton_from_nodes(joint_ids: list[str], root_nodes: list[_ParsedNode]) -> SkeletonModel:
    joint_set = set(joint_ids)
    local_positions: dict[str, tuple[float, float, float]] = {}
    local_rotations: dict[str, tuple[float, float, float, float]] = {}
    world_positions: dict[str, tuple[float, float, float]] = {}
    world_rotations: dict[str, tuple[float, float, float, float]] = {}
    ancestry: dict[str, str | None] = {}
    object_names: dict[str, str] = {}

    def walk(
        node: _ParsedNode,
        parent_position: tuple[float, float, float],
        parent_rotation: tuple[float, float, float, float],
        parent_node_id: str | None,
    ) -> None:
        current_rotation = _normalize_quaternion(_quaternion_multiply(parent_rotation, node.rotation))
        current_position = _add_vectors(parent_position, _rotate_vector(parent_rotation, node.position))

        current_parent_id = parent_node_id
        if node.node_id:
            nearest_joint_parent = current_parent_id
            while nearest_joint_parent is not None and nearest_joint_parent not in joint_set:
                nearest_joint_parent = ancestry.get(nearest_joint_parent)

            ancestry[node.node_id] = nearest_joint_parent
            if node.node_id in joint_set:
                local_positions[node.node_id] = node.position
                local_rotations[node.node_id] = node.rotation
                world_positions[node.node_id] = current_position
                world_rotations[node.node_id] = current_rotation
                object_names[node.node_id] = node.object_name or node.node_id
            current_parent_id = node.node_id

        for child in node.children:
            walk(child, current_position, current_rotation, current_parent_id)

    identity_rotation = (1.0, 0.0, 0.0, 0.0)
    origin = (0.0, 0.0, 0.0)
    for root_node in root_nodes:
        walk(root_node, origin, identity_rotation, None)

    bones: list[BoneModel] = []
    for joint_id in joint_ids:
        if joint_id not in world_positions:
            continue
        object_name = object_names.get(joint_id, joint_id)
        if object_name in _HIDDEN_JOINT_NAMES:
            continue

        world_position = world_positions[joint_id]
        parent_id = ancestry.get(joint_id)
        parent_name = object_names.get(parent_id, parent_id) if parent_id else None
        if parent_name in _HIDDEN_JOINT_NAMES:
            parent_name = None

        local_position = local_positions.get(joint_id, (0.0, 0.0, 0.0))
        local_rotation = local_rotations.get(joint_id, (1.0, 0.0, 0.0, 0.0))
        world_position = world_positions[joint_id]
        world_rotation = world_rotations.get(joint_id, (1.0, 0.0, 0.0, 0.0))
        bone_length = length(local_position)
        primary_axis = normalize(local_position) if bone_length > 1e-6 else (1.0, 0.0, 0.0)

        bones.append(
            BoneModel(
                name=object_name,
                parent_name=parent_name,
                local_position=local_position,
                local_rotation=local_rotation,
                world_position=world_position,
                world_rotation=world_rotation,
                rest_local_position=local_position,
                rest_local_rotation=local_rotation,
                rest_world_position=world_position,
                rest_world_rotation=world_rotation,
                bone_length=bone_length,
                primary_axis=primary_axis,
            )
        )

    return SkeletonModel(name="Rig", bones=bones)


def extract_skeleton_from_generated_component(
    generated_component_path: Path | None,
    status_lines: list[str] | None = None,
) -> SkeletonModel:
    messages = status_lines if status_lines is not None else []

    if generated_component_path is None or not generated_component_path.exists():
        messages.append("No generated component available for skeleton extraction.")
        return SkeletonModel(name="Rig")

    try:
        qml_text = generated_component_path.read_text(encoding="utf-8")
    except OSError as exc:
        messages.append(f"Could not read generated component for skeleton extraction: {exc}")
        return SkeletonModel(name="Rig")

    joint_ids = _parse_joint_ids(qml_text)
    if not joint_ids:
        messages.append("No skeleton joints were found in generated component.")
        return SkeletonModel(name="Rig")

    root_nodes = _parse_node_tree(qml_text)
    skeleton = _build_skeleton_from_nodes(joint_ids, root_nodes)
    messages.append(f"Extracted {len(skeleton.bones)} skeleton joints from generated component.")
    return skeleton
