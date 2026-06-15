import math
import sys
from dataclasses import dataclass
from pathlib import Path

from PySide6.QtCore import QUrl
from PySide6.QtGui import QGuiApplication
from PySide6.QtQml import QQmlComponent, QQmlEngine

from app.autoposing.runtime_retargeter import RuntimeRetargeter
from app.autoposing.service import AutoPosingService
from app.document.document_manager import DocumentManager
from app.document.models import DocumentModel
from core.autoposing.models import RetargetPoseModel, SolvedPoseModel
from core.math import distance
from core.rig.models import RigModel
from core.skeleton.models import BoneModel, SkeletonModel
from infra.runtime.runtime_pose_writer import RuntimePoseWriter


_CHECKED_LIMB_CHAINS = (
    ("upperarm_l", "lowerarm_l"),
    ("lowerarm_l", "hand_l"),
    ("upperarm_r", "lowerarm_r"),
    ("lowerarm_r", "hand_r"),
    ("thigh_l", "calf_l"),
    ("calf_l", "foot_l"),
    ("thigh_r", "calf_r"),
    ("calf_r", "foot_r"),
)
_MAIN_RUNTIME_BONES = (
    "pelvis",
    "spine_01",
    "spine_02",
    "spine_03",
    "spine_04",
    "spine_05",
    "neck_01",
    "neck_02",
    "head",
    "clavicle_l",
    "upperarm_l",
    "lowerarm_l",
    "hand_l",
    "clavicle_r",
    "upperarm_r",
    "lowerarm_r",
    "hand_r",
    "thigh_l",
    "calf_l",
    "foot_l",
    "thigh_r",
    "calf_r",
    "foot_r",
)
_MAX_RUNTIME_ALIGNMENT_ERROR = 10.0


@dataclass(slots=True)
class CheckResult:
    name: str
    passed: bool
    details: str


@dataclass(slots=True)
class CheckContext:
    document: DocumentModel
    service: AutoPosingService
    retargeter: RuntimeRetargeter
    writer: RuntimePoseWriter
    engine: QQmlEngine | None
    component: QQmlComponent | None
    runtime_root: object | None


def _ensure_app() -> QGuiApplication:
    app = QGuiApplication.instance()
    if app is not None:
        return app
    return QGuiApplication(sys.argv)


def _joint_from_rig(rig: RigModel, name: str):
    return next(joint for joint in rig.joints if joint.name == name)


def _bone_from_skeleton(skeleton: SkeletonModel, name: str) -> BoneModel:
    return next(bone for bone in skeleton.bones if bone.name == name)


def _controller(document: DocumentModel, identifier: str):
    return next(controller for controller in document.autoposing.controllers if controller.identifier == identifier)


def _effector(solved_pose: SolvedPoseModel, identifier: str):
    return next(effector for effector in solved_pose.effectors if effector.controller_id == identifier)


def _retarget_joint(retarget_pose: RetargetPoseModel, name: str):
    return next(joint for joint in retarget_pose.joints if joint.name == name)


def _quaternion_angle_degrees(left: tuple[float, float, float, float], right: tuple[float, float, float, float]) -> float:
    dot = abs(sum(left[index] * right[index] for index in range(4)))
    dot = max(-1.0, min(1.0, dot))
    return math.degrees(2.0 * math.acos(dot))


def _instantiate_runtime_component(generated_component_path: Path | None) -> tuple[QQmlEngine | None, QQmlComponent | None, object | None]:
    if generated_component_path is None or not generated_component_path.exists():
        return None, None, None

    engine = QQmlEngine()
    component = QQmlComponent(engine, QUrl.fromLocalFile(str(generated_component_path)))
    if component.isError():
        errors = "\n".join(error.toString() for error in component.errors())
        raise RuntimeError(f"Could not load runtime component:\n{errors}")

    runtime_root = component.create()
    if runtime_root is None:
        errors = "\n".join(error.toString() for error in component.errors())
        raise RuntimeError(f"Could not create runtime component:\n{errors}")
    return engine, component, runtime_root


def _fresh_context() -> CheckContext:
    document = DocumentManager().open_startup_document()
    service = AutoPosingService()
    retargeter = RuntimeRetargeter()
    writer = RuntimePoseWriter()
    engine, component, runtime_root = _instantiate_runtime_component(document.generated_component_path)
    if runtime_root is not None:
        writer.bind_runtime_model(runtime_root)
        writer.apply_retarget_pose(document.retarget_pose)
    return CheckContext(
        document=document,
        service=service,
        retargeter=retargeter,
        writer=writer,
        engine=engine,
        component=component,
        runtime_root=runtime_root,
    )


def _solve_context(context: CheckContext) -> SkeletonModel | None:
    document = context.document
    document.solved_pose = context.service.solve_pose(document.autoposing, document.skeleton)
    document.solver_rig = document.solved_pose.rig
    document.retarget_pose = context.retargeter.retarget(
        document.autoposing,
        document.solved_pose,
        document.skeleton,
    )
    context.writer.apply_retarget_pose(document.retarget_pose)
    return context.writer.capture_runtime_skeleton(document.skeleton)


def _chain_length_error(rest: SkeletonModel, solved: RigModel) -> float:
    max_error = 0.0
    for parent_name, child_name in _CHECKED_LIMB_CHAINS:
        rest_parent = _bone_from_skeleton(rest, parent_name)
        rest_child = _bone_from_skeleton(rest, child_name)
        solved_parent = _joint_from_rig(solved, parent_name)
        solved_child = _joint_from_rig(solved, child_name)
        rest_length = distance(rest_parent.rest_world_position, rest_child.rest_world_position)
        solved_length = distance(solved_parent.world_position, solved_child.world_position)
        max_error = max(max_error, abs(rest_length - solved_length))
    return max_error


def _runtime_alignment_error(runtime_skeleton: SkeletonModel | None, solved_rig: RigModel) -> float | None:
    if runtime_skeleton is None:
        return None

    runtime_by_name = {bone.name: bone for bone in runtime_skeleton.bones}
    max_error = 0.0
    for joint_name in _MAIN_RUNTIME_BONES:
        if joint_name not in runtime_by_name:
            continue
        solved_joint = _joint_from_rig(solved_rig, joint_name)
        runtime_bone = runtime_by_name[joint_name]
        max_error = max(max_error, distance(solved_joint.world_position, runtime_bone.world_position))
    return max_error


def _run_near_wrist_check() -> CheckResult:
    context = _fresh_context()
    controller = _controller(context.document, "wrist_l_main")
    start = controller.target.world_position
    context.service.move_controller(
        context.document.autoposing,
        "wrist_l_main",
        (start[0] - 12.0, start[1] + 10.0, start[2] + 8.0),
    )
    runtime_skeleton = _solve_context(context)
    effector = _effector(context.document.solved_pose, "wrist_l_main")
    solved_hand = _joint_from_rig(context.document.solver_rig, "hand_l")
    hand_motion = distance(_bone_from_skeleton(context.document.skeleton, "hand_l").rest_world_position, solved_hand.world_position)
    length_error = _chain_length_error(context.document.skeleton, context.document.solver_rig)
    runtime_error = _runtime_alignment_error(runtime_skeleton, context.document.solver_rig)
    runtime_ok = runtime_error is None or runtime_error <= _MAX_RUNTIME_ALIGNMENT_ERROR
    passed = (not effector.clamped) and hand_motion > 5.0 and length_error <= 0.0015 and runtime_ok
    details = (
        f"clamped={effector.clamped}, hand_motion={hand_motion:.3f}, "
        f"length_error={length_error:.6f}, runtime_error={'n/a' if runtime_error is None else f'{runtime_error:.3f}'}"
    )
    return CheckResult("near wrist reach", passed, details)


def _run_far_wrist_check() -> CheckResult:
    context = _fresh_context()
    controller = _controller(context.document, "wrist_l_main")
    start = controller.target.world_position
    target = (start[0] - 140.0, start[1] + 90.0, start[2] + 120.0)
    context.service.move_controller(context.document.autoposing, "wrist_l_main", target)
    runtime_skeleton = _solve_context(context)
    effector = _effector(context.document.solved_pose, "wrist_l_main")
    pelvis_delta = distance(
        _bone_from_skeleton(context.document.skeleton, "pelvis").rest_world_position,
        _joint_from_rig(context.document.solver_rig, "pelvis").world_position,
    )
    clamp_gap = distance(effector.target_world_position, effector.solved_world_position)
    length_error = _chain_length_error(context.document.skeleton, context.document.solver_rig)
    runtime_error = _runtime_alignment_error(runtime_skeleton, context.document.solver_rig)
    runtime_ok = runtime_error is None or runtime_error <= _MAX_RUNTIME_ALIGNMENT_ERROR
    passed = effector.clamped and clamp_gap > 5.0 and pelvis_delta > 1.0 and length_error <= 0.0015 and runtime_ok
    details = (
        f"clamped={effector.clamped}, clamp_gap={clamp_gap:.3f}, pelvis_delta={pelvis_delta:.3f}, "
        f"length_error={length_error:.6f}, runtime_error={'n/a' if runtime_error is None else f'{runtime_error:.3f}'}"
    )
    return CheckResult("far wrist clamp", passed, details)


def _run_far_ankle_check() -> CheckResult:
    context = _fresh_context()
    controller = _controller(context.document, "ankle_l_main")
    start = controller.target.world_position
    target = (start[0] + 120.0, start[1] + 200.0, start[2] + 160.0)
    context.service.move_controller(context.document.autoposing, "ankle_l_main", target)
    runtime_skeleton = _solve_context(context)
    effector = _effector(context.document.solved_pose, "ankle_l_main")
    pelvis_delta = distance(
        _bone_from_skeleton(context.document.skeleton, "pelvis").rest_world_position,
        _joint_from_rig(context.document.solver_rig, "pelvis").world_position,
    )
    clamp_gap = distance(effector.target_world_position, effector.solved_world_position)
    length_error = _chain_length_error(context.document.skeleton, context.document.solver_rig)
    runtime_error = _runtime_alignment_error(runtime_skeleton, context.document.solver_rig)
    runtime_ok = runtime_error is None or runtime_error <= _MAX_RUNTIME_ALIGNMENT_ERROR
    passed = effector.clamped and clamp_gap > 5.0 and pelvis_delta > 1.0 and length_error <= 0.0015 and runtime_ok
    details = (
        f"clamped={effector.clamped}, clamp_gap={clamp_gap:.3f}, pelvis_delta={pelvis_delta:.3f}, "
        f"length_error={length_error:.6f}, runtime_error={'n/a' if runtime_error is None else f'{runtime_error:.3f}'}"
    )
    return CheckResult("far ankle clamp", passed, details)


def _run_direction_check() -> CheckResult:
    context = _fresh_context()
    chest_controller = _controller(context.document, "chest_dir")
    chest_start = chest_controller.target.world_position
    context.service.move_controller(
        context.document.autoposing,
        "chest_dir",
        (chest_start[0] + 70.0, chest_start[1] + 30.0, chest_start[2] + 45.0),
    )
    head_controller = _controller(context.document, "head_dir")
    head_start = head_controller.target.world_position
    context.service.move_controller(
        context.document.autoposing,
        "head_dir",
        (head_start[0] + 40.0, head_start[1] + 20.0, head_start[2] + 60.0),
    )
    _solve_context(context)
    spine_pose = _retarget_joint(context.document.retarget_pose, "spine_05")
    head_pose = _retarget_joint(context.document.retarget_pose, "head")
    rest_spine = _bone_from_skeleton(context.document.skeleton, "spine_05")
    rest_head = _bone_from_skeleton(context.document.skeleton, "head")
    spine_angle = _quaternion_angle_degrees(spine_pose.local_rotation, rest_spine.rest_local_rotation)
    head_angle = _quaternion_angle_degrees(head_pose.local_rotation, rest_head.rest_local_rotation)
    passed = spine_angle > 5.0 and head_angle > 3.0
    details = f"spine_angle={spine_angle:.2f} deg, head_angle={head_angle:.2f} deg"
    return CheckResult("direction controller rotation", passed, details)


def _run_twist_distribution_check() -> CheckResult:
    context = _fresh_context()
    controller = _controller(context.document, "wrist_l_main")
    start = controller.target.world_position
    context.service.move_controller(
        context.document.autoposing,
        "wrist_l_main",
        (start[0] - 30.0, start[1] + 80.0, start[2] + 40.0),
    )
    _solve_context(context)
    upper_twist_01 = _retarget_joint(context.document.retarget_pose, "upperarm_twist_01_l")
    upper_twist_02 = _retarget_joint(context.document.retarget_pose, "upperarm_twist_02_l")
    lower_twist_01 = _retarget_joint(context.document.retarget_pose, "lowerarm_twist_01_l")
    upper_rest_01 = _bone_from_skeleton(context.document.skeleton, "upperarm_twist_01_l")
    upper_rest_02 = _bone_from_skeleton(context.document.skeleton, "upperarm_twist_02_l")
    lower_rest_01 = _bone_from_skeleton(context.document.skeleton, "lowerarm_twist_01_l")
    angle_01 = _quaternion_angle_degrees(upper_twist_01.local_rotation, upper_rest_01.rest_local_rotation)
    angle_02 = _quaternion_angle_degrees(upper_twist_02.local_rotation, upper_rest_02.rest_local_rotation)
    angle_03 = _quaternion_angle_degrees(lower_twist_01.local_rotation, lower_rest_01.rest_local_rotation)
    passed = angle_01 > 1.0 and angle_02 > 1.0 and angle_03 > 1.0
    details = f"upper01={angle_01:.2f} deg, upper02={angle_02:.2f} deg, lower01={angle_03:.2f} deg"
    return CheckResult("twist distribution", passed, details)


def main() -> int:
    _ensure_app()
    checks = [
        _run_near_wrist_check(),
        _run_far_wrist_check(),
        _run_far_ankle_check(),
        _run_direction_check(),
        _run_twist_distribution_check(),
    ]

    print("MCS AutoPosing self-check")
    print("-" * 72)
    failures = 0
    for result in checks:
        status = "PASS" if result.passed else "FAIL"
        print(f"[{status}] {result.name}: {result.details}")
        if not result.passed:
            failures += 1

    if failures:
        print("-" * 72)
        print(f"Self-check finished with {failures} failing scenario(s).")
        return 1

    print("-" * 72)
    print("Self-check finished successfully.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
