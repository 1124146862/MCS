import math
import sys
from dataclasses import dataclass
from pathlib import Path

from PySide6.QtCore import QUrl
from PySide6.QtGui import QGuiApplication
from PySide6.QtQml import QQmlComponent, QQmlEngine

from app.autoposing.anatomy_profile import build_humanoid_anatomy_profile
from app.autoposing.constraints import extract_pose_constraints
from app.autoposing.controller_layout import direction_rod_length
from app.autoposing.pose_prior import PosePriorEstimator
from app.autoposing.preview_pipeline import AutoPosingPreviewPipeline
from app.autoposing.runtime_retargeter import RuntimeRetargeter
from app.autoposing.service import AutoPosingService
from app.autoposing.solver_adapter import SyncAutoPosingSolverAdapter
from app.autoposing.support_solver import SupportSolver
from app.document.document_manager import DocumentManager
from app.document.models import DocumentModel
from core.autoposing.models import RetargetPoseModel, SolvedPoseModel
from core.math import distance
from core.rig.models import RigModel
from core.skeleton.models import BoneModel, SkeletonModel
from infra.runtime.runtime_pose_writer import RuntimePoseWriter
from ui.viewmodels.document_view_model import DocumentViewModel


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
    adapter: SyncAutoPosingSolverAdapter
    pipeline: AutoPosingPreviewPipeline
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


def _controller_data(controller_rows: list[dict[str, object]], identifier: str) -> dict[str, object]:
    return next(row for row in controller_rows if row["id"] == identifier)


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
    adapter = SyncAutoPosingSolverAdapter(service, retargeter)
    writer = RuntimePoseWriter()
    pipeline = AutoPosingPreviewPipeline(document, service, adapter, writer)
    pipeline.prime(document.preview_frame)
    engine, component, runtime_root = _instantiate_runtime_component(document.generated_component_path)
    if runtime_root is not None:
        pipeline.bind_runtime_model(runtime_root)
        document.preview_frame = pipeline.consume_latest_frame()
        document.posed_skeleton = document.preview_frame.posed_skeleton
    return CheckContext(
        document=document,
        service=service,
        retargeter=retargeter,
        adapter=adapter,
        pipeline=pipeline,
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


def _run_preview_pipeline_revision_check() -> CheckResult:
    context = _fresh_context()
    controller = _controller(context.document, "wrist_l_main")
    start = controller.target.world_position
    target_one = (start[0] - 8.0, start[1] + 6.0, start[2] + 5.0)
    target_two = (start[0] - 18.0, start[1] + 14.0, start[2] + 9.0)

    context.pipeline.begin_drag("wrist_l_main")
    revision_one = context.pipeline.next_revision()
    context.pipeline.update_target("wrist_l_main", target_one, revision_one)
    revision_two = context.pipeline.next_revision()
    context.pipeline.update_target("wrist_l_main", target_two, revision_two)
    frame = context.pipeline.request_preview()
    context.pipeline.end_drag("wrist_l_main")

    latest_frame = context.pipeline.consume_latest_frame()
    target_ok = _controller(context.document, "wrist_l_main").target.world_position == target_two
    passed = (
        frame is not None
        and frame.revision == revision_two
        and latest_frame.revision == revision_two
        and target_ok
    )
    details = (
        f"frame_revision={'n/a' if frame is None else frame.revision}, latest_revision={latest_frame.revision}, "
        f"final_target={context.document.autoposing.controllers[3].target.world_position}"
    )
    return CheckResult("preview pipeline revision", passed, details)


def _run_viewmodel_preview_source_check() -> CheckResult:
    context = _fresh_context()
    view_model = DocumentViewModel(context.document)
    if context.runtime_root is not None:
        view_model.bindRuntimeModel(context.runtime_root)

    controller = _controller(context.document, "wrist_l_main")
    start = controller.target.world_position
    preview_target = (start[0] - 16.0, start[1] + 12.0, start[2] + 7.0)

    view_model.beginAutoPosingDrag("wrist_l_main")
    view_model.previewAutoPosingController("wrist_l_main", *preview_target)
    view_model._flush_preview_request()
    solved_before_tamper = _effector(context.document.preview_frame.solved_pose, "wrist_l_main").solved_world_position
    controller.target.solved_world_position = (999.0, 999.0, 999.0)
    controller_rows = view_model._autoposing_controller_visuals_data()
    controller_row = _controller_data(controller_rows, "wrist_l_main")
    display_tuple = (
        float(controller_row["displayX"]),
        float(controller_row["displayY"]),
        float(controller_row["displayZ"]),
    )

    view_model.endAutoPosingDrag("wrist_l_main")
    committed_revision = context.document.preview_frame.revision
    committed_status = context.document.status_message

    passed = (
        display_tuple == solved_before_tamper
        and committed_revision >= 1
        and "AutoPosing moved: wrist_l_main" in committed_status
    )
    details = (
        f"display={display_tuple}, solved_before_tamper={solved_before_tamper}, "
        f"committed_revision={committed_revision}, status={committed_status}"
    )
    return CheckResult("viewmodel preview frame authority", passed, details)


def _run_viewmodel_drag_signal_split_check() -> CheckResult:
    context = _fresh_context()
    view_model = DocumentViewModel(context.document)

    handle_emits: list[int] = []
    preview_emits: list[int] = []
    autoposing_emits: list[int] = []
    view_model.autoPosingControllerHandlesChanged.connect(lambda: handle_emits.append(1))
    view_model.viewportPreviewChanged.connect(lambda: preview_emits.append(1))
    view_model.autoposingChanged.connect(lambda: autoposing_emits.append(1))

    controller = _controller(context.document, "wrist_l_main")
    start = controller.target.world_position
    target_one = (start[0] - 10.0, start[1] + 7.0, start[2] + 4.0)
    target_two = (start[0] - 22.0, start[1] + 15.0, start[2] + 11.0)

    view_model.beginAutoPosingDrag("wrist_l_main")
    after_begin_handles = len(handle_emits)
    after_begin_preview = len(preview_emits)

    view_model.previewAutoPosingController("wrist_l_main", *target_one)
    view_model.previewAutoPosingController("wrist_l_main", *target_two)
    after_preview_handles = len(handle_emits)
    after_preview_preview = len(preview_emits)
    target_ok = controller.target.world_position == target_two

    view_model._flush_preview_request()
    after_flush_handles = len(handle_emits)
    after_flush_preview = len(preview_emits)
    flushed_revision = context.document.preview_frame.revision
    flushed_effector = _effector(context.document.preview_frame.solved_pose, "wrist_l_main")
    visual_row = _controller_data(view_model._autoposing_controller_visuals_data(), "wrist_l_main")
    visual_ok = (
        float(visual_row["displayX"]),
        float(visual_row["displayY"]),
        float(visual_row["displayZ"]),
    ) == flushed_effector.solved_world_position
    normal_skeleton_hidden = len(view_model._autoposing_preview_joints_data()) == 0 and len(view_model._autoposing_preview_bones_data()) == 0
    handles_before_debug = len(handle_emits)
    view_model.setAutoPosingDebugVisible(True)
    debug_handle_emit = len(handle_emits) - handles_before_debug == 1
    debug_skeleton_visible = len(view_model._autoposing_preview_joints_data()) > 0 and len(view_model._autoposing_preview_bones_data()) > 0

    handles_before_end = len(handle_emits)
    view_model.endAutoPosingDrag("wrist_l_main")
    end_handle_emit = len(handle_emits) - handles_before_end == 1
    handle_row = _controller_data(view_model._autoposing_controller_handles_data(), "wrist_l_main")
    final_effector = _effector(context.document.preview_frame.solved_pose, "wrist_l_main")
    handle_ok = (
        float(handle_row["handleX"]),
        float(handle_row["handleY"]),
        float(handle_row["handleZ"]),
    ) == final_effector.solved_world_position

    passed = (
        view_model._preview_request_timer.interval() == 16
        and after_begin_handles == 0
        and after_begin_preview == 0
        and after_preview_handles == 0
        and after_preview_preview == 0
        and target_ok
        and after_flush_handles == 0
        and after_flush_preview == 1
        and flushed_revision >= 2
        and visual_ok
        and normal_skeleton_hidden
        and debug_skeleton_visible
        and handles_before_debug == 0
        and debug_handle_emit
        and end_handle_emit
        and handle_ok
        and len(autoposing_emits) >= 2
    )
    details = (
        f"handles={len(handle_emits)}, preview={len(preview_emits)}, "
        f"revision={context.document.preview_frame.revision}, target_ok={target_ok}, "
        f"visual_ok={visual_ok}, handle_ok={handle_ok}, "
        f"normal_skeleton_hidden={normal_skeleton_hidden}, debug_skeleton_visible={debug_skeleton_visible}, "
        f"debug_handle_emit={debug_handle_emit}, end_handle_emit={end_handle_emit}"
    )
    return CheckResult("viewmodel drag signal split", passed, details)


def _run_normal_tether_filter_check() -> CheckResult:
    context = _fresh_context()
    view_model = DocumentViewModel(context.document)
    controller = _controller(context.document, "wrist_l_main")
    start = controller.target.world_position
    target = (start[0] - 95.0, start[1] + 45.0, start[2] + 70.0)

    view_model.beginAutoPosingDrag("wrist_l_main")
    view_model.previewAutoPosingController("wrist_l_main", *target)
    view_model._flush_preview_request()

    controllers_by_id = {controller.identifier: controller for controller in context.document.autoposing.controllers}
    tether_rows = view_model._autoposing_pressure_tethers_data()
    inactive_tethers = [
        row["startId"]
        for row in tether_rows
        if not (
            controllers_by_id[row["startId"]].active
            or controllers_by_id[row["startId"]].fixed
            or controllers_by_id[row["startId"]].selected
            or controllers_by_id[row["startId"]].always_active
        )
    ]

    view_model.setAutoPosingDebugVisible(True)
    debug_tether_count = len(view_model._autoposing_pressure_tethers_data())
    view_model.endAutoPosingDrag("wrist_l_main")

    passed = not inactive_tethers and debug_tether_count >= len(tether_rows)
    details = (
        f"normal_tethers={[row['startId'] for row in tether_rows]}, "
        f"inactive_tethers={inactive_tethers}, debug_tether_count={debug_tether_count}"
    )
    return CheckResult("normal tether filter", passed, details)


def _run_constraint_extraction_check() -> CheckResult:
    context = _fresh_context()
    document = context.document
    wrist = _controller(document, "wrist_l_main")
    pelvis = _controller(document, "pelvis_main")
    ankle = _controller(document, "ankle_l_main")
    wrist.target.world_position = (
        wrist.target.world_position[0] - 18.0,
        wrist.target.world_position[1] + 10.0,
        wrist.target.world_position[2] + 6.0,
    )
    wrist.active = True
    pelvis.fixed = True
    anatomy = build_humanoid_anatomy_profile(document.skeleton)
    constraints = extract_pose_constraints(document.autoposing, anatomy)

    passed = (
        "wrist_l_main" in constraints.positions
        and "pelvis_main" in constraints.positions
        and "ankle_l_main" in constraints.positions
        and "ankle_l_main" in constraints.supports
        and constraints.positions["wrist_l_main"].strength > 0.0
        and constraints.positions["pelvis_main"].locked
        and constraints.positions["pelvis_main"].priority == 100
        and constraints.supports["ankle_l_main"].locked
        and "wrist_r_main" not in constraints.positions
    )
    details = (
        f"positions={sorted(constraints.positions)}, supports={sorted(constraints.supports)}, "
        f"ankle_always_active={ankle.always_active}"
    )
    return CheckResult("constraint extraction", passed, details)


def _run_constraint_influence_check() -> CheckResult:
    context = _fresh_context()
    document = context.document
    wrist_l = _controller(document, "wrist_l_main")
    wrist_r = _controller(document, "wrist_r_main")
    elbow = _controller(document, "elbow_l_secondary")
    context.service.move_controller(
        document.autoposing,
        "wrist_l_main",
        (wrist_l.target.world_position[0] - 30.0, wrist_l.target.world_position[1] + 20.0, wrist_l.target.world_position[2]),
    )
    context.service.move_controller(
        document.autoposing,
        "wrist_r_main",
        (wrist_r.target.world_position[0] + 30.0, wrist_r.target.world_position[1] + 20.0, wrist_r.target.world_position[2]),
    )
    context.service.fix_controller(document.autoposing, "wrist_r_main")
    context.service.fix_controller(document.autoposing, "elbow_l_secondary", elbow.target.world_position)
    anatomy = build_humanoid_anatomy_profile(document.skeleton)
    constraints = extract_pose_constraints(document.autoposing, anatomy)
    active_wrist = constraints.positions["wrist_l_main"]
    fixed_wrist = constraints.positions["wrist_r_main"]
    fixed_elbow = constraints.positions["elbow_l_secondary"]
    ankle = constraints.positions["ankle_l_main"]
    passed = (
        active_wrist.pose_intent_weight >= 0.95
        and fixed_wrist.preservation_weight >= 0.99
        and fixed_wrist.pose_intent_weight < active_wrist.pose_intent_weight * 0.2
        and fixed_elbow.pose_intent_weight == 0.0
        and ankle.support_weight >= 0.99
        and ankle.pose_intent_weight <= 0.11
    )
    details = (
        f"active_wrist_intent={active_wrist.pose_intent_weight:.2f}, "
        f"fixed_wrist_intent={fixed_wrist.pose_intent_weight:.2f}, "
        f"fixed_elbow_intent={fixed_elbow.pose_intent_weight:.2f}, "
        f"ankle_support={ankle.support_weight:.2f}, ankle_intent={ankle.pose_intent_weight:.2f}"
    )
    return CheckResult("constraint influence semantics", passed, details)


def _run_anatomy_profile_check() -> CheckResult:
    context = _fresh_context()
    anatomy = build_humanoid_anatomy_profile(context.document.skeleton)
    required = ("pelvis", "spine_05", "head", "upperarm_l", "hand_l", "thigh_l", "foot_l", "foot_r")
    complete_required = all(anatomy.has(name) for name in required)
    chains_ok = all(
        anatomy.chain_available((chain.start_name, chain.mid_name, chain.end_name))
        for chain in anatomy.limb_chains.values()
    )
    support_ok = "foot_l" in anatomy.support_joint_names and "foot_r" in anatomy.support_joint_names
    frames_ok = (
        anatomy.body_height > 1.0
        and anatomy.torso_length > 0.0
        and anatomy.arm_length("l") > 0.0
        and anatomy.leg_length("l") > 0.0
        and anatomy.foot_length("l") > 0.0
    )
    passed = complete_required and chains_ok and support_ok and frames_ok and anatomy.length("thigh_l", "foot_l") > 0.0
    details = (
        f"required={complete_required}, chains={chains_ok}, supports={anatomy.support_joint_names}, "
        f"floor_y={anatomy.floor_y:.3f}, body_height={anatomy.body_height:.3f}, "
        f"arm_l={anatomy.arm_length('l'):.3f}, leg_l={anatomy.leg_length('l'):.3f}"
    )
    return CheckResult("anatomy profile", passed, details)


def _run_pose_prior_anatomy_check() -> CheckResult:
    context = _fresh_context()
    document = context.document
    anatomy = build_humanoid_anatomy_profile(document.skeleton)
    wrist = _controller(document, "wrist_l_main")
    wrist_start = wrist.target.world_position
    wrist.target.world_position = (wrist_start[0] - 70.0, wrist_start[1] + 30.0, wrist_start[2] + 40.0)
    wrist.active = True
    constraints = extract_pose_constraints(document.autoposing, anatomy)
    prior = PosePriorEstimator().estimate(document.skeleton, anatomy, constraints)
    prior_by_name = {joint.name: joint for joint in prior.joints}
    chest_delta = distance(prior_by_name["spine_05"].world_position, anatomy.rest("spine_05"))
    pelvis_delta = distance(prior_by_name["pelvis"].world_position, anatomy.rest("pelvis"))

    ankle = _controller(document, "ankle_l_main")
    ankle_start = ankle.target.world_position
    ankle.target.world_position = (ankle_start[0], ankle_start[1] - 100.0, ankle_start[2])
    constraints = extract_pose_constraints(document.autoposing, anatomy)
    prior = PosePriorEstimator().estimate(document.skeleton, anatomy, constraints)
    prior_by_name = {joint.name: joint for joint in prior.joints}
    foot_above_floor = prior_by_name["foot_l"].world_position[1] >= anatomy.floor_y
    pelvis_above_floor = prior_by_name["pelvis"].world_position[1] >= anatomy.floor_y

    passed = chest_delta > 1.0 and 0.1 < pelvis_delta < chest_delta and foot_above_floor and pelvis_above_floor
    details = (
        f"chest_delta={chest_delta:.3f}, pelvis_delta={pelvis_delta:.3f}, "
        f"foot_y={prior_by_name['foot_l'].world_position[1]:.3f}, floor_y={anatomy.floor_y:.3f}"
    )
    return CheckResult("pose prior anatomy", passed, details)


def _run_support_rule_check() -> CheckResult:
    context = _fresh_context()
    document = context.document
    anatomy = build_humanoid_anatomy_profile(document.skeleton)
    ankle = _controller(document, "ankle_l_main")
    ankle.target.world_position = (
        ankle.target.world_position[0],
        anatomy.floor_y - 50.0,
        ankle.target.world_position[2],
    )
    constraints = extract_pose_constraints(document.autoposing, anatomy)
    support_solver = SupportSolver()
    state = support_solver.state_from_constraints(anatomy, constraints)
    support_target = state.support_targets.get("foot_l")
    solved_pose = context.service.solve_pose(document.autoposing, document.skeleton)
    foot = _joint_from_rig(solved_pose.rig, "foot_l")
    ball = _joint_from_rig(solved_pose.rig, "ball_l")
    passed = (
        support_target is not None
        and support_target[1] >= anatomy.floor_y
        and foot.world_position[1] >= anatomy.floor_y
        and ball.world_position[1] >= anatomy.floor_y
    )
    details = (
        f"support_target_y={'n/a' if support_target is None else f'{support_target[1]:.3f}'}, "
        f"foot_y={foot.world_position[1]:.3f}, ball_y={ball.world_position[1]:.3f}, floor_y={anatomy.floor_y:.3f}"
    )
    return CheckResult("support rules", passed, details)


def _run_shoulder_girdle_check() -> CheckResult:
    context = _fresh_context()
    document = context.document
    anatomy = build_humanoid_anatomy_profile(document.skeleton)
    wrist = _controller(document, "wrist_l_main")
    start = wrist.target.world_position
    context.service.move_controller(
        document.autoposing,
        "wrist_l_main",
        (start[0] - 140.0, start[1] + 90.0, start[2] + 120.0),
    )
    _solve_context(context)
    clavicle_delta = distance(
        _bone_from_skeleton(document.skeleton, "clavicle_l").rest_world_position,
        _joint_from_rig(document.solver_rig, "clavicle_l").world_position,
    )
    chest_delta = distance(
        _bone_from_skeleton(document.skeleton, "spine_05").rest_world_position,
        _joint_from_rig(document.solver_rig, "spine_05").world_position,
    )
    pelvis_delta = distance(
        _bone_from_skeleton(document.skeleton, "pelvis").rest_world_position,
        _joint_from_rig(document.solver_rig, "pelvis").world_position,
    )
    head_delta = distance(
        _bone_from_skeleton(document.skeleton, "head").rest_world_position,
        _joint_from_rig(document.solver_rig, "head").world_position,
    )
    passed = (
        clavicle_delta > anatomy.arm_length("l") * 0.015
        and chest_delta > pelvis_delta * 0.5
        and head_delta < max(chest_delta * 1.5, anatomy.body_height * 0.12)
    )
    details = (
        f"clavicle_delta={clavicle_delta:.3f}, chest_delta={chest_delta:.3f}, "
        f"pelvis_delta={pelvis_delta:.3f}, head_delta={head_delta:.3f}, arm_l={anatomy.arm_length('l'):.3f}"
    )
    return CheckResult("shoulder girdle recruitment", passed, details)


def _run_support_stability_check() -> CheckResult:
    context = _fresh_context()
    document = context.document
    anatomy = build_humanoid_anatomy_profile(document.skeleton)
    right_foot_rest = _bone_from_skeleton(document.skeleton, "foot_r").rest_world_position
    right_ball_rest = _bone_from_skeleton(document.skeleton, "ball_r").rest_world_position
    ankle = _controller(document, "ankle_l_main")
    start = ankle.target.world_position
    context.service.move_controller(
        document.autoposing,
        "ankle_l_main",
        (start[0] + 90.0, start[1] + 120.0, start[2] + 70.0),
    )
    _solve_context(context)
    right_foot_delta = distance(right_foot_rest, _joint_from_rig(document.solver_rig, "foot_r").world_position)
    right_ball_delta = distance(right_ball_rest, _joint_from_rig(document.solver_rig, "ball_r").world_position)
    pelvis_delta = distance(
        _bone_from_skeleton(document.skeleton, "pelvis").rest_world_position,
        _joint_from_rig(document.solver_rig, "pelvis").world_position,
    )
    passed = (
        right_foot_delta < anatomy.body_height * 0.015
        and right_ball_delta < anatomy.body_height * 0.020
        and pelvis_delta > anatomy.body_height * 0.005
    )
    details = (
        f"right_foot_delta={right_foot_delta:.3f}, right_ball_delta={right_ball_delta:.3f}, "
        f"pelvis_delta={pelvis_delta:.3f}, body_height={anatomy.body_height:.3f}"
    )
    return CheckResult("support foot stability", passed, details)


def _run_visual_policy_check() -> CheckResult:
    context = _fresh_context()
    view_model = DocumentViewModel(context.document)
    normal_joints_hidden = len(view_model._autoposing_preview_joints_data()) == 0
    controller_rows = view_model._autoposing_controller_visuals_data()
    has_finger_controller = any("finger" in str(row["jointName"]).lower() for row in controller_rows)
    wrist_row = _controller_data(controller_rows, "wrist_l_main")
    ankle_row = _controller_data(controller_rows, "ankle_l_main")
    view_model.setAutoPosingDebugVisible(True)
    debug_joints_visible = len(view_model._autoposing_preview_joints_data()) > 0
    passed = (
        normal_joints_hidden
        and debug_joints_visible
        and not has_finger_controller
        and wrist_row["visualColor"] == "#3fae56"
        and ankle_row["locked"]
        and ankle_row["visualColor"] == "#2a98ff"
    )
    details = (
        f"normal_joints_hidden={normal_joints_hidden}, debug_joints_visible={debug_joints_visible}, "
        f"controllers={len(controller_rows)}, has_finger_controller={has_finger_controller}"
    )
    return CheckResult("visual policy", passed, details)


def _run_controller_schema_topology_check() -> CheckResult:
    context = _fresh_context()
    view_model = DocumentViewModel(context.document)
    controller_ids = {controller.identifier for controller in context.document.autoposing.controllers}
    edge_pairs = {
        (edge.start_controller_id, edge.end_controller_id)
        for edge in context.document.autoposing.controller_edges
    }
    required_controllers = {
        "pelvis_additional",
        "stomach_secondary",
        "chest_additional",
        "head_additional",
        "shoulder_l_lesser",
        "hand_l_secondary",
        "hand_l_additional",
        "foot_l_secondary",
        "ball_l_lesser",
        "foot_l_additional",
        "shoulder_r_lesser",
        "hand_r_secondary",
        "foot_r_secondary",
    }
    required_edges = {
        ("shoulder_l_lesser", "elbow_l_secondary"),
        ("elbow_l_secondary", "wrist_l_main"),
        ("wrist_l_main", "hand_l_secondary"),
        ("hand_l_secondary", "hand_l_additional"),
        ("hip_l_lesser", "knee_l_secondary"),
        ("knee_l_secondary", "ankle_l_main"),
        ("ankle_l_main", "foot_l_secondary"),
        ("foot_l_secondary", "ball_l_lesser"),
        ("pelvis_main", "stomach_secondary"),
        ("stomach_secondary", "chest_secondary"),
        ("chest_secondary", "chest_additional"),
    }
    topology_rows = view_model._autoposing_topology_edges_data()
    normal_skeleton_hidden = len(view_model._autoposing_preview_joints_data()) == 0
    passed = (
        required_controllers.issubset(controller_ids)
        and required_edges.issubset(edge_pairs)
        and len(topology_rows) >= len(required_edges)
        and normal_skeleton_hidden
        and all(float(row["restLength"]) > 0.001 for row in topology_rows)
    )
    missing_controllers = sorted(required_controllers - controller_ids)
    missing_edges = sorted(required_edges - edge_pairs)
    details = (
        f"controllers={len(controller_ids)}, edges={len(edge_pairs)}, topology_rows={len(topology_rows)}, "
        f"missing_controllers={missing_controllers}, missing_edges={missing_edges}, "
        f"normal_skeleton_hidden={normal_skeleton_hidden}"
    )
    return CheckResult("controller schema topology", passed, details)


def _run_schema_layout_offset_check() -> CheckResult:
    context = _fresh_context()
    view_model = DocumentViewModel(context.document)

    wrist = _controller(context.document, "wrist_l_main")
    wrist_start = wrist.target.world_position
    wrist_target = (wrist_start[0] - 32.0, wrist_start[1] + 16.0, wrist_start[2] + 12.0)
    view_model.beginAutoPosingDrag("wrist_l_main")
    view_model.previewAutoPosingController("wrist_l_main", *wrist_target)
    view_model.endAutoPosingDrag("wrist_l_main")
    hand_secondary = _controller(context.document, "hand_l_secondary")
    hand_additional = _controller(context.document, "hand_l_additional")
    hand_cluster_ok = (
        distance(wrist.target.world_position, hand_secondary.target.world_position) > 3.0
        and distance(hand_secondary.target.world_position, hand_additional.target.world_position) > 3.0
    )

    ankle = _controller(context.document, "ankle_l_main")
    ankle_start = ankle.target.world_position
    ankle_target = (ankle_start[0] + 18.0, ankle_start[1] + 10.0, ankle_start[2] + 20.0)
    view_model.beginAutoPosingDrag("ankle_l_main")
    view_model.previewAutoPosingController("ankle_l_main", *ankle_target)
    view_model.endAutoPosingDrag("ankle_l_main")
    foot_secondary = _controller(context.document, "foot_l_secondary")
    foot_additional = _controller(context.document, "foot_l_additional")
    foot_cluster_ok = (
        distance(ankle.target.world_position, foot_secondary.target.world_position) > 3.0
        and distance(foot_secondary.target.world_position, foot_additional.target.world_position) > 3.0
    )

    passed = hand_cluster_ok and foot_cluster_ok
    details = (
        f"hand_cluster_ok={hand_cluster_ok}, foot_cluster_ok={foot_cluster_ok}, "
        f"hand_wrist_gap={distance(wrist.target.world_position, hand_secondary.target.world_position):.3f}, "
        f"foot_ankle_gap={distance(ankle.target.world_position, foot_secondary.target.world_position):.3f}"
    )
    return CheckResult("schema layout offsets", passed, details)


def _run_semantic_constraint_extraction_check() -> CheckResult:
    context = _fresh_context()
    document = context.document
    anatomy = build_humanoid_anatomy_profile(document.skeleton)
    for controller_id in (
        "elbow_l_secondary",
        "hand_l_additional",
        "foot_l_additional",
        "ball_l_lesser",
        "chest_additional",
    ):
        controller = _controller(document, controller_id)
        start = controller.target.world_position
        context.service.move_controller(document.autoposing, controller_id, (start[0] + 8.0, start[1] + 5.0, start[2] + 6.0))

    constraints = extract_pose_constraints(document.autoposing, anatomy)
    passed = (
        "elbow_l_secondary" in constraints.bend_hints
        and "hand_l_additional" in constraints.orientation_hints
        and "foot_l_additional" in constraints.orientation_hints
        and "ball_l_lesser" in constraints.foot_contacts
        and "chest_additional" in constraints.orientation_hints
    )
    details = (
        f"bend={sorted(constraints.bend_hints)}, "
        f"orientation={sorted(constraints.orientation_hints)}, foot_contacts={sorted(constraints.foot_contacts)}"
    )
    return CheckResult("semantic constraint extraction", passed, details)


def _run_bend_hint_semantics_check() -> CheckResult:
    context = _fresh_context()
    document = context.document
    elbow = _controller(document, "elbow_l_secondary")
    start = elbow.target.world_position
    target = (start[0] - 50.0, start[1] + 24.0, start[2] + 28.0)
    context.service.move_controller(document.autoposing, "elbow_l_secondary", target)
    _solve_context(context)
    solved_elbow = _joint_from_rig(document.solver_rig, "lowerarm_l")
    elbow_motion = distance(solved_elbow.world_position, _bone_from_skeleton(document.skeleton, "lowerarm_l").rest_world_position)
    hint_error = distance(solved_elbow.world_position, target)
    length_error = _chain_length_error(document.skeleton, document.solver_rig)
    passed = elbow_motion > 4.0 and hint_error < 60.0 and length_error <= 0.0015
    details = f"elbow_motion={elbow_motion:.3f}, hint_error={hint_error:.3f}, length_error={length_error:.6f}"
    return CheckResult("bend hint semantics", passed, details)


def _run_terminal_orientation_retarget_check() -> CheckResult:
    context = _fresh_context()
    document = context.document
    hand = _controller(document, "hand_l_additional")
    hand_start = hand.target.world_position
    context.service.move_controller(document.autoposing, "hand_l_additional", (hand_start[0] - 42.0, hand_start[1] + 16.0, hand_start[2] + 34.0))
    foot = _controller(document, "foot_l_additional")
    foot_start = foot.target.world_position
    context.service.move_controller(document.autoposing, "foot_l_additional", (foot_start[0] + 24.0, foot_start[1] + 18.0, foot_start[2] + 28.0))
    _solve_context(context)

    hand_pose = _retarget_joint(document.retarget_pose, "hand_l")
    foot_pose = _retarget_joint(document.retarget_pose, "foot_l")
    ball_pose = _retarget_joint(document.retarget_pose, "ball_l")
    hand_rest = _bone_from_skeleton(document.skeleton, "hand_l")
    foot_rest = _bone_from_skeleton(document.skeleton, "foot_l")
    ball_rest = _bone_from_skeleton(document.skeleton, "ball_l")
    hand_angle = _quaternion_angle_degrees(hand_pose.local_rotation, hand_rest.rest_local_rotation)
    foot_angle = _quaternion_angle_degrees(foot_pose.local_rotation, foot_rest.rest_local_rotation)
    ball_angle = _quaternion_angle_degrees(ball_pose.local_rotation, ball_rest.rest_local_rotation)
    solved_wrist = _joint_from_rig(document.solver_rig, "hand_l")
    wrist_drift = distance(solved_wrist.world_position, _bone_from_skeleton(document.skeleton, "hand_l").rest_world_position)
    passed = hand_angle > 2.0 and foot_angle > 2.0 and ball_angle > 2.0 and wrist_drift < 5.0
    details = (
        f"hand_angle={hand_angle:.2f}, foot_angle={foot_angle:.2f}, "
        f"ball_angle={ball_angle:.2f}, wrist_drift={wrist_drift:.3f}"
    )
    return CheckResult("terminal orientation retarget", passed, details)


def _run_body_orientation_hint_check() -> CheckResult:
    context = _fresh_context()
    document = context.document
    chest = _controller(document, "chest_additional")
    chest_start = chest.target.world_position
    context.service.move_controller(document.autoposing, "chest_additional", (chest_start[0] + 45.0, chest_start[1] + 10.0, chest_start[2] + 28.0))
    head = _controller(document, "head_additional")
    head_start = head.target.world_position
    context.service.move_controller(document.autoposing, "head_additional", (head_start[0] + 28.0, head_start[1] + 12.0, head_start[2] + 22.0))
    _solve_context(context)

    spine_pose = _retarget_joint(document.retarget_pose, "spine_05")
    head_pose = _retarget_joint(document.retarget_pose, "head")
    spine_rest = _bone_from_skeleton(document.skeleton, "spine_05")
    head_rest = _bone_from_skeleton(document.skeleton, "head")
    spine_angle = _quaternion_angle_degrees(spine_pose.local_rotation, spine_rest.rest_local_rotation)
    head_angle = _quaternion_angle_degrees(head_pose.local_rotation, head_rest.rest_local_rotation)
    passed = spine_angle > 2.0 and head_angle > 2.0
    details = f"spine_angle={spine_angle:.2f}, head_angle={head_angle:.2f}"
    return CheckResult("body orientation hint", passed, details)


def _run_direction_rods_check() -> CheckResult:
    context = _fresh_context()
    view_model = DocumentViewModel(context.document)
    rods = view_model._autoposing_direction_rods_data()
    initial_ok = all(
        abs(
            distance(
                (float(row["startX"]), float(row["startY"]), float(row["startZ"])),
                (float(row["endX"]), float(row["endY"]), float(row["endZ"])),
            )
            - direction_rod_length(str(row["id"]))
        )
        <= 0.001
        for row in rods
    )

    chest = _controller(context.document, "chest_dir")
    start = chest.target.world_position
    context.service.move_controller(context.document.autoposing, "chest_dir", (start[0] + 120.0, start[1] + 40.0, start[2] + 90.0))
    frame = context.adapter.solve_frame(context.document.autoposing, context.document.skeleton, revision=1)
    view_model._commit_preview_frame(frame)
    chest_rod = _controller_data(view_model._autoposing_direction_rods_data(), "chest_dir")
    chest_length = distance(
        (float(chest_rod["startX"]), float(chest_rod["startY"]), float(chest_rod["startZ"])),
        (float(chest_rod["endX"]), float(chest_rod["endY"]), float(chest_rod["endZ"])),
    )
    passed = initial_ok and abs(chest_length - 30.0) <= 0.001
    details = f"initial_ok={initial_ok}, chest_length={chest_length:.3f}"
    return CheckResult("direction rods", passed, details)


def _run_controller_menu_state_check() -> CheckResult:
    context = _fresh_context()
    view_model = DocumentViewModel(context.document)
    wrist = view_model.autoPosingControllerMenuState("wrist_l_main")
    ankle = view_model.autoPosingControllerMenuState("ankle_l_main")
    direction = view_model.autoPosingControllerMenuState("chest_dir")
    passed = (
        wrist["menuKind"] == "controller"
        and wrist["canLock"]
        and not wrist["canUnlock"]
        and ankle["menuKind"] == "alwaysActive"
        and ankle["locked"]
        and not ankle["canLock"]
        and not ankle["canUnlock"]
        and direction["menuKind"] == "direction"
        and direction["isDirection"]
    )
    details = f"wrist={wrist}, ankle={ankle}, direction={direction}"
    return CheckResult("controller menu state", passed, details)


def _run_lock_fix_layout_sync_check() -> CheckResult:
    context = _fresh_context()
    view_model = DocumentViewModel(context.document)

    view_model.lockAutoPosingController("wrist_l_main")
    wrist = _controller(context.document, "wrist_l_main")
    locked_ok = wrist.active and _controller_data(view_model._autoposing_controller_visuals_data(), "wrist_l_main")["locked"]
    view_model.unlockAutoPosingController("wrist_l_main")
    unlocked_ok = not wrist.active and not _controller_data(view_model._autoposing_controller_visuals_data(), "wrist_l_main")["locked"]

    view_model.fixAutoPosingController("elbow_l_secondary")
    elbow = _controller(context.document, "elbow_l_secondary")
    fixed_ok = elbow.fixed and _controller_data(view_model._autoposing_controller_visuals_data(), "elbow_l_secondary")["fixed"]
    view_model.unfixAutoPosingController("elbow_l_secondary")
    elbow_effector_after_unfix = _effector(context.document.preview_frame.solved_pose, "elbow_l_secondary")
    unfixed_ok = not elbow.fixed and distance(elbow.target.world_position, elbow_effector_after_unfix.solved_world_position) < 0.001

    wrist_start = wrist.target.world_position
    view_model.beginAutoPosingDrag("wrist_l_main")
    view_model.previewAutoPosingController("wrist_l_main", wrist_start[0] - 35.0, wrist_start[1] + 18.0, wrist_start[2] + 12.0)
    view_model.endAutoPosingDrag("wrist_l_main")
    elbow_effector = _effector(context.document.preview_frame.solved_pose, "elbow_l_secondary")
    layout_ok = distance(elbow.target.world_position, elbow_effector.solved_world_position) < 0.001
    target_preserved = distance(wrist.target.world_position, (wrist_start[0] - 35.0, wrist_start[1] + 18.0, wrist_start[2] + 12.0)) < 0.001

    passed = locked_ok and unlocked_ok and fixed_ok and unfixed_ok and layout_ok and target_preserved
    details = (
        f"locked_ok={locked_ok}, unlocked_ok={unlocked_ok}, fixed_ok={fixed_ok}, "
        f"unfixed_ok={unfixed_ok}, layout_ok={layout_ok}, target_preserved={target_preserved}"
    )
    return CheckResult("lock fix layout sync", passed, details)


def _run_finger_schema_mode_check() -> CheckResult:
    context = _fresh_context()
    document = context.document
    view_model = DocumentViewModel(document)
    finger_controllers = [controller for controller in document.autoposing.controllers if controller.role.startswith("finger")]
    default_mode = view_model._autoposing_display_mode_value() == "normal"
    default_hidden = not any(row["role"] in {"finger_tip", "finger_pre_tip"} for row in view_model._autoposing_controller_visuals_data())
    default_debug_hidden = len(view_model._autoposing_preview_joints_data()) == 0

    view_model.setAutoPosingDisplayMode("fingers")
    visible_rows = [row for row in view_model._autoposing_controller_visuals_data() if row["role"] in {"finger_tip", "finger_pre_tip"}]
    topology_rows = view_model._autoposing_topology_edges_data()
    index_l = _controller(document, "index_l_tip")
    index_r = _controller(document, "index_r_tip")
    finger_menu = view_model.autoPosingControllerMenuState("index_l_tip")
    enabled_ok = (
        view_model._autoposing_display_mode_value() == "fingers"
        and len(finger_controllers) == 20
        and len(visible_rows) == 20
        and index_l.active
        and index_r.active
        and any(row["endId"] == "index_l_tip" for row in topology_rows)
        and finger_menu["menuKind"] == "finger"
    )

    view_model.setAutoPosingDisplayMode("debug")
    debug_rows = [row for row in view_model._autoposing_controller_visuals_data() if row["role"] in {"finger_tip", "finger_pre_tip"}]
    debug_ok = (
        view_model._autoposing_display_mode_value() == "debug"
        and len(debug_rows) == 20
        and len(view_model._autoposing_preview_joints_data()) > 0
        and len(view_model._autoposing_preview_bones_data()) > 0
    )

    view_model.setAutoPosingDisplayMode("normal")
    anatomy = build_humanoid_anatomy_profile(document.skeleton)
    constraints = extract_pose_constraints(document.autoposing, anatomy)
    disabled_hidden = not any(row["role"] in {"finger_tip", "finger_pre_tip"} for row in view_model._autoposing_controller_visuals_data())
    passed = default_mode and default_hidden and default_debug_hidden and enabled_ok and debug_ok and disabled_hidden and not constraints.finger_tips
    details = (
        f"finger_count={len(finger_controllers)}, default_mode={default_mode}, default_hidden={default_hidden}, "
        f"default_debug_hidden={default_debug_hidden}, enabled_ok={enabled_ok}, debug_ok={debug_ok}, "
        f"disabled_hidden={disabled_hidden}, disabled_tip_constraints={len(constraints.finger_tips)}"
    )
    return CheckResult("finger schema mode", passed, details)


def _run_finger_tip_retarget_check() -> CheckResult:
    context = _fresh_context()
    document = context.document
    context.service.set_fingers_enabled(document.autoposing, True)
    tip = _controller(document, "index_l_tip")
    start = tip.target.world_position
    context.service.move_controller(document.autoposing, "index_l_tip", (start[0] + 12.0, start[1] + 5.0, start[2] + 8.0))
    _solve_context(context)

    index_pose = _retarget_joint(document.retarget_pose, "index_01_l")
    index_rest = _bone_from_skeleton(document.skeleton, "index_01_l")
    hand = _joint_from_rig(document.solver_rig, "hand_l")
    hand_rest = _bone_from_skeleton(document.skeleton, "hand_l")
    effector = _effector(document.solved_pose, "index_l_tip")
    index_angle = _quaternion_angle_degrees(index_pose.local_rotation, index_rest.rest_local_rotation)
    hand_drift = distance(hand.world_position, hand_rest.rest_world_position)
    passed = index_angle > 0.5 and hand_drift < 0.001 and distance(effector.solved_world_position, start) > 1.0
    details = f"index_angle={index_angle:.3f}, hand_drift={hand_drift:.6f}, clamped={effector.clamped}"
    return CheckResult("finger tip retarget", passed, details)


def _run_finger_pre_tip_bend_check() -> CheckResult:
    context = _fresh_context()
    document = context.document
    context.service.set_fingers_enabled(document.autoposing, True)
    pre_tip = _controller(document, "index_l_pre_tip")
    start = pre_tip.target.world_position
    context.service.move_controller(document.autoposing, "index_l_pre_tip", (start[0] - 10.0, start[1] + 8.0, start[2] + 12.0))
    _solve_context(context)

    solved_mid = _joint_from_rig(document.solver_rig, "index_02_l")
    rest_mid = _bone_from_skeleton(document.skeleton, "index_02_l")
    index_pose = _retarget_joint(document.retarget_pose, "index_01_l")
    index_rest = _bone_from_skeleton(document.skeleton, "index_01_l")
    mid_motion = distance(solved_mid.world_position, rest_mid.rest_world_position)
    index_angle = _quaternion_angle_degrees(index_pose.local_rotation, index_rest.rest_local_rotation)
    passed = mid_motion > 0.5 and index_angle > 0.5
    details = f"mid_motion={mid_motion:.3f}, index_angle={index_angle:.3f}"
    return CheckResult("finger pre-tip bend", passed, details)


def _run_finger_palm_spread_check() -> CheckResult:
    pinky_context = _fresh_context()
    pinky_doc = pinky_context.document
    pinky_context.service.set_fingers_enabled(pinky_doc.autoposing, True)
    pinky = _controller(pinky_doc, "pinky_l_tip")
    pinky_start = pinky.target.world_position
    pinky_context.service.move_controller(
        pinky_doc.autoposing,
        "pinky_l_tip",
        (pinky_start[0] + 10.0, pinky_start[1] + 2.0, pinky_start[2] - 14.0),
    )
    _solve_context(pinky_context)
    pinky_hand_angle = _quaternion_angle_degrees(
        _retarget_joint(pinky_doc.retarget_pose, "hand_l").local_rotation,
        _bone_from_skeleton(pinky_doc.skeleton, "hand_l").rest_local_rotation,
    )

    middle_context = _fresh_context()
    middle_doc = middle_context.document
    middle_context.service.set_fingers_enabled(middle_doc.autoposing, True)
    middle = _controller(middle_doc, "middle_l_tip")
    middle_start = middle.target.world_position
    middle_context.service.move_controller(
        middle_doc.autoposing,
        "middle_l_tip",
        (middle_start[0] + 10.0, middle_start[1] + 2.0, middle_start[2] - 14.0),
    )
    _solve_context(middle_context)
    middle_hand_angle = _quaternion_angle_degrees(
        _retarget_joint(middle_doc.retarget_pose, "hand_l").local_rotation,
        _bone_from_skeleton(middle_doc.skeleton, "hand_l").rest_local_rotation,
    )

    passed = pinky_hand_angle > 0.1 and middle_hand_angle < 0.1
    details = f"pinky_hand_angle={pinky_hand_angle:.3f}, middle_hand_angle={middle_hand_angle:.3f}"
    return CheckResult("finger palm spread", passed, details)


def _run_thumb_opposition_check() -> CheckResult:
    context = _fresh_context()
    document = context.document
    context.service.set_fingers_enabled(document.autoposing, True)
    thumb = _controller(document, "thumb_l_tip")
    thumb_start = thumb.target.world_position
    context.service.move_controller(
        document.autoposing,
        "thumb_l_tip",
        (thumb_start[0] - 8.0, thumb_start[1] + 8.0, thumb_start[2] + 12.0),
    )
    pre_tip = _controller(document, "thumb_l_pre_tip")
    pre_start = pre_tip.target.world_position
    context.service.move_controller(
        document.autoposing,
        "thumb_l_pre_tip",
        (pre_start[0] - 5.0, pre_start[1] + 6.0, pre_start[2] + 10.0),
    )
    _solve_context(context)

    thumb_angle = _quaternion_angle_degrees(
        _retarget_joint(document.retarget_pose, "thumb_01_l").local_rotation,
        _bone_from_skeleton(document.skeleton, "thumb_01_l").rest_local_rotation,
    )
    index_angle = _quaternion_angle_degrees(
        _retarget_joint(document.retarget_pose, "index_01_l").local_rotation,
        _bone_from_skeleton(document.skeleton, "index_01_l").rest_local_rotation,
    )
    passed = thumb_angle > 0.5 and index_angle < 0.1
    details = f"thumb_angle={thumb_angle:.3f}, index_angle={index_angle:.3f}"
    return CheckResult("thumb opposition", passed, details)


def _run_hand_finger_priority_check() -> CheckResult:
    context = _fresh_context()
    document = context.document
    context.service.set_fingers_enabled(document.autoposing, True)
    hand = _controller(document, "hand_l_additional")
    hand_start = hand.target.world_position
    context.service.move_controller(
        document.autoposing,
        "hand_l_additional",
        (hand_start[0] - 35.0, hand_start[1] + 12.0, hand_start[2] + 24.0),
    )
    _solve_context(context)
    hand_only_angle = _quaternion_angle_degrees(
        _retarget_joint(document.retarget_pose, "hand_l").local_rotation,
        _bone_from_skeleton(document.skeleton, "hand_l").rest_local_rotation,
    )

    index = _controller(document, "index_l_tip")
    index_start = index.target.world_position
    context.service.move_controller(
        document.autoposing,
        "index_l_tip",
        (index_start[0] + 12.0, index_start[1] + 5.0, index_start[2] + 8.0),
    )
    _solve_context(context)
    combined_angle = _quaternion_angle_degrees(
        _retarget_joint(document.retarget_pose, "hand_l").local_rotation,
        _bone_from_skeleton(document.skeleton, "hand_l").rest_local_rotation,
    )
    passed = hand_only_angle > 2.0 and abs(combined_angle - hand_only_angle) < 2.0
    details = f"hand_only_angle={hand_only_angle:.3f}, combined_angle={combined_angle:.3f}"
    return CheckResult("hand finger priority", passed, details)


def _run_ball_roll_check() -> CheckResult:
    context = _fresh_context()
    document = context.document
    initial_solved = context.service.solve_pose(document.autoposing, document.skeleton)
    initial_foot = _joint_from_rig(initial_solved.rig, "foot_l").world_position
    ball = _controller(document, "ball_l_lesser")
    ball_start = ball.target.world_position
    context.service.move_controller(
        document.autoposing,
        "ball_l_lesser",
        (ball_start[0] + 90.0, ball_start[1] + 25.0, ball_start[2] + 80.0),
    )
    _solve_context(context)

    solved_foot = _joint_from_rig(document.solver_rig, "foot_l").world_position
    solved_ball = _joint_from_rig(document.solver_rig, "ball_l").world_position
    effector = _effector(document.solved_pose, "ball_l_lesser")
    foot_angle = _quaternion_angle_degrees(
        _retarget_joint(document.retarget_pose, "foot_l").local_rotation,
        _bone_from_skeleton(document.skeleton, "foot_l").rest_local_rotation,
    )
    ball_angle = _quaternion_angle_degrees(
        _retarget_joint(document.retarget_pose, "ball_l").local_rotation,
        _bone_from_skeleton(document.skeleton, "ball_l").rest_local_rotation,
    )
    anatomy = build_humanoid_anatomy_profile(document.skeleton)
    foot_drift = distance(initial_foot, solved_foot)
    passed = (
        effector.clamped
        and solved_ball[1] >= anatomy.floor_y
        and foot_drift < 0.001
        and foot_angle > 0.5
        and ball_angle > 0.5
    )
    details = (
        f"foot_drift={foot_drift:.6f}, ball_y={solved_ball[1]:.3f}, floor_y={anatomy.floor_y:.3f}, "
        f"foot_angle={foot_angle:.3f}, ball_angle={ball_angle:.3f}, clamped={effector.clamped}"
    )
    return CheckResult("ball roll", passed, details)


def _run_finger_reset_layout_check() -> CheckResult:
    context = _fresh_context()
    document = context.document
    view_model = DocumentViewModel(document)
    view_model.setAutoPosingDisplayMode("fingers")
    view_model.unlockAutoPosingController("index_l_tip")

    wrist = _controller(document, "wrist_l_main")
    wrist_start = wrist.target.world_position
    wrist_target = (wrist_start[0] - 30.0, wrist_start[1] + 18.0, wrist_start[2] + 12.0)
    view_model.beginAutoPosingDrag("wrist_l_main")
    view_model.previewAutoPosingController("wrist_l_main", *wrist_target)
    view_model.endAutoPosingDrag("wrist_l_main")

    index = _controller(document, "index_l_tip")
    original_default = index.target.default_position
    followed_target = index.target.world_position
    view_model.beginAutoPosingDrag("index_l_tip")
    view_model.previewAutoPosingController("index_l_tip", followed_target[0] + 15.0, followed_target[1] + 8.0, followed_target[2] + 12.0)
    view_model.endAutoPosingDrag("index_l_tip")
    view_model.resetAutoPosingController("index_l_tip")

    reset_effector = _effector(document.preview_frame.solved_pose, "index_l_tip")
    reset_target = index.target.world_position
    target_near_effector = distance(reset_target, reset_effector.solved_world_position) < 0.001
    did_not_jump_to_rest = distance(reset_target, original_default) > 1.0
    passed = target_near_effector and did_not_jump_to_rest and not index.active
    details = (
        f"target_near_effector={target_near_effector}, did_not_jump_to_rest={did_not_jump_to_rest}, "
        f"active={index.active}, rest_gap={distance(reset_target, original_default):.3f}"
    )
    return CheckResult("finger reset layout", passed, details)


def main() -> int:
    _ensure_app()
    checks = [
        _run_near_wrist_check(),
        _run_far_wrist_check(),
        _run_far_ankle_check(),
        _run_direction_check(),
        _run_twist_distribution_check(),
        _run_preview_pipeline_revision_check(),
        _run_viewmodel_preview_source_check(),
        _run_viewmodel_drag_signal_split_check(),
        _run_normal_tether_filter_check(),
        _run_constraint_extraction_check(),
        _run_constraint_influence_check(),
        _run_anatomy_profile_check(),
        _run_pose_prior_anatomy_check(),
        _run_support_rule_check(),
        _run_shoulder_girdle_check(),
        _run_support_stability_check(),
        _run_visual_policy_check(),
        _run_controller_schema_topology_check(),
        _run_schema_layout_offset_check(),
        _run_semantic_constraint_extraction_check(),
        _run_bend_hint_semantics_check(),
        _run_terminal_orientation_retarget_check(),
        _run_body_orientation_hint_check(),
        _run_direction_rods_check(),
        _run_controller_menu_state_check(),
        _run_lock_fix_layout_sync_check(),
        _run_finger_schema_mode_check(),
        _run_finger_tip_retarget_check(),
        _run_finger_pre_tip_bend_check(),
        _run_finger_palm_spread_check(),
        _run_thumb_opposition_check(),
        _run_hand_finger_priority_check(),
        _run_ball_roll_check(),
        _run_finger_reset_layout_check(),
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
