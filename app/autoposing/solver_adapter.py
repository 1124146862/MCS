from __future__ import annotations

from abc import ABC, abstractmethod
from time import perf_counter

from core.autoposing.models import PreviewFrameModel, SolvedPoseModel
from core.skeleton.models import BoneModel, SkeletonModel

from .runtime_retargeter import RuntimeRetargeter
from .service import AutoPosingService


class AutoPosingSolverAdapter(ABC):
    @abstractmethod
    def solve_frame(self, rig, skeleton: SkeletonModel, revision: int) -> PreviewFrameModel:
        raise NotImplementedError

    @abstractmethod
    def build_posed_skeleton(self, skeleton: SkeletonModel, solved_pose: SolvedPoseModel) -> SkeletonModel:
        raise NotImplementedError


class SyncAutoPosingSolverAdapter(AutoPosingSolverAdapter):
    def __init__(
        self,
        autoposing_service: AutoPosingService | None = None,
        runtime_retargeter: RuntimeRetargeter | None = None,
    ) -> None:
        self._autoposing_service = autoposing_service or AutoPosingService()
        self._runtime_retargeter = runtime_retargeter or RuntimeRetargeter()

    def solve_frame(self, rig, skeleton: SkeletonModel, revision: int) -> PreviewFrameModel:
        solve_started = perf_counter()
        solved_pose = self._autoposing_service.solve_pose(rig, skeleton)
        solve_elapsed_ms = (perf_counter() - solve_started) * 1000.0

        retarget_started = perf_counter()
        retarget_pose = self._runtime_retargeter.retarget(rig, solved_pose, skeleton)
        retarget_elapsed_ms = (perf_counter() - retarget_started) * 1000.0

        posed_skeleton = self.build_posed_skeleton(skeleton, solved_pose)
        return PreviewFrameModel(
            revision=revision,
            solved_pose=solved_pose,
            retarget_pose=retarget_pose,
            posed_skeleton=posed_skeleton,
            effectors=list(solved_pose.effectors),
            status_message=solved_pose.status_message,
            timings={
                "solveMs": solve_elapsed_ms,
                "retargetMs": retarget_elapsed_ms,
                "totalMs": solve_elapsed_ms + retarget_elapsed_ms,
            },
        )

    def build_posed_skeleton(self, skeleton: SkeletonModel, solved_pose: SolvedPoseModel) -> SkeletonModel:
        reference_by_name = {bone.name: bone for bone in skeleton.bones}
        bones = []
        for joint in solved_pose.rig.joints:
            reference = reference_by_name.get(joint.name)
            bones.append(
                BoneModel(
                    name=joint.name,
                    parent_name=joint.parent_name,
                    local_position=reference.local_position if reference else joint.local_position,
                    local_rotation=reference.local_rotation if reference else joint.local_rotation,
                    world_position=joint.world_position,
                    world_rotation=joint.world_rotation,
                    rest_local_position=reference.rest_local_position if reference else joint.rest_local_position,
                    rest_local_rotation=reference.rest_local_rotation if reference else joint.rest_local_rotation,
                    rest_world_position=reference.rest_world_position if reference else joint.rest_world_position,
                    rest_world_rotation=reference.rest_world_rotation if reference else joint.rest_world_rotation,
                    bone_length=reference.bone_length if reference else joint.bone_length,
                    primary_axis=reference.primary_axis if reference else joint.primary_axis,
                )
            )
        return SkeletonModel(name=skeleton.name, bones=bones)
