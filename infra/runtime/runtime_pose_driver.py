from core.autoposing.models import RetargetJointPoseModel, RetargetPoseModel
from core.rig.models import RigModel

from .runtime_pose_writer import RuntimePoseWriter


class RuntimePoseDriver(RuntimePoseWriter):
    """Backward-compatible runtime pose driver facade.

    The old prototype wrote proxy-rig world positions directly to the runtime
    model. The rebuilt pipeline keeps that public name available, but routes
    the data through the new retarget-pose writer.
    """

    def apply_proxy_pose(self, proxy_rig: RigModel) -> bool:
        pose = RetargetPoseModel(
            joints=[
                RetargetJointPoseModel(
                    name=joint.name,
                    local_position=joint.local_position,
                    local_rotation=joint.local_rotation,
                    world_position=joint.world_position,
                )
                for joint in proxy_rig.joints
            ]
        )
        return self.apply_retarget_pose(pose)
