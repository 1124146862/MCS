from core.autoposing.models import AutoPosingRigModel, SolvedPoseModel
from core.skeleton.models import SkeletonModel

from .anatomy_profile import build_humanoid_anatomy_profile
from .constraints import extract_pose_constraints
from .pose_prior import PosePriorEstimator
from .relaxation_solver import WholeBodyRelaxationSolver


class AutoPosingSolver:
    def __init__(self) -> None:
        self._pose_prior = PosePriorEstimator()
        self._relaxation_solver = WholeBodyRelaxationSolver()

    def solve(self, rig: AutoPosingRigModel, skeleton: SkeletonModel) -> SolvedPoseModel:
        if not skeleton.bones:
            return SolvedPoseModel(status_message="No skeleton available.")

        anatomy = build_humanoid_anatomy_profile(skeleton)
        constraints = extract_pose_constraints(rig, anatomy)
        initial_pose = self._pose_prior.estimate(skeleton, anatomy, constraints)
        return self._relaxation_solver.solve(rig, skeleton, anatomy, constraints, initial_pose)
