from __future__ import annotations

from core.skeleton.models import SkeletonModel

from .controller_schema import (
    AutoPosingControllerDefinition,
    AutoPosingControllerEdgeDefinition,
    AutoPosingControllerGraphDefinition,
)
from .finger_profile import build_finger_anatomy_profile


def build_finger_controller_schema(skeleton: SkeletonModel) -> AutoPosingControllerGraphDefinition:
    profile = build_finger_anatomy_profile(skeleton)
    graph = AutoPosingControllerGraphDefinition()
    for chain in profile.chains.values():
        suffix = "L" if chain.side == "l" else "R"
        label = chain.finger_name.capitalize()
        graph.controllers.append(
            AutoPosingControllerDefinition(
                identifier=chain.pre_tip_controller_id,
                name=f"{label} Pre Tip {suffix}",
                role="finger_pre_tip",
                joint_name=chain.pre_tip_name,
                world_position=chain.pre_tip_world_position,
                visible=False,
            )
        )
        graph.controllers.append(
            AutoPosingControllerDefinition(
                identifier=chain.tip_controller_id,
                name=f"{label} Tip {suffix}",
                role="finger_tip",
                joint_name=chain.distal_name,
                world_position=chain.tip_world_position,
                visible=False,
            )
        )
        graph.edges.append(
            AutoPosingControllerEdgeDefinition(
                start_controller_id=f"hand_{chain.side}_secondary",
                end_controller_id=chain.pre_tip_controller_id,
                edge_type="topology",
                visible=False,
            )
        )
        graph.edges.append(
            AutoPosingControllerEdgeDefinition(
                start_controller_id=chain.pre_tip_controller_id,
                end_controller_id=chain.tip_controller_id,
                edge_type="topology",
                visible=False,
            )
        )
    return graph
