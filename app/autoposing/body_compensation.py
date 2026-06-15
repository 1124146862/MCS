from core.math import add, scale
from core.skeleton.models import Vec3


def apply_capped_compensation(
    pelvis: Vec3,
    chest: Vec3,
    head: Vec3,
    compensation: Vec3,
    *,
    pelvis_weight: float,
    chest_weight: float,
    head_weight: float,
    pelvis_locked: bool,
    chest_locked: bool,
    head_locked: bool,
) -> tuple[Vec3, Vec3, Vec3]:
    if compensation == (0.0, 0.0, 0.0):
        return pelvis, chest, head

    if not pelvis_locked:
        pelvis = add(pelvis, scale(compensation, pelvis_weight))
    if not chest_locked:
        chest = add(chest, scale(compensation, chest_weight))
    if not head_locked:
        head = add(head, scale(compensation, head_weight))
    return pelvis, chest, head
