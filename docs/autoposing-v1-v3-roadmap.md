# MCS AutoPosing V1-V3 Roadmap

## Goal

This roadmap now assumes the rebuilt four-layer AutoPosing path:

`Controller Target -> Solver Skeleton -> Deform Retarget -> Runtime FBX Skeleton`

It replaces the older prototype-only path:

`Controller -> ProxyRig Solve -> Runtime Joint Apply -> Skinned Mesh`

The meaning of each layer is:

- `Controller Target`: user drag target and state holder
- `Solver Skeleton`: length-preserving pose solution
- `Deform Retarget`: maps solved motion into the real FBX deform chain
- `Runtime FBX Skeleton`: final joint hierarchy that drives the skinned mesh

## Current Status

Project locations:

- Code project root: local `MCS` source tree
- Research docs root: local motion-capture notes workspace

What already exists:

- Qt Quick 3D viewport and dark editor shell
- Runtime FBX import into generated QML assets
- Camera controls, orientation gizmo, and floor grid
- Skeleton preview overlay and AutoPosing controller overlay
- V1 drag-space interaction
- V2 starting point with two-bone IK and proxy rig solving

What is still missing:

- Driving the real runtime FBX joints
- Real skinned mesh deformation
- A full Cascadeur-class AutoPosing solver
- Keyframes, constraints, and undo/redo

## V1

### Target

V1 is about making the editing foundation feel alive:

- controllers can be dragged reliably
- drag distance is decoupled from zoom level
- controller motion can drive an internal proxy rig

### Main Modules

- `core/autoposing/models.py`
- `core/rig/models.py`
- `app/autoposing/service.py`
- `ui/viewport/AutoPosingOverlay.qml`

### Acceptance

- hand, foot, head, and pelvis controls move clearly
- green points can follow and re-layout from blue points
- the preview skeleton alone already looks like an editable rig

## V2

### Target

V2 upgrades the system from fake interpolation to real pose solving.

Core work:

- two-bone IK for `upperarm -> lowerarm -> hand`
- two-bone IK for `thigh -> calf -> foot`
- chest / spine / head distribution solving
- green / blue / fixed controller state rules
- elbow and knee bend-plane behavior

### What Is Already Implemented

- `core/ik/two_bone.py` now exists
- `AutoPosingService` now solves limb chains with real two-bone IK
- wrist and ankle targets can influence elbows, knees, chest, and pelvis
- controller display positions are derived from the solved proxy rig

### V2 Boundary

V2 can make the proxy skeleton move in a much more believable way, but it still cannot fully match Cascadeur yet.

Why:

- Cascadeur uses a more advanced full-body constraint system
- its AutoPosing includes pose propagation, balance, center-of-mass logic, and richer coordination
- this prototype still does not write the solved pose back into the real runtime skeleton

Correct V2 expectation:

- skeleton motion becomes much more reasonable
- limbs no longer move by only a tiny amount
- the real body mesh still does not fully deform with the solved pose

### V2 Implementation Order

1. stabilize drag space with ray-plane intersection
2. establish the proxy rig
3. bind controllers to the proxy rig
4. add two-bone IK
5. add spine / chest / head distribution
6. add state-machine and fixed-point rules
7. run scripted and visual self-checks

### V2 Acceptance

- moving a wrist clearly recomputes the elbow
- moving an ankle influences the knee and pelvis
- chest and head react with visible pose propagation
- fixed points participate in solving but are not dragged away automatically

## V3

### Target

V3 is where the real imported body starts moving:

- find runtime joint hierarchy
- build `solver joint -> runtime joint` mapping
- write rotation / translation back to runtime joints
- let `Skin` drive the real mesh deformation

### Main Modules

- `infra/runtime/model_adapter.py`
- `infra/runtime/joint_binding.py`
- `app/autoposing/runtime_bridge.py`

### Acceptance

- moving a wrist deforms the real arm mesh
- moving an ankle deforms the real leg and pelvis mesh
- mesh motion stays visible even when skeleton overlay is hidden

## Scope Boundaries

### V1 should not do

- direct editing of real FBX joints
- keyframe system work
- complex inspector or constraints UI

### V2 should not do

- solving logic directly inside QML
- mixing mesh deformation into the AutoPosing core

### V3 should not do

- skipping the proxy rig and editing the mesh layer directly

## Priority Right Now

The next correct order is still:

1. make V2 limb IK, bend planes, and spine distribution stable
2. verify that the skeleton really moves through scripts and screenshots
3. then move into V3 and write the result back to the real FBX body
