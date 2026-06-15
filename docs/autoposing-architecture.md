# MCS AutoPosing Architecture

## Goal

This document defines the rebuilt AutoPosing stack used by the Qt / PySide prototype.

The stable data flow is now:

`Controller Target -> Solver Skeleton -> Deform Retarget -> Runtime FBX Skeleton`

This replaces the older prototype path:

`Controller -> ProxyRig -> Direct Runtime Apply`

The new path exists to stop large drags from stretching the character into invalid shapes.

## Layer Rules

### `core/`

Pure domain data only.

Main models:

- `core/autoposing/models.py`
- `core/rig/models.py`
- `core/skeleton/models.py`
- `core/math/transforms.py`

Responsibilities:

- rest pose data
- solved pose data
- controller target state
- retarget pose state
- shared transform math

Forbidden here:

- QML references
- runtime node access
- file import logic

### `app/`

Editor-facing pose logic lives here.

Main modules:

- `app/autoposing/controller_state.py`
- `app/autoposing/reach_constraints.py`
- `app/autoposing/body_compensation.py`
- `app/autoposing/solver.py`
- `app/autoposing/runtime_retargeter.py`
- `app/autoposing/service.py`
- `app/document/document_manager.py`

Responsibilities:

- controller mode, selection, fixed state
- reach clamp and pressure classification
- two-bone IK solving
- spine / neck chain solving
- limited pelvis / chest / head compensation
- solver skeleton to FBX deform skeleton retargeting

### `infra/`

External resources and runtime node write-back live here.

Main modules:

- `infra/importers/runtime_asset_importer.py`
- `infra/importers/skeleton_extractor.py`
- `infra/runtime/runtime_pose_writer.py`
- `infra/testing/autoposing_self_check.py`

Responsibilities:

- import FBX runtime assets
- extract rest skeleton data from generated QML
- write retarget pose into live runtime nodes
- run repeatable architecture checks

### `ui/`

Only presentation and interaction bridging.

Main modules:

- `ui/viewmodels/document_view_model.py`
- `ui/viewport/ViewportPanel.qml`
- `ui/viewport/SkeletonOverlay.qml`
- `ui/viewport/AutoPosingOverlay.qml`

Responsibilities:

- show target controllers and solved effectors separately
- expose solver pressure colors
- display status such as `Target unreachable - clamped`
- keep current editor layout and style intact

## Pose Data Models

### Controller Target

`ControllerTargetModel` stores what the user is actually dragging.

Important fields:

- `world_position`
- `default_position`
- `solved_world_position`
- `clamped_world_position`
- `pressure`
- `pressure_state`
- `clamped`

Meaning:

- the target can move outside reachable space
- the solved effector stays physically limited
- UI can show both at the same time

### Solver Skeleton

`RigModel / JointModel` is now treated as the solved pose skeleton, not just a preview line set.

Each joint stores:

- rest local/world position
- rest local/world rotation
- current local/world position
- current local/world rotation
- bone length
- primary axis

### Retarget Pose

`RetargetPoseModel` is the boundary object between solving and runtime write-back.

Meaning:

- solver output does not write directly into deform joints
- runtime writer does not guess controller meaning
- auxiliary bones can be distributed cleanly in one place

## Solver Rules

### Limbs

Arms and legs use two-bone IK with fixed segment lengths.

Requirements:

- `upper -> lower -> end` chain length must stay constant
- bend hints come from elbow / knee secondary controllers
- unreachable targets are clamped to max reach

### Spine And Neck

The trunk is solved as chained fixed-length segments instead of loose interpolation.

Current scope:

- `pelvis -> spine_05`
- `spine_05 -> neck_02 -> head`

### Compensation

Compensation is intentionally limited.

Allowed:

- pelvis offset
- chest offset
- head offset

Not allowed:

- full-body explosion
- uncontrolled whole-character drift

### Pressure States

The solver labels joints and controller tethers with:

- `normal`
- `stretch`
- `squeeze`

UI uses these states for green / red / blue feedback.

## Retarget Rules

### Main Bones

Main humanoid chain retargets from solved world directions back into local FBX rotations.

Covered bones:

- pelvis / spine / neck / head
- clavicle / upperarm / lowerarm / hand
- thigh / calf / foot / ball

### Auxiliary Bones

Current retarget distribution:

- arm twist bones
- leg twist bones
- `neck_02`
- `ball_l`
- `ball_r`

Rules:

- twist bones receive weighted fractions of parent twist
- ball bones follow foot direction
- pelvis may receive limited world-position override
- root motion is still out of scope for this round

## UI Behavior

### Overlay Behavior

AutoPosing overlay now distinguishes:

- dragged target point
- solved effector point

The tether line between them exposes reach pressure in real time.

### Skeleton Preview

In AutoPosing mode the viewport shows the solved skeleton continuously, including while dragging.

Joint mode still shows the runtime skeleton.

## Verification

Run the scripted check from the project root:

```powershell
python -m infra.testing.autoposing_self_check
```

The self-check covers:

- near wrist drag
- unreachable wrist drag
- unreachable ankle drag
- direction controller rotation
- twist-bone distribution

The main acceptance targets for this rebuild are:

- limb lengths stay constant
- unreachable targets clamp instead of stretching the mesh
- runtime skeleton stays close to solver skeleton
- drag updates continue refreshing during pointer movement

## Current Scope

Included:

- humanoid torso + limbs + helper bones
- twist bones
- `neck_02`
- `ball_l / ball_r`

Not included yet:

- finger AutoPosing
- root motion
- full physics-based Cascadeur-class whole-body balancing
- keyframe / undo-redo system rebuild
