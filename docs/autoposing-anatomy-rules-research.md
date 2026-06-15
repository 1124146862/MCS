# AutoPosing Anatomy Rules Research

Status: actionable research draft  
Purpose: turn public AutoPosing, rigging, biomechanics, and open model references into implementable linked-motion rules for the next naturalness pass.

## Executive Summary

The next AutoPosing naturalness work should be a rule-driven anatomy layer, not a single larger IK method.

The current implementation already has the right high-level split: constraints, anatomy profile, pose prior, support solver, limb solver, spine solver, relaxation solver, controller schema, finger solver, and retargeter. The gap is that most whole-body behavior is still produced by fixed scalar weights, for example arm overreach pushes pelvis/chest/head with hard-coded values. That cannot express why a wrist drag should first recruit clavicle/shoulder, why a fixed controller should not become a pose-intent guide, why the head should resist drift, or why a support foot should dominate pelvis correction.

The concrete next design is:

```text
controllers
  -> semantic constraints
  -> anatomy frames and rule config
  -> procedural pose prior
  -> anatomy scoring
  -> targeted correction passes
  -> final IK/retarget
```

The most important correction to the previous draft: each controller must carry two separate meanings.

- Preservation: this point must stay where the user put it.
- Pose intent: this point should guide the inferred whole-body pose.

Cascadeur's public docs say active controllers are used to define/generate poses, while fixed controllers are not taken into account when generating poses. Therefore a fixed point should be treated as a hard preservation constraint but with low or zero pose-intent weight. A locked/active point should preserve its target and also drive pose inference. Always-active ankles should mostly drive support, not broad pose intent.

## Sources Used

### Cascadeur Product Semantics

- AutoPosing: https://cascadeur.com/help/tools/animation_tools/autoposing
- AutoPosing Tool settings: https://cascadeur.com/help/category/238
- Relaxation: https://cascadeur.com/help//relaxation_in_cascadeur
- AI tools statement: https://cascadeur.com/help/category/285
- AutoPosing for Fingers: https://cascadeur.com/help/category/205

Actionable facts:

- Active controllers are blue and used for generating poses.
- Inactive controllers follow the generated pose but do not define it.
- Ankle controllers are always active.
- Fixed controllers are not used for generating poses.
- Fixed middle-chain controllers, such as elbows and knees, are discouraged because they can distort AutoPosing.
- Direction controller lengths are exposed: pelvis 20, chest 30, head 20.
- AutoPosing exposes floor, flexible toe, snapping angle, and physics iteration settings.
- Finger AutoPosing requires five actual fingers; index and pinky affect the palm, other fingers affect their own chains.
- Cascadeur uses machine learning to predict pose intent, then uses relaxation/IK-like correction to restore connected body structure.

### Biomechanics and Open Models

- OpenSim Lower Limb Model 2010: https://opensimconfluence.atlassian.net/wiki/spaces/OpenSim/pages/53087777
- OpenSim Gait 2392/2354: https://opensimconfluence.atlassian.net/wiki/spaces/OpenSim/pages/53086215/Gait%2B2392%2Band%2B2354%2BModels
- Full-body Rajagopal model paper: https://pubmed.ncbi.nlm.nih.gov/27392337/
- NASA segment mass/COM report: https://ntrs.nasa.gov/api/citations/19700027497/downloads/19700027497.pdf
- NASA human integration handbook: https://www.nasa.gov/wp-content/uploads/2023/12/ochmo-hb-004-rev-a-dec2023.pdf
- CDC normal joint ROM: https://archive.cdc.gov/www_cdc_gov/ncbddd/jointrom/index.html
- ISB standards index: https://isbweb.org/activities/standards
- ISB shoulder/elbow/wrist/hand JCS PDF: https://media.isbweb.org/images/documents/standards/Wu%20et%20al%20J%20Biomech%2038%20%282005%29%20981%E2%80%93992.pdf

Actionable facts:

- Lower limb models commonly represent ankle, subtalar, and metatarsophalangeal joints as hinge-like/revolute joints.
- OpenSim Lower Limb Model 2010 gives useful first-pass lower-limb soft ranges: MTP -30 to 30 degrees, subtalar -20 to 20 degrees, ankle -40 plantarflexion to 20 dorsiflexion, knee 0 to 100 degrees.
- CDC ROM gives population reference ranges. For adults 20-44, examples include hip flexion around 130 degrees, knee flexion around 138-142 degrees, ankle dorsiflexion around 13 degrees, ankle plantar flexion around 55-62 degrees, shoulder flexion around 169-172 degrees, elbow flexion around 145-150 degrees.
- NASA/Dempster-style segment data supports approximate mass and center-of-mass estimation. It is not exact enough for physics, but good enough for a support tendency score.
- ISB standards support using semantic joint axes instead of one generic rotation frame for every limb.

### Shoulder Girdle and Scapulohumeral Rhythm

- In vivo scapulohumeral rhythm PubMed: https://pubmed.ncbi.nlm.nih.gov/19395283/
- Scapular activity and elevation review: https://pmc.ncbi.nlm.nih.gov/articles/PMC2857390/
- Shoulder impingement biomechanics: https://pmc.ncbi.nlm.nih.gov/articles/PMC3010321/
- Scapulothoracic rhythm study: https://pmc.ncbi.nlm.nih.gov/articles/PMC6620199/

Actionable facts:

- The shoulder is not only an upperarm hinge. Humerus, scapula, clavicle, and chest coordinate.
- The commonly cited 2:1 glenohumeral-to-scapulothoracic rhythm is a heuristic, not a hard law.
- Published in-vivo work reports ratios such as about 2.3 during arm raising and 2.7 during lowering.
- For animation, the useful rule is not exact scapula anatomy. The useful rule is: high or far wrist targets should recruit clavicle/shoulder/chest before the arm reaches impossible extension.

### Learned Priors and Future Direction

- SMPL: https://smpl.is.tue.mpg.de/
- VPoser/human_body_prior: https://github.com/nghorbani/human_body_prior

Actionable facts:

- Learned pose priors model correlations among joints and penalize impossible poses.
- VPoser optimizes body pose, translation, and global orientation iteratively against keypoints.
- This is useful as a future interface target, not as the next implementation step. A procedural scorer should come first so failures are inspectable.

## Current Implementation Gap

Current files already line up with a layered architecture:

- `app/autoposing/constraints.py`: extracts controller constraints.
- `app/autoposing/anatomy_profile.py`: defines known humanoid chains.
- `app/autoposing/pose_prior.py`: creates an initial pose.
- `app/autoposing/relaxation_solver.py`: orchestrates spine, limbs, support, fingers, and compensation.
- `app/autoposing/body_compensation.py`: applies simple capped pelvis/chest/head offsets.
- `app/autoposing/support_solver.py`: floor clamp and support targets.
- `app/autoposing/runtime_retargeter.py`: turns solved positions into local rotations.

Main gaps:

1. Constraint state is too coarse. `fixed`, `active`, `always_active`, and `selected` are collapsed into one engaged/strength idea.
2. Anatomy profile lacks frames, axes, mass weights, support contacts, and soft joint limits.
3. Compensation is not rule-specific. Arm and leg overreach use fixed weights instead of shoulder/support/head/spine reasoning.
4. There is no anatomy score object. We cannot tell whether a pose is bad because of COM, shoulder overreach, joint limits, head drift, or support drift.
5. Support is mostly floor clamp plus support target alignment. It does not estimate COM or a support region.
6. Shoulder/clavicle positions are only rest offsets from chest. Wrist motion does not currently recruit a real shoulder-girdle layer.

## Implementation Principle

Every anatomy rule should be implemented as a small pass with:

- Inputs: anatomy, constraints, current solved/prior positions, locks.
- Outputs: adjusted positions, optional orientation hints, and named score values.
- Caps: maximum translation/rotation per pass.
- Lock behavior: never overwrite hard preservation constraints.
- Debug: named pressure values for self-check and UI.

Avoid hidden magic weights. Put all thresholds in a single config object.

## Proposed Data Structures

### Constraint Semantics Split

Extend internal constraints conceptually like this:

```python
@dataclass(frozen=True)
class ConstraintInfluence:
    controller_id: str
    preservation_weight: float
    pose_intent_weight: float
    support_weight: float
    orientation_weight: float
    locked: bool
    fixed: bool
    always_active: bool
```

Suggested mapping:

| Controller state | Preservation | Pose intent | Support | Meaning |
| --- | ---: | ---: | ---: | --- |
| active/locked normal | 0.90 | 1.00 | 0.00 | User wants this point to define pose |
| selected while dragging | 0.95 | 1.00 | depends | Live user intent |
| fixed normal | 1.00 | 0.00-0.15 | 0.00 | Preserve target, do not infer whole body from it |
| fixed elbow/knee | 1.00 | 0.00 | 0.00 | Allowed but warned; do not drive torso |
| always-active ankle, not dragged | 0.85 | 0.10 | 1.00 | Support anchor |
| active ankle being dragged | 0.95 | 0.80 | 0.20 | Moving foot, not support foot |
| inactive | 0.00 | 0.00 | 0.00 | Follow solved layout only |

This is the first concrete change to make. It prevents fixed or always-active points from accidentally acting like broad pose guides.

### Anatomy Rule Config

Add a centralized config object, either in a new module or as part of anatomy profile:

```python
@dataclass(frozen=True)
class AnatomyRuleSet:
    shoulder: ShoulderRuleConfig
    support: SupportRuleConfig
    spine: SpineRuleConfig
    head: HeadRuleConfig
    joint_limits: JointLimitRuleConfig
    foot: FootRuleConfig
    masses: SegmentMassConfig
```

This config should be built from skeleton scale:

- `body_height = distance(floor_y, head_y)`
- `torso_length = distance(pelvis, spine_05)`
- `arm_length = length(upperarm, lowerarm) + length(lowerarm, hand)`
- `leg_length = length(thigh, calf) + length(calf, foot)`
- `foot_length = length(foot, ball)`

All thresholds below should be scale-relative, not fixed world units.

### Anatomy Score

Add one score object per solved frame:

```python
@dataclass(frozen=True)
class AnatomyScore:
    support_error: float
    com_error: float
    shoulder_error_l: float
    shoulder_error_r: float
    joint_limit_pressure: dict[str, float]
    spine_smoothness_error: float
    head_drift_error: float
    foot_roll_pressure_l: float
    foot_roll_pressure_r: float
```

The first implementation can keep this internal and expose only test hooks. Later the UI can show it as debug pressure.

## Rule Cards

### Rule 0: Controller State Semantics

Goal: prevent controller states from driving the wrong layer.

Inputs:

- Controller `active`, `fixed`, `always_active`, `selected`.
- Controller role: main, secondary, lesser, additional, direction, finger.
- Whether the controller is currently being dragged.

Outputs:

- Preservation constraints.
- Pose-intent constraints.
- Support constraints.
- Orientation hints.

Algorithm:

```text
for each controller:
  if inactive:
    no hard constraint; layout follows solved pose
  if active:
    preservation_weight = 0.90
    pose_intent_weight = 1.00
  if fixed:
    preservation_weight = 1.00
    pose_intent_weight = 0.00 for elbow/knee/secondary/support-middle
    pose_intent_weight = 0.15 max for main endpoints
  if always_active ankle and not dragged:
    support_weight = 1.00
    pose_intent_weight = 0.10
```

Files affected later:

- `app/autoposing/constraints.py`
- `app/autoposing/pose_prior.py`
- `app/autoposing/relaxation_solver.py`

Tests:

- Fixed wrist preserves target but produces less chest/pelvis compensation than active wrist.
- Fixed elbow does not move chest/pelvis.
- Always-active non-dragged ankle stabilizes support but does not rotate whole body as an active pose guide.

Priority: P0. This should happen before tuning naturalness.

### Rule 1: Shoulder Girdle Recruitment

Goal: dragging a wrist should recruit clavicle, shoulder, and chest before the arm looks stretched or disconnected.

Inputs:

- `wrist_*_main` pose-intent target.
- `elbow_*_secondary` bend hint.
- Rest positions: `spine_05`, `clavicle_*`, `upperarm_*`, `hand_*`.
- Current reach pressure from `LimbSolver`.
- Chest/head/pelvis lock states.

Trigger:

- Wrist controller has pose-intent weight > 0.
- Arm reach pressure > 0.05, or wrist delta > `0.12 * arm_length`, or target height above shoulder > `0.25 * arm_length`.

Heuristic:

```text
arm_activation =
  max(
    smoothstep(0.12 * arm_length, 0.65 * arm_length, length(wrist_delta)),
    reach_pressure
  ) * pose_intent_weight

height_activation =
  smoothstep(0.15 * arm_length, 0.75 * arm_length, target_y - shoulder_y)

forward_activation =
  positive_dot(normalize(wrist_delta), character_forward)

clavicle_up =
  up * arm_length * 0.035 * arm_activation * height_activation

clavicle_forward =
  forward * arm_length * 0.025 * arm_activation * forward_activation

clavicle_out =
  side_out * arm_length * 0.020 * arm_activation

chest_shift =
  wrist_delta * 0.06 * arm_activation

chest_yaw_or_twist_hint =
  side_sign * clamp(length(horizontal_wrist_delta) / arm_length, 0, 1) * 8 degrees
```

Caps:

- Clavicle translation cap: `0.08 * arm_length`.
- Chest translation cap per arm: `0.05 * torso_length`.
- Pelvis translation from arm only: `0.025 * body_height`.
- Head follow from arm only: `0.015 * body_height`.

Scapulohumeral heuristic:

- Treat 2.3:1 as a soft rhythm for high arm poses only.
- If computed arm elevation is under 45 degrees, do not apply rhythm.
- If elevation is high, target scapula/clavicle contribution should grow toward roughly 25-35% of apparent elevation, but clamp heavily.

Outputs:

- Adjusted `clavicle_l/r` solved positions.
- Adjusted `upperarm_*` chain start through clavicle offset.
- Small chest target shift and orientation hint.
- Anatomy score: `shoulder_error_l/r`, `shoulder_recruitment_l/r`.

Files affected later:

- New `app/autoposing/shoulder_solver.py`.
- `app/autoposing/relaxation_solver.py` calls shoulder pass before final arm solve.
- `app/autoposing/runtime_retargeter.py` uses resulting clavicle/upperarm positions.

Acceptance:

- Far wrist drag: clavicle moves visibly, chest follows slightly, pelvis much less.
- Support foot drift stays under 0.5 world units or a scale-relative equivalent.
- Arm bone length error remains at existing tolerance.
- Head does not move more than chest unless head is active.

Priority: P0.

### Rule 2: Pelvis/Chest/Head Compensation Weighting

Goal: replace fixed global weights with context-aware compensation.

Inputs:

- Reach pressure from arms and legs.
- Pose-intent weights.
- Support state.
- Locks: pelvis, chest, head, direction controllers.

Current issue:

- `relaxation_solver.py` uses fixed arm and leg compensation weights.
- It cannot tell a high hand reach from a low side reach.
- It cannot tell moving a foot from using a foot as support.

Proposed logic:

```text
if arm overreach:
  chest_weight = 0.35 to 0.55
  pelvis_weight = 0.04 to 0.16
  head_weight = 0.05 to 0.12
  increase chest_weight for high/forward reaches
  decrease pelvis_weight if both feet are supporting

if one ankle is moved:
  pelvis_weight = 0.45 to 0.75
  chest_weight = 0.12 to 0.28
  head_weight = 0.03 to 0.10
  opposite support foot weight = 1.00

if both ankles are moved:
  pelvis follows average feet more strongly
  support COM correction is reduced

if head active:
  head_weight = 0
  neck/spine absorb residual
```

Outputs:

- Replace `apply_capped_compensation(...)` calls with named rule results.
- Anatomy score includes `compensation_source = arm_l | arm_r | leg_l | leg_r | mixed`.

Files affected later:

- Replace or expand `app/autoposing/body_compensation.py`.
- `app/autoposing/relaxation_solver.py`.

Tests:

- Wrist drag produces more chest than pelvis movement.
- Ankle drag produces more pelvis than chest movement.
- Head active prevents head auto-follow.
- Chest direction active reduces automatic chest yaw from wrist.

Priority: P0.

### Rule 3: Support Foot and COM Correction

Goal: keep standing poses from looking like the character is falling when one hand or one foot moves.

Inputs:

- Solved positions of pelvis, spine, head, arms, legs.
- Support contact points: `foot_l/r`, `ball_l/r`.
- Controller state: always-active ankle, active dragged ankle, fixed foot/ball.
- Floor Y.

Approximate segment mass config:

```text
head_neck: 0.08
trunk_pelvis: 0.43
upperarm each: 0.028
forearm each: 0.018
hand each: 0.007
thigh each: 0.10
calf each: 0.046
foot each: 0.014
```

These values are approximations inspired by NASA/Dempster-style segment tables. Normalize the total to 1.0 at runtime.

Segment COM points:

```text
upperarm: 0.47 from shoulder to elbow
forearm: 0.41 from elbow to hand
thigh: 0.39 from hip to knee
calf: 0.41 from knee to ankle
hand: hand joint or midpoint toward hand controller
foot: midpoint foot to ball
trunk_pelvis: weighted blend pelvis 0.45, chest 0.45, head 0.10
head_neck: head
```

Support region:

```text
if both feet supporting:
  support_points = foot_l, ball_l, foot_r, ball_r
  support_polygon = convex_hull(project_xz(points))

if one foot supporting:
  support_segment = foot -> ball
  support_capsule_radius = max(0.18 * foot_length, 0.03 * body_height)

if no foot supporting:
  skip COM correction
```

Correction:

```text
com_xz = weighted_average(segment_com_xz)
nearest = nearest_point_on_support_region(com_xz)
error = nearest - com_xz

if length(error) > dead_zone:
  pelvis += xz(error) * 0.55 * support_weight
  chest += xz(error) * 0.25 * support_weight
  head += xz(error) * 0.10 * support_weight
```

Dead zone:

- Two-foot support: `0.06 * body_height`.
- One-foot support: `0.035 * body_height`.

Caps:

- Pelvis COM correction per pass: `0.06 * body_height`.
- Chest COM correction per pass: `0.035 * body_height`.
- Never move a fixed/active support foot to satisfy COM.

Outputs:

- Adjusted pelvis/chest/head targets.
- `AnatomyScore.com_error`.
- `AnatomyScore.support_error`.

Files affected later:

- New `app/autoposing/balance_solver.py`.
- Expand `app/autoposing/support_solver.py` or keep support floor logic there and COM logic in `balance_solver.py`.
- `app/autoposing/pose_prior.py` can use a lightweight version before relaxation.

Tests:

- Far single-wrist drag shifts pelvis slightly toward the support region.
- Moving left ankle keeps right foot/ball stable.
- Two support feet produce less pelvis drift than one support foot.
- If both ankles are active, COM correction is reduced so foot posing remains controllable.

Priority: P0/P1. Start P0 with support stability, then add COM in P1 if needed.

### Rule 4: Spine Smoothness and Head Anti-Drift

Goal: torso should distribute motion smoothly; head should not over-follow limbs.

Inputs:

- Solved spine chain: pelvis, spine_01..spine_05, neck, head.
- Chest/head/pelvis constraints and direction constraints.

Spine smoothness score:

```text
for each internal spine joint i:
  curvature_i = position[i-1] - 2 * position[i] + position[i+1]
spine_smoothness_error = sum(length(curvature_i)) / torso_length
```

Correction:

```text
for 2 iterations:
  for spine_01..spine_04:
    target = lerp(current, midpoint(prev, next), 0.15)
    preserve chain length by re-solving linear spine chain after smoothing
```

Head anti-drift:

```text
if not head active/fixed and not head_dir active:
  desired_head = rest_head + chest_delta * 0.18 + pelvis_delta * 0.05
  head = lerp(head, desired_head, 0.55)
  cap head displacement relative to chest to 0.04 * body_height
```

Outputs:

- Smoother spine positions.
- Limited head follow.
- Scores: `spine_smoothness_error`, `head_drift_error`.

Files affected later:

- `app/autoposing/spine_solver.py`
- `app/autoposing/relaxation_solver.py`

Tests:

- Far hand drag bends chest/spine without pulling head too far.
- Head active target is preserved.
- Chest direction remains authoritative when active.

Priority: P1 after shoulder/support basics.

### Rule 5: Joint Soft Limits and Pressure

Goal: warn and bias away from implausible joint rotations without forbidding stylized animation.

Inputs:

- Retarget local rotations.
- Anatomy joint axes.
- Controller engagement state.

Initial soft ranges:

| Joint | Suggested first range | Source/use |
| --- | ---: | --- |
| Shoulder flexion/elevation | 0 to 170 deg | CDC reference; soft only |
| Elbow flexion | 0 to 150 deg | CDC reference |
| Knee flexion | 0 to 140 deg for animation, 0 to 100 deg strict gait model | CDC + OpenSim |
| Hip flexion | 0 to 130 deg | CDC reference |
| Hip extension | 0 to 18 deg | CDC reference |
| Ankle dorsiflexion | 0 to 20 deg | OpenSim/CDC |
| Ankle plantarflexion | 0 to 40-55 deg | OpenSim/CDC |
| MTP/ball flexion | -30 to 30 deg | OpenSim lower limb model |
| Subtalar/lateral foot roll | -20 to 20 deg | OpenSim lower limb model |

Pressure function:

```text
ratio = abs(angle) / soft_limit
if ratio <= 0.80:
  pressure = 0
elif ratio <= 1.00:
  pressure = smoothstep(0.80, 1.00, ratio) * 0.5
else:
  pressure = 0.5 + min(0.5, (ratio - 1.0) * 0.75)
```

Correction:

- P1 first version: no hard correction, only score and UI/debug pressure.
- P2 version: use tiny bias toward nearest legal bend plane if pressure > 0.75 and controller is not fixed.

Files affected later:

- New `app/autoposing/joint_limits.py`.
- `app/autoposing/runtime_retargeter.py` can compute pressure after local rotations.
- `SolvedEffectorModel` or debug model may optionally expose joint-limit pressure.

Tests:

- Extreme elbow/knee bend produces pressure state but preserves target behavior.
- Fixed stylized pose is not forcibly corrected.
- Pressure does not trigger for neutral rest pose.

Priority: P1.

### Rule 6: Foot/Ball Roll and Ground Interaction

Goal: foot controls should affect foot/ball orientation and contact, not pull the leg chain apart.

Inputs:

- `ankle_*_main`
- `foot_*_secondary`
- `foot_*_additional`
- `ball_*_lesser`
- Floor Y and support state.

Current useful fact:

- The current skeleton has `foot_*` and `ball_*`, no independent toe bone. Therefore `ball_*_lesser` should act as toe/ball endpoint.

Forward roll:

```text
foot_forward = normalize(rest_ball - rest_foot)
target_forward = normalize(foot_secondary_or_ball - solved_foot)
roll_angle = signed_angle_on_plane(foot_forward, target_forward, side_axis)
roll_angle = clamp(roll_angle, -30 deg, 30 deg)
```

Lateral roll:

```text
foot_lateral = normalize(foot_additional - solved_foot)
lateral_angle = signed_angle_on_plane(rest_lateral, foot_lateral, forward_axis)
lateral_angle = clamp(lateral_angle, -20 deg, 20 deg)
```

Ground snapping:

```text
if support_weight > 0:
  foot_y = max(foot_y, floor_y)
  ball_y = max(ball_y, floor_y)
  if angle_between(foot_arch, floor_plane) < snapping_angle:
    bias foot/ball toward floor contact
```

Use Cascadeur's snapping angle default 20 degrees as the first tuning target.

Outputs:

- Ball solved point remains clamped to foot reach.
- Foot/ball retarget rotation changes.
- Ankle main position remains leg IK endpoint and should not be pulled by ball.
- Scores: `foot_roll_pressure_l/r`.

Files affected later:

- `app/autoposing/support_solver.py`
- `app/autoposing/relaxation_solver.py` `_ball_roll_position`
- `app/autoposing/runtime_retargeter.py`

Tests:

- Dragging `ball_l_lesser` changes ball/foot rotation but not ankle position.
- Foot/ball never goes below floor.
- Far ball drag clamps and shows pressure tether.
- Support foot remains stable when only hand is dragged.

Priority: P1.

### Rule 7: Hand/Finger/Palm Coupling

Goal: fingers should improve palm direction without overriding explicit hand controls.

Inputs:

- Hand orientation controllers: `hand_*_secondary`, `hand_*_additional`.
- Finger tips/pre-tips.
- Finger chain profile with thumb opposable flag.

Rules:

```text
if hand secondary/additional active/fixed:
  hand orientation weight = 1.0
  index/pinky palm spread weight = 0.03 to 0.05
else:
  index/pinky palm spread weight = 0.15 to 0.20

thumb:
  use opposition plane, shorter reach clamp, stronger curl-side bias

middle/ring:
  affect only own finger chain by default
```

Outputs:

- Palm spread hint.
- Finger pressure.
- Thumb-specific bend/orientation.

Files affected later:

- `app/autoposing/finger_profile.py`
- `app/autoposing/finger_solver.py`
- `app/autoposing/runtime_retargeter.py`

Tests:

- Hand additional active dominates palm orientation.
- Index/pinky active gives small palm rotation.
- Middle/ring active does not move palm much.
- Thumb tip/pre-tip changes thumb rotations without pulling other fingers.

Priority: P2 because body naturalness is more visible.

## Priority Plan

### P0.0: Fix Constraint Semantics

Why first:

- It prevents fixed controllers and always-active ankles from being misused as pose-intent guides.
- It aligns implementation with Cascadeur's active/fixed meaning.

Concrete tasks:

- Add internal influence weights or equivalent properties to constraints.
- Update pose prior and compensation to use pose-intent weight.
- Update support solver to use support weight.
- Keep preservation behavior for fixed points.

Self-checks:

- Active wrist drives chest/pelvis.
- Fixed wrist preserves target but drives much less chest/pelvis.
- Fixed elbow does not drive torso.
- Always-active ankle stabilizes foot but does not overdrive pose.

### P0.1: Expand Anatomy Profile into Frames

Why:

- Every later rule needs semantic axes and scale values.

Concrete tasks:

- Add side axis, up axis, forward axis.
- Add per-limb chain scale values.
- Add shoulder frame: chest, clavicle, upperarm, hand.
- Add foot frame: ankle/foot/ball, forward/lateral/up.
- Add approximate body height, torso length, arm length, leg length, foot length.

Files:

- `app/autoposing/anatomy_profile.py`

Self-checks:

- Profile builds with missing ball/clavicle gracefully.
- Lengths are positive and scale-relative.
- Left/right side axes have opposite signs where needed.

### P0.2: Shoulder Girdle Solver

Why:

- This is the largest visible difference when dragging one hand.

Concrete tasks:

- Add `shoulder_solver.py`.
- Run shoulder pass before final arm IK solve.
- Adjust clavicle and arm chain starts.
- Add chest orientation/position bias.

Self-checks:

- Far wrist target moves clavicle more than current implementation.
- Chest responds but pelvis remains capped.
- Bone length tolerance unchanged.

### P0.3: Support Stability First

Why:

- Single-foot/hand drags currently risk visual instability.

Concrete tasks:

- Add support role distinction: support foot vs moving foot.
- Keep opposite support foot/ball target stable when one ankle is dragged.
- Only then add approximate COM correction.

Self-checks:

- Drag left ankle: right foot and right ball drift under threshold.
- Drag wrist: both support feet remain stable unless active/fixed.

### P0.4: Replace Hard-Coded Compensation Weights

Why:

- Once shoulder/support rules exist, fixed global weights are misleading.

Concrete tasks:

- Replace two direct `apply_capped_compensation` calls with a `BodyCompensationPlan`.
- Let plan store source, weights, caps, and lock decisions.

Self-checks:

- Arm overreach: chest > pelvis.
- Leg overreach: pelvis > chest.
- Head active disables head auto-follow.

### P1.0: COM Scoring and Correction

Why:

- It improves standing plausibility after support rules are stable.

Concrete tasks:

- Add `balance_solver.py`.
- Estimate segment COM.
- Compute support polygon/capsule.
- Apply capped pelvis/chest correction.

Self-checks:

- Far reach moves COM toward support.
- Both feet support less correction than one foot support.

### P1.1: Joint Soft-Limit Pressure

Why:

- Useful for debugging and preventing extreme failures.

Concrete tasks:

- Add `joint_limits.py`.
- Use CDC/OpenSim values as soft reference.
- Output named pressure; do not hard clamp at first.

Self-checks:

- Extreme ankle/knee/elbow raises pressure.
- Normal pose pressure near zero.

### P1.2: Foot/Ball Roll Refinement

Why:

- It brings foot/toe behavior closer to Cascadeur without a full physics engine.

Concrete tasks:

- Interpret `ball_*_lesser` as MTP/toe endpoint.
- Clamp foot/ball roll to soft ranges.
- Preserve ankle as leg endpoint.

Self-checks:

- Ball drag changes foot/ball rotation, not ankle position.

### P2: Finger/Palm Refinement

Why:

- Already structurally present; now needs priority rules and thumb-specific behavior.

Concrete tasks:

- Add thumb opposition axis.
- Tune index/pinky palm spread vs hand additional priority.

## Detailed Acceptance Matrix

| Scenario | Expected result | Numeric acceptance |
| --- | --- | --- |
| Far wrist drag upward | clavicle, chest, pelvis respond in that order | clavicle delta > 0.02 arm length; chest delta > pelvis delta; head delta < chest delta unless head active |
| Far wrist drag sideways | shoulder side responds, opposite side mostly stable | active-side clavicle delta > 2x opposite-side clavicle delta |
| Fixed wrist far from body | target preserved but less whole-body inference | chest/pelvis response < 35% of active wrist response |
| Fixed elbow | elbow target preserved, no broad torso inference | chest/pelvis response near zero unless wrist also active |
| One ankle dragged | pelvis follows, opposite support foot stable | opposite foot/ball drift < 0.01 body height |
| Hand drag with both feet support | COM correction small but visible | pelvis COM correction <= 0.06 body height |
| Both ankles active | pelvis follows average feet, support correction reduced | no forced snap toward old support polygon |
| Ball drag | foot/ball rotation changes, ankle not pulled | ankle displacement from leg IK target < 0.005 body height |
| Extreme knee/elbow | pressure visible, no bone stretch | bone length error <= existing tolerance |
| Head active | head target preserved | head auto-follow weight = 0 |

## Open Questions

1. Should fixed endpoint controllers have exactly zero pose-intent weight, or a very small 0.10-0.15 fallback for main endpoints?
   - Recommendation: start with zero for middle-chain controllers and 0.10 for main endpoints. This preserves animation usability while respecting Cascadeur semantics.

2. Should COM correction run during every preview frame?
   - Recommendation: yes, but only as a cheap weighted average and one correction pass. Full iterative balance can wait.

3. Should joint soft limits correct pose or only show pressure?
   - Recommendation: pressure only in first pass. Correction can make stylized animation feel too restrictive.

4. Should shoulder-girdle adjustment be position-based or rotation-based?
   - Recommendation: start position-based because the solver works in world positions; retargeter will convert to rotations. Add rotation scoring later.

5. Should the future learned prior use SMPL/VPoser directly?
   - Recommendation: not immediately. First define `PosePriorProvider` interface and maybe a small pose-library retrieval mode. Direct SMPL/VPoser integration has licensing, skeleton mapping, and runtime cost concerns.

## Suggested Module Layout

Add later:

- `app/autoposing/anatomy_rules.py`
  - config dataclasses and skeleton-scale derived thresholds.
- `app/autoposing/anatomy_scorer.py`
  - computes named score values from solved pose.
- `app/autoposing/shoulder_solver.py`
  - shoulder girdle recruitment.
- `app/autoposing/balance_solver.py`
  - support polygon and approximate COM.
- `app/autoposing/joint_limits.py`
  - soft ROM tables and pressure calculation.

Modify later:

- `app/autoposing/constraints.py`
  - split preservation/intent/support influence.
- `app/autoposing/anatomy_profile.py`
  - add semantic frames and body scale.
- `app/autoposing/pose_prior.py`
  - use intent weights instead of raw active/fixed strength.
- `app/autoposing/relaxation_solver.py`
  - call shoulder/support/balance passes explicitly.
- `app/autoposing/body_compensation.py`
  - replace generic capped compensation with context plan.
- `app/autoposing/runtime_retargeter.py`
  - consume foot/shoulder/joint-limit hints.

## Minimal P0 Implementation Order

Do not implement everything at once. The most controlled sequence is:

1. Add constraint influence semantics without changing visual behavior.
2. Add anatomy frames and derived scale values.
3. Add anatomy score object in read-only mode.
4. Add shoulder girdle solver and tests.
5. Add support stability distinction.
6. Replace hard-coded arm/leg compensation with compensation plan.
7. Add COM scoring/correction only after 1-6 are stable.

This gives visible improvement after step 4, but keeps each failure diagnosable.

## Why This Is More Actionable Than A General Pose Prior

A "stronger anatomy prior" can mean too many things. For this project, it should mean these concrete constraints:

- active means pose intent; fixed means preservation;
- wrist intent recruits shoulder girdle before torso;
- torso compensation is source-specific;
- support foot has a stronger vote than free limbs;
- COM projection is nudged toward the support region;
- head drift is resisted unless explicitly controlled;
- foot/ball roll affects orientation/contact, not leg length;
- joint ROM is a soft pressure system, not a hard clamp;
- learned prior remains a future provider, not the next dependency.

That is enough to move the system from "procedural IK with controller schema" toward "anatomy-informed sparse pose completion" while staying debuggable.
