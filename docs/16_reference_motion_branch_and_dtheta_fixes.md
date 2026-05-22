# Reference Motion Branch Ambiguity and Backward-Yaw Convention Fixes

This document records two issues found in `Open_Duck_reference_motion_generator/open_duck_reference_motion_generator/placo_walk_engine.py` while generating reference motions for the Open Duck Mini V2, and the fixes applied.

Both issues only became visible during MuJoCo replay of the generated `recordings/*.json` files. The polynomial fit and downstream training pipeline blindly trust whatever Placo emits, so a wrong motion produces a policy that learns the wrong thing.

## Issue 1: IK branch ambiguity (mirror-sign knee/ankle)

### Symptom

After replaying generated reference motions in MuJoCo, some commands produced visually wrong leg poses: the right knee would bend *forward* out of the robot's body instead of bending behind/below it. Sign analysis confirmed the right-side knee/ankle joint values had the *same sign* as the left-side joints in the affected motions, instead of the expected mirror-sign convention.

About 3 of 9 sweep motions were wrong - the broken ones included backward + turn and turn-in-place. Forward motion was usually correct.

### Root cause

Open Duck Mini V2's home pose has mirror-symmetric leg joints:

| Joint | Home (rad) |
|------|------------|
| left_knee  | +1.368 |
| right_knee | -1.368 |
| left_ankle  | -0.784 |
| right_ankle | +0.784 |

The URDF declared symmetric joint limits `[-pi/2, +pi/2]` for both knees and both ankles, leaving the IK with two equally valid branches per joint (knee can bend forward or backward, ankle can pitch up or down). Placo's `KinematicsSolver` was configured with `enable_joint_limits(False)`, so even the URDF limits were ignored during planning.

`placo_walk_engine.py` had a knee-limit override block that applied the *same* `[lo, hi]` range to both `left_knee` and `right_knee`. For symmetric mirror joints this is wrong - it tries to force both knees onto the same sign. Combined with limits being disabled, the solver still chose freely between branches; the override didn't help.

The hip_pitch joints already had asymmetric URDF limits (`left_hip_pitch: [-1.222, +0.524]` and `right_hip_pitch: [-0.524, +1.222]`), which physically restricted them to the correct branch. That is why hip_pitch never flipped - the bug only manifested for joints where the URDF range was symmetric across zero.

### Failed approach (don't repeat)

Adding all 10 leg joints with their home pose values to the `joint_angles` dict in `placo_presets/medium.json` worked in the sense that all motions came out with correct mirror signs. But `joint_angles` is wired into Placo as a *persistent soft joints task* with weight 1.0 (`placo_walk_engine.py:71-73`), not a one-shot IK seed. So every solver step the task pulled the leg joints toward the standing pose, fighting against the foot trajectory tasks. The result was both legs flexing/extending in sync trying to satisfy two contradictory targets - in playback the robot bounced/jumped instead of stepping.

Lesson: `joint_angles` is for joints that should genuinely be held at a fixed value during the walk (head, neck). It is not a free "initial guess" hook.

### Fix

For `robot_type == "open_duck_mini_v2"`:
- Enable joint limits on the solver.
- Override the symmetric `[-pi/2, +pi/2]` knee/ankle limits with per-side ranges that exclude the wrong branch:

```python
self.solver.enable_joint_limits(True)
self.robot.set_joint_limits("left_knee",  0.01, 1.5708)
self.robot.set_joint_limits("right_knee", -1.5708, -0.01)
self.robot.set_joint_limits("left_ankle", -1.5708, -0.01)
self.robot.set_joint_limits("right_ankle", 0.01, 1.5708)
```

Other robots (`open_duck_mini`, `go_bdx`, etc.) keep the original behaviour to avoid breaking their reference generation.

After this change, all 9 sweep motions had correct mirror signs and the legs visually matched MuJoCo's home pose. The `medium.json` preset was reverted to head-only `joint_angles` (no leg joints) because joint limits now handle the branch selection.

### Why this is fragile if duplicated

If a future robot is added with the same symmetric URDF limit pattern, this bug will return silently - the polynomial fit will be wrong but training will still run and the policy will look mostly OK until backward/turn-in-place commands are tested. The check for it is cheap:

```python
# Verify mirror signs in any generated reference motion
left_knee_val * right_knee_val < 0  # must be True
left_ankle_val * right_ankle_val < 0  # must be True
```

Run this on each newly generated `.json` recording before refitting polynomials.

## Issue 2: Backward + turn direction convention

### Symptom

After fixing Issue 1, the four turn motions (forward+left, forward+right, back+left, back+right) had correct leg poses but the backward turn motions visually curved the *opposite* way from user intuition. Commanding `back + dtheta>0` ("back+left") produced a trajectory that curved into world-right.

### Root cause

Placo's footstep planner treats `dtheta` as a *body-frame* yaw rate, not a desired curvature direction. When the body moves backward and yaws CCW, the trajectory in world frame actually curves to the body's right - the same kinematics as a real car: holding the steering wheel left and reversing makes the car curve right.

For training purposes the goal is to make the *commanded sign* match the *visible world-frame curvature direction*. The intended convention:

- `cmd dtheta > 0` → world trajectory curves left, regardless of forward/back.
- `cmd dtheta < 0` → world trajectory curves right, regardless of forward/back.

### Fix

In `PlacoWalkEngine.set_traj`, flip the internal `d_theta` sign when `d_x < 0`:

```python
def set_traj(self, d_x, d_y, d_theta):
    if d_x < 0:
        d_theta = -d_theta
    self.d_x = d_x
    self.d_y = d_y
    self.d_theta = d_theta
    ...
```

The filename and the command stored as training input keep the original sign (`6_-0.148_0.0_+1.111.json` still means "back, dtheta_cmd=+1.111"). Only the planner internal value is flipped.

### Geometric result

With the flip, backward+left and forward+left trace the same world-space arc (centre on body's initial left). Backward traverses the arc clockwise, forward traverses CCW. The body's *heading* yaws in opposite directions during the two motions, exactly as a car body does when reversing along the same path with the steering held in the same position. This is what we want - the policy learns that "command direction == visible trajectory direction".

## Files touched

- `Open_Duck_reference_motion_generator/open_duck_reference_motion_generator/placo_walk_engine.py`
  - Added `self.robot_type` field.
  - For `open_duck_mini_v2`: enable joint limits, set per-side knee and ankle ranges.
  - `set_traj` flips `d_theta` when `d_x < 0`.
- `Open_Duck_reference_motion_generator/open_duck_reference_motion_generator/robots/open_duck_mini_v2/placo_presets/medium.json`
  - `joint_angles` reverted to head/neck only (leg joints removed).

## Verification procedure

After any change to the reference motion generator:

1. Regenerate the 9-motion sweep (`scripts/auto_waddle.py --duck open_duck_mini_v2 --sweep`).
2. Check every recording's first frame: `left_knee * right_knee < 0` and `left_ankle * right_ankle < 0` (mirror signs).
3. Visually replay `6_*_+1.111.json` and `8_*_+1.111.json` in `replay_reference.py` - both should curve into world-left along the same arc.
4. Visually replay `0_*_-1.111.json` and `2_*_-1.111.json` - both should curve into world-right along the same arc.

Only after these pass should the full 210-motion sweep be regenerated and the polynomial refit.
