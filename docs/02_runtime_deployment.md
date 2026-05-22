# Open Duck Mini Runtime - Technical Deployment Report

**Source repos analysed:**
- `/home/lakieb/Documents/open_duck_mini_research/Open_Duck_Mini_Runtime` (branch: `v2`)
- `/home/lakieb/Documents/open_duck_mini_research/Open_Duck_Mini/docs/`

---

## 1. Deployment Architecture

The robot runs entirely on a **Raspberry Pi Zero 2W** running Raspberry Pi OS Lite (64-bit). There is no offboard compute in the walking loop - the ONNX policy, servo communication, IMU reading, and gamepad handling all execute on the Pi.

**Key hardware components:**
- Raspberry Pi Zero 2W - main compute
- Feetech ST3215 serial bus servos (14 joints) connected via USB-to-serial adapter at `/dev/ttyACM0`
- BNO055 IMU connected via I2C (`board.SCL` / `board.SDA`)
- Two foot contact switches on GPIO pins D22 (left) and D27 (right)
- Xbox One controller connected over Bluetooth
- Optional: antenna servos on GPIO D12/D13 (PWM), LED eyes on D23/D24, projector on D25, I2S speaker

**Software stack:**
- Python 3, virtualenv (`open-duck-mini-runtime`)
- `rustypot==0.1.0` - servo communication library
- `onnxruntime==1.18.1` - policy inference
- `adafruit-circuitpython-bno055==5.4.13` - IMU driver
- `pygame==2.6.0` - Xbox controller and audio
- `pypot` (Pollen Robotics fork, `support-feetech-sts3215` branch) - used only for motor configuration scripts

**USB latency tuning:** A udev rule sets the FTDI serial adapter latency timer to 1 ms, reducing round-trip time for servo bus commands:
```
SUBSYSTEM=="usb-serial", DRIVER=="ftdi_sio", ATTR{latency_timer}="1"
```

---

## 2. Control Loop

**Entry point:** `scripts/v2_rl_walk_mujoco.py`, class `RLWalk.run()`

**Default control frequency:** 50 Hz (configurable via `--control_freq`, default 50)

The main loop is a tight synchronous cycle with soft real-time enforcement:

```
while True:
    1. Poll Xbox controller (non-blocking queue read)
    2. Call get_obs()        - read IMU + servos + feet contacts
    3. Advance imitation phase counter
    4. policy.infer(obs)     - ONNX forward pass
    5. Compute motor targets = init_pos + action * action_scale
    6. Optional: apply LowPassActionFilter
    7. Add head command offsets to head joints
    8. hwi.set_position_all(action_dict) - write all servo targets
    9. time.sleep(max(0, 1/control_freq - elapsed))
```

If a loop iteration exceeds the 20 ms budget, a warning is printed but execution continues. There is no hard preemption or watchdog.

**Command polling rate:** 20 Hz (Xbox controller worker thread, non-blocking queue read in main loop)

**IMU sampling rate:** Matches `control_freq` (50 Hz by default), runs in a daemon thread.

---

## 3. Servo Interface

**Library:** `rustypot` v0.1.0, wrapping the Feetech half-duplex TTL serial protocol.

**Bus speed:** 1,000,000 baud (1 Mbit/s). Baud rate is the speed of serial communication - how many bits of data per second. 1 Mbit/s is fast for serial, but reading all 14 servos still takes a few milliseconds because the bus is half-duplex (can only send OR receive at any moment, not both - so each servo read requires: send request, wait, receive reply).

```python
self.io = rustypot.feetech(usb_port, 1000000)
```

**14 controlled joints with their Feetech bus IDs:**

| Joint | ID | Joint | ID |
|---|---|---|---|
| right_hip_yaw | 10 | left_hip_yaw | 20 |
| right_hip_roll | 11 | left_hip_roll | 21 |
| right_hip_pitch | 12 | left_hip_pitch | 22 |
| right_knee | 13 | left_knee | 23 |
| right_ankle | 14 | left_ankle | 24 |
| neck_pitch | 30 | head_pitch | 31 |
| head_yaw | 32 | head_roll | 33 |

**PID configuration (default, written at startup):**

These are tuning knobs for how the servo reaches its target position. Kp (proportional gain) = how aggressively the servo pushes towards the target. Higher Kp = faster but more likely to overshoot and vibrate. Ki (integral gain) = corrects for persistent errors that Kp alone can't fix. Kd (derivative gain) = acts like a brake, damping oscillations. With Ki=0 and Kd=0, the servo has no braking and no drift correction - it pushes hard towards the target and may oscillate slightly when it arrives.

- Kp = 32 (leg joints), Kp = 8 (head joints: IDs 30-33)
- Ki = 0
- Kd = 0 (configurable at runtime via `--d` flag; default 0)
- Maximum acceleration = 0 (unconstrained)
- Acceleration = 0

The servos operate in **position control mode** (mode 0). All 14 goal positions are written in a single bus transaction per control cycle via `io.write_goal_position(ids, positions)`.

**Turn-on sequence:**
1. Set all joints to low-torque Kp = 2
2. Move to `init_pos` (standing pose)
3. Wait 1 second
4. Raise to full Kp = 32 (legs) / 8 (head)

**Turn-off:** Calls `io.disable_torque(all_ids)`.

**Per-robot zero offset calibration:** Each joint has a software offset stored in `duck_config.json` under `joints_offsets`. The `find_soft_offsets.py` script guides an interactive process to determine per-joint offsets and write them into the config. All position writes and reads apply this offset:
```python
# write:  pos + offset
# read:   pos - offset
```

---

## 4. IMU Integration

**Hardware:** Bosch BNO055 connected via I2C

**Driver:** `adafruit-circuitpython-bno055` v5.4.13

**Two IMU classes exist:**

### `raw_imu.py` (used in `v2_rl_walk_mujoco.py`)
- Mode: `NDOF_MODE` (9-DOF sensor fusion with magnetometer). NDOF mode uses all 9 sensors (accelerometer + gyroscope + magnetometer) and fuses them into an orientation estimate. The simpler IMUPLUS mode uses only 6 (no magnetometer). The magnetometer can be confused by nearby electronics, so the choice matters for reliability.
- Reads: `imu.gyro` (rad/s, 3-axis) and `imu.acceleration` (m/s², 3-axis)
- Output dict: `{"gyro": [x,y,z], "accelero": [x,y,z]}`
- Optional x-axis accelerometer tare on startup (`tare_x()`, commented out by default)

### `imu.py` (quaternion-based, used by `imu_server.py` for debug visualisation)
- Mode: `IMUPLUS_MODE` (6-DOF, no magnetometer)
- Reads quaternion, converts to Euler (xyz), applies pitch bias, converts back to quaternion (scalar-last, as Isaac/MJX expects)
- Handles calibration data save/load from `imu_calib_data.pkl`

**Axis remapping (both classes):** The BNO055 is installed non-standard orientation. A remap is applied in both upside-down and right-way-up configurations. The remap swaps X/Y axes and negates all three in the upside-down case:
```python
axis_remap = (REMAP_Y, REMAP_X, REMAP_Z, NEGATIVE, NEGATIVE, NEGATIVE)  # upside down
axis_remap = (REMAP_Y, REMAP_X, REMAP_Z, NEGATIVE, POSITIVE, POSITIVE)  # right way up
```

**Threading:** IMU sampling runs in a daemon thread (a daemon thread runs in the background and is automatically killed when the main programme exits - the practical meaning is that the IMU and controller are read continuously in the background, and the main control loop just picks up the latest reading whenever it needs it), pushing data into a `Queue(maxsize=1)`. The main control loop reads non-blocking from this queue, falling back to the last known value if no new data is available.

**Observation contribution:** The 6 IMU values (gyro xyz + accelero xyz) occupy the first 6 elements of the observation vector.

---

## 5. Teleoperation

**Interface:** Xbox One controller over Bluetooth, polled via `pygame.joystick` at 20 Hz in a daemon thread.

### Axis mapping

**Walking mode (default):**
| Axis | Pygame index | Command |
|---|---|---|
| Left stick X (inverted) | axis 0 | `lin_vel_y` (lateral, range ±0.2 m/s) |
| Left stick Y (inverted) | axis 1 | `lin_vel_x` (forward, range ±0.15 m/s) |
| Right stick X (inverted) | axis 2 | `ang_vel` (yaw rate, range ±1.0 rad/s) |
| Left trigger | axis 5 | Left antenna position |
| Right trigger | axis 4 | Right antenna position |

**Head control mode (toggle with Y button):**
| Axis | Command |
|---|---|
| Left stick X | `head_yaw` (±0.5 rad) |
| Left stick Y | `head_pitch` (−0.78 to +0.3 rad) |
| Right stick X | `head_roll` (±0.5 rad) |

### Button mapping
| Button | Index | Action |
|---|---|---|
| A | 0 | Pause / unpause policy |
| B | 1 | Play random sound |
| X | 3 | Toggle projector |
| Y | 4 | Toggle head control mode |
| LB | 6 | Sprint mode (phase frequency factor = 1.3) |
| RB | 7 | (registered, no action assigned) |
| D-pad up | hat | Increase phase frequency offset +0.05 |
| D-pad down | hat | Decrease phase frequency offset -0.05 |

The command vector `last_commands` has 7 elements: `[lin_vel_x, lin_vel_y, ang_vel, neck_pitch(unused), head_pitch, head_yaw, head_roll]`.

---

## 6. Policy Inference

**File:** `mini_bdx_runtime/onnx_infer.py`

**Runtime:** `onnxruntime` v1.18.1, CPU execution provider only (no GPU/NPU acceleration).

```python
self.ort_session = onnxruntime.InferenceSession(
    self.onnx_model_path, providers=["CPUExecutionProvider"]
)
```

**Input:** Named tensor `"obs"`, dtype `float32`. In `awd=True` mode (used by the walk script) the input is wrapped in a batch dimension: `{input_name: [inputs]}` and output `outputs[0][0]` is returned.

**Observation vector structure (54 elements total, inferred from `get_obs()`):**

| Slice | Size | Content |
|---|---|---|
| 0:3 | 3 | IMU gyroscope (rad/s) |
| 3:6 | 3 | IMU accelerometer (m/s²) |
| 6:13 | 7 | Commands [vx, vy, ω, neck_pitch, head_pitch, head_yaw, head_roll] |
| 13:27 | 14 | Joint positions relative to init pose (rad) |
| 27:41 | 14 | Joint velocities × 0.05 (rad/s, scaled) |
| 41:55 | 14 | Last action |
| 55:69 | 14 | Last-last action |
| 69:83 | 14 | Last-last-last action |
| 83:97 | 14 | Current motor targets |
| 97:99 | 2 | Foot contact flags [left, right] |
| 99:101 | 2 | Imitation phase [cos(φ), sin(φ)] |

Note: the benchmark in `onnx_infer.py` uses an input size of 54 for a different model variant; the v2 walk observation size works out to approximately 101 elements from the vector concatenation above.

**Output:** Raw action tensor, 14 elements (one per DOF). This is an unbounded float vector; the policy was trained to produce position offsets.

**Latency:** The `onnx_infer.py` benchmark loop measures average inference time at 1000 iterations. No specific figure is hardcoded; performance depends on the Pi Zero 2W's ARM Cortex-A53 cores running onnxruntime's CPU kernels.

---

## 7. Sim-to-Real Gap

### Motor modelling
The sim2real doc (`Open_Duck_Mini/docs/sim2real.md`) describes using **BAM** (Bayesian Actuator Modelling, Rhoban) to identify the Feetech ST3215 motors at 7.4 V. The identified parameters exported to MuJoCo units are: `damping`, `kp`, `frictionloss`, `armature`, `forcerange`. These are baked into the MJCF model used for training, so the policy is trained against a physically-plausible motor model rather than an ideal position servo.

### Observation noise and domain randomisation
These are handled entirely on the training side (Open Duck Playground). The runtime code does not add artificial noise at inference time.

### Action scaling
The raw policy output is scaled by `action_scale` (default 0.25) before being added to the init pose:
```python
motor_targets = init_pos + action * action_scale
```
This keeps commanded deviations small relative to the standing pose, matching the training reward structure.

### Action filtering (optional)
A `LowPassActionFilter` can be applied post-inference. IIR (Infinite Impulse Response) is a type of digital filter. In plain terms: it blends the new command with the previous (already-filtered) command. The 'alpha' value controls how much weight to give each. Higher cutoff frequency = more weight on the new value = less smoothing. Lower cutoff = smoother but slower to respond to genuine changes.

```python
alpha = (1/cutoff_freq) / (1/control_freq + 1/cutoff_freq)
last_action = alpha * last_action + (1 - alpha) * current_action
```
This is off by default (`cutoff_frequency=None`). A stabilisation window of 1 second is applied before the filtered output replaces raw targets, allowing the filter state to settle.

### Velocity scaling
Joint velocities read from hardware are multiplied by 0.05 before being concatenated into the observation. This matches the scaling convention used in the Isaac/MJX training environment.

### Joint reordering
`rl_utils.py` defines `mujoco_to_isaac()` and `isaac_to_mujoco()` reorder functions, accounting for the different joint orderings between MuJoCo (right-first) and IsaacGym (left-first). The v2 walk script uses the HWI joint order directly, which matches the Isaac ordering.

### Action history
Three steps of action history (`last_action`, `last_last_action`, `last_last_last_action`) are fed into the observation, matching the training environment's recurrent observation structure.

### Phase signal
A 2D sinusoidal phase signal `[cos(φ), sin(φ)]` is included in the observation. The phase counter advances by `phase_frequency_factor + phase_frequency_factor_offset` each step. The frequency factor scales from 1.0 to 1.2 linearly with forward velocity (up to 0.15 m/s), and an LB-button sprint mode forces it to 1.3. The phase is backed by `PolyReferenceMotion` (polynomial reference motion coefficients from `polynomial_coefficients.pkl`), which provides the period and step count used to normalise the phase.

### Per-robot joint offsets
Software offsets in `duck_config.json` compensate for mechanical zero-position errors introduced during servo horn installation.

---

## 8. Head Control

The head has 4 DOF controlled by servos 30-33:
- `neck_pitch` (ID 30)
- `head_pitch` (ID 31)
- `head_yaw` (ID 32)
- `head_roll` (ID 33)

Head joints run at lower gain (Kp = 8) than leg joints (Kp = 32).

**During walking:** The policy outputs positions for all 14 joints including the head (indices 5-8 in the motor targets array). These are blended with user head commands:
```python
head_motor_targets = last_commands[3:] + motor_targets[5:9]
motor_targets[5:9] = head_motor_targets
```
The head commands default to `[0, 0, 0, 0]` when head control mode is off, so the policy drives the head unless the user explicitly activates head mode.

**Head puppet mode (`scripts/head_puppet.py`):** A standalone script that bypasses the walking policy entirely. The body is held at init pose, and the Xbox controller drives `head_yaw`, `head_pitch`, and `head_roll` directly within these limits:

| Joint | Range |
|---|---|
| neck_pitch | -20° to +60° |
| head_pitch | -60° to +45° |
| head_yaw | -60° to +60° |
| head_roll | -20° to +20° |

The puppet loop runs at 60 Hz.

**Antenna servos** (not on the Feetech bus): Driven by PWM on GPIO pins D12 (right) and D13 (left) at 50 Hz. Pulse width maps linearly from 1.0 ms to 2.0 ms for the range [-1, +1]. During walking, the right and left trigger axes of the Xbox controller drive each antenna independently.

---

## 9. Safety and Limits

The project is a research/hobbyist robot; there is no comprehensive safety framework. The following mechanisms are present:

**Soft turn-on:** Servos are powered at Kp = 2 during the move to init pose, limiting torque during the potentially large initial motion. Full gain is applied only after the robot is approximately in the standing pose.

**Turn-off:** `hwi.turn_off()` calls `io.disable_torque(all_ids)`, releasing all joints to free-wheel. This is called on `KeyboardInterrupt`.

**Velocity clamping (commented out):** Code exists to clip motor targets to `prev_targets ± max_velocity * dt` (max velocity = 5.24 rad/s ≈ 50 RPM), but it is commented out in the current version.

**No current/torque limits at runtime:** Current limits are not read or enforced in the runtime loop. The BAM-identified `forcerange` in the MJCF model constrains forces during simulation training but has no runtime equivalent.

**No joint angle limits at runtime:** There is no software clipping of commanded positions to safe ranges during the walk loop. The `head_puppet.py` script does apply explicit degree limits, but these are not used in `v2_rl_walk_mujoco.py`.

**No watchdog/fall detection:** There is no IMU-based fall detection or automatic motor disable on loss of balance.

**Pause function:** Button A toggles a `paused` flag that causes the main loop to `time.sleep(0.1)` and skip inference and servo writes, holding the robot at its last commanded position.

**Antenna and peripheral cleanup:** `KeyboardInterrupt` triggers graceful shutdown of antennas (PWM deinit), eyes (GPIO deinit), projector (GPIO deinit), and feet contacts (GPIO deinit).

---

## 10. Code Structure - File-by-File Overview

### `scripts/v2_rl_walk_mujoco.py`
The **main entry point** for RL walking. Contains the `RLWalk` class which orchestrates all subsystems: HWI, IMU, ONNX policy, Xbox controller, reference motion, and optional expression features. The `run()` method is the 50 Hz control loop. This is the file to modify when experimenting with the observation pipeline or action post-processing.

### `mini_bdx_runtime/mini_bdx_runtime/rustypot_position_hwi.py`
**Hardware interface layer.** Wraps `rustypot.feetech()` to provide named-joint position read/write in radians. Handles the servo ID map, per-joint software offsets, and the turn-on/turn-off sequence. The `init_pos` standing pose and `zero_pos` are defined here.

### `mini_bdx_runtime/mini_bdx_runtime/raw_imu.py`
**Primary IMU driver** (used in walking). Runs BNO055 in NDOF_MODE. Outputs raw gyroscope and accelerometer vectors in a daemon thread, making data available non-blocking to the control loop.

### `mini_bdx_runtime/mini_bdx_runtime/imu.py`
**Secondary IMU driver** (used for debug visualisation and IMU server). Runs in IMUPLUS_MODE, reads quaternion, applies pitch bias, and outputs scalar-last quaternion. Used by `imu_server.py` to stream orientation over TCP for remote frame verification.

### `mini_bdx_runtime/mini_bdx_runtime/onnx_infer.py`
**Policy inference wrapper.** Initialises an `onnxruntime.InferenceSession` on CPU and exposes a single `infer(inputs)` method. Handles the batch dimension wrapping required by the `awd=True` model format.

### `mini_bdx_runtime/mini_bdx_runtime/rl_utils.py`
**RL utility functions.** Contains: joint reorder functions (`mujoco_to_isaac`, `isaac_to_mujoco`), `action_to_pd_targets`, `make_action_dict` (converts action array to named joint dict, skipping antennas), `quat_rotate_inverse` (quaternion-based vector rotation), `ActionFilter` (windowed mean), and `LowPassActionFilter` (first-order IIR).

### `mini_bdx_runtime/mini_bdx_runtime/xbox_controller.py`
**Xbox controller interface.** Reads axes and buttons via `pygame.joystick` at `command_freq` Hz in a daemon thread. Manages walking / head-control mode toggle, axis scaling, and deadzone filtering (triggers below 0.1 clamped to 0). Returns a 7-element command vector plus button states.

### `mini_bdx_runtime/mini_bdx_runtime/buttons.py`
**Button debouncing.** Each `Button` object tracks `is_pressed`, `triggered` (rising edge with 200 ms timeout), and `released`. `Buttons` aggregates A, B, X, Y, LB, RB, D-pad up, D-pad down.

### `mini_bdx_runtime/mini_bdx_runtime/poly_reference_motion.py`
**Polynomial reference motion.** Loads `polynomial_coefficients.pkl`, which encodes parametric walking gaits as polynomial coefficients indexed by (dx, dy, dtheta) velocity. Used in the control loop only to obtain the gait period for phase signal normalisation; the actual reference joint positions are not fed into the observation in v2.

### `mini_bdx_runtime/mini_bdx_runtime/duck_config.py`
**Per-robot configuration.** Loads `~/duck_config.json`. Manages: `start_paused`, `imu_upside_down`, `phase_frequency_factor_offset`, optional expression feature flags (eyes, projector, antennas, speaker, microphone, camera), and per-joint software offsets.

### `mini_bdx_runtime/mini_bdx_runtime/feet_contacts.py`
**Foot contact sensor.** Reads two binary digital inputs (active-low with pull-up) on GPIO D22 and D27. Returns `[left_contact, right_contact]` as booleans. No filtering or debouncing is applied.

### `mini_bdx_runtime/mini_bdx_runtime/antennas.py`
**Antenna servo driver.** Controls two hobby servos via 50 Hz PWM on GPIO D12/D13 using `pwmio`. Input range [-1, +1] maps to pulse widths 1.0-2.0 ms (standard servo convention). Left and right signs are inverted relative to each other.

### `mini_bdx_runtime/mini_bdx_runtime/eyes.py`
**LED eye blinker.** Drives two digital output pins (D24/D23) in a daemon thread with randomised blink intervals (1-4 seconds, 100 ms blink duration).

### `mini_bdx_runtime/mini_bdx_runtime/projector.py`
**Projector GPIO toggle.** Simple on/off digital output on D25.

### `mini_bdx_runtime/mini_bdx_runtime/sounds.py`
**Audio playback.** Uses `pygame.mixer` to load `.wav` files from the assets directory at startup. Exposes `play(name)` and `play_random_sound()`. Bundled assets: beeps, happy sounds, lamp and motor sounds.

### `mini_bdx_runtime/mini_bdx_runtime/camera.py`
**Camera capture.** Uses `picamzero` to capture a frame, resize to 512×512, rotate 90°, and base64-encode it. Used for optional vision features (not active in the standard walk loop).

### `scripts/configure_motor.py`
One-time per-motor setup: assigns ID, sets P=32 / I=0 / D=0, clears acceleration limits, moves to zero. Uses `pypot`'s `FeetechSTS3215IO` (higher-level than rustypot, used only for configuration).

### `scripts/configure_all_motors.py`
Iterates all 14 joints and applies the same configuration as `configure_motor.py` in sequence.

### `scripts/find_soft_offsets.py`
Interactive guided procedure for determining per-joint software zero offsets. Disables each joint in turn, prompts the user to move it to the mechanical zero, and records the offset.

### `scripts/head_puppet.py`
Standalone head teleoperation at 60 Hz using Xbox controller, bypassing the walk policy. Useful for testing expression features independently.

### `scripts/imu_server.py` / `scripts/imu_client.py`
TCP socket server/client pair for streaming IMU quaternion data to a remote computer for frame orientation verification during setup.

### `scripts/turn_on.py` / `scripts/turn_off.py`
Minimal scripts to bring the robot to init pose with full torque, or release all joints.

### `scripts/record_data.py` / `scripts/new_record_data.py` / `scripts/plot_recorded_data.py`
Data recording and plotting utilities for debugging and analysis. The `--save_obs` flag in the walk script supports the same use case inline.

### `example_config.json`
Template `duck_config.json` with all fields at their default/zero values. Copy to `~/duck_config.json` and populate with robot-specific offsets and feature flags.

---

## Key Numbers Summary

| Parameter | Value |
|---|---|
| Control loop frequency | 50 Hz (default) |
| Servo bus baud rate | 1,000,000 bps |
| Default leg Kp | 32 |
| Default head Kp | 8 |
| Ki / Kd | 0 / 0 |
| Action scale | 0.25 |
| Velocity observation scale | 0.05× |
| Xbox poll rate | 20 Hz |
| IMU sampling rate | 50 Hz (matches control freq) |
| IMU mode (walk) | NDOF_MODE (9-DOF fusion) |
| Antenna PWM frequency | 50 Hz |
| Forward velocity range | ±0.15 m/s |
| Lateral velocity range | ±0.2 m/s |
| Yaw rate range | ±1.0 rad/s |
| ONNX provider | CPUExecutionProvider |
| Action history depth | 3 steps |
