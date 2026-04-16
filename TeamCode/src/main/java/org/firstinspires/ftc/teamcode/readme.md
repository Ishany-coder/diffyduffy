# Differential Swerve Drive — TeamCode

A two-module differential swerve drive base for FTC, plus a
robot-centric teleop test OpMode.

## Files

| File | Purpose |
| --- | --- |
| `AnalogEncoder.java`     | Wrapper around a REV Through-Bore encoder wired in analog mode. Converts voltage → radians with optional offset / reversal. |
| `Module.java`            | One swerve pod. Runs a PIDF heading loop and mixes steer + drive into two motor powers. |
| `DriveTrain.java`        | Owns both modules. Holds tuning constants, runs the inverse kinematics, manages Lynx hub bulk caching. |
| `DiffySwerveTeleOp.java` | TeleOp driver OpMode. Left stick = translation, right stick X = rotation. Robot-centric. |

## Hardware map names

| Hardware | Config name |
| --- | --- |
| Left module drive motors  | `motor1`, `motor2` |
| Right module drive motors | `motor3`, `motor4` |
| Left steer encoder        | `sensor1` (analog input) |
| Right steer encoder       | `sensor2` (analog input) |

Change these in the `HARDWARE NAMES` block of `DriveTrain.java` if your config differs.

---

## How differential swerve works (mechanically)

Each module has **two motors** driving into a differential. The differential has two
outputs: the **wheel** (rolling) and the **module housing** (steering yaw). The two
motor speeds `ω1` and `ω2` mix into those outputs linearly:

```
steer_rate    ∝ (ω1 + ω2) / 2     ← both motors same direction rotates the pod
wheel_rate    ∝ (ω1 - ω2) / 2     ← motors opposite drives the wheel
```

So if both motors spin the same way the module just *turns*; if they spin opposite
ways the wheel *rolls* without steering; any other combination does both at once.
The code inverts this mix to command the motors:

```
m1 = steer + drive
m2 = steer - drive
```

where `steer` is the PIDF output that closes the heading loop and `drive` is a
feed-forward from the requested wheel speed.

---

## Inverse kinematics (chassis → modules)

Coordinate frame used by `DriveTrain` (matches `Vector2d` in the code):

```
+x  →  robot right
+y  →  robot forward
+ω  →  clockwise when viewed from above
```

Given a desired chassis twist `(vx, vy, ω)` and a module at pose `(rx, ry)`
relative to the center, the module's world-frame velocity is:

```
vx_i = vx + ω * ry_i
vy_i = vy - ω * rx_i
```

From there we extract the module's target heading and wheel speed:

```
θ_i     = atan2(vy_i, vx_i)
speed_i = hypot(vx_i, vy_i)
```

`DriveTrain.update()` runs this for each module then **desaturates**: if any
`speed_i` exceeds the theoretical free-wheel speed, every module's speed is
scaled down by the same factor so the robot's direction of motion is preserved.

### Free-speed cap

```
MAX_INCHES_PER_SEC = (MOTOR_MAX_RPM * DRIVE_GEAR_RATIO / 60) * π * WHEEL_DIAMETER_IN
MAX_RAD_PER_SEC    = MAX_INCHES_PER_SEC / distance(center → module)
```

These are derived from the constants at the top of `DriveTrain.java`. **Tune those
constants to your actual drivetrain** or the kinematics will misreport what the
robot can do.

---

## Module heading control (PIDF)

`Module.update(targetAngle, targetSpeed)` closes the steering loop every cycle:

1. Read current heading from the analog encoder.
2. Compute wrapped error `err = atan2(sin(target − heading), cos(target − heading))`.
   The `atan2(sin, cos)` trick normalizes angle differences to `(-π, π]`, which
   eliminates the "358° vs 2°" problem.
3. **Shortest-path optimization.** If `|err| > 90°` the module would take the long
   way around. Instead we add π to the target, invert the drive command, and let
   the wheel spin backwards — mechanically equivalent, but cuts the steering
   rotation in half.
4. Compute `steer = kP·err + kI·∫err dt + kD·(Δerr/Δt) + kF·sign(err)`. The `kF`
   term is an optional static-friction breakaway.
5. Convert requested wheel speed to a feed-forward power using the free-wheel
   speed from above.
6. Combine `m1 = steer + drive`, `m2 = steer − drive`, then desaturate so
   `|m1|,|m2| ≤ 1`.

### PIDF tuning starting point

Default gains in `Module.java`:

```
kP = 0.6   kI = 0.0   kD = 0.03   kF = 0.0
```

Tune on the real bot: raise `kP` until you see oscillation, then add `kD` to
damp it. Add a touch of `kF` only if a stationary module needs a nudge to break
static friction. `kI` is rarely needed for a swerve pod.

---

## AnalogEncoder

REV Through-Bore encoders in analog mode output `0 .. Vmax` over one full
revolution. The class reads the voltage, normalizes it by `getMaxVoltage()`,
scales to `0..2π`, applies an optional offset (mount angle) and reversal flag,
and wraps to `(-π, π]`:

```java
frac = voltage / maxVoltage
raw  = frac * 2π                // reverse if encoder is mirrored
θ    = wrap(raw - offset)       // offset = module angle when voltage = 0
```

**Zeroing procedure:** park each module pointing straight forward, read
`getAngleInRad()`, call `setOffset(thatValue)`. (The drivetrain exposes each
encoder via `getLeftModule().getEncoder()` so you can wire this into an init
step when you're ready.)

---

## Manual bulk reading

`DriveTrain` grabs every `LynxModule` (both Control and Expansion hubs) and
sets them to `BulkCachingMode.MANUAL`. At the top of every `update()` call
`clearBulkCache()` fires once, so every subsequent encoder / motor read that
loop hits the cache instead of the bus. This keeps the control loop fast
regardless of how many sensors the rest of the robot adds later.

---

## TeleOp controls

`DiffySwerveTeleOp`, robot-centric:

| Input | Meaning |
| --- | --- |
| Left stick X   | Strafe right (+) / left  (−) |
| Left stick Y   | Forward (+, stick is inverted internally) / reverse (−) |
| Right stick X  | Rotate CW (+) / CCW (−) |

Stick values are scaled by `MAX_INCHES_PER_SEC` and `MAX_RAD_PER_SEC` from
`DriveTrain`, then passed through small deadbands. `TRANSLATION_SCALE` and
`ROTATION_SCALE` cap peak output while drivers get used to the chassis.

Telemetry shows commanded velocity and live module headings so you can verify
the modules snap to the expected angles when you push the stick in a direction.

---

## What still needs tuning / wiring

- Real values for `MOTOR_MAX_RPM`, `WHEEL_DIAMETER_IN`, `DRIVE_GEAR_RATIO`,
  and the module positions in `DriveTrain.java`.
- Encoder offsets (`AnalogEncoder.setOffset`) for each module — call once per
  robot after mounting.
- Encoder reversal (`setReversed(true)`) if a module reports decreasing angle
  when you rotate it CCW.
- PIDF gains on the real robot.
- Second-order kinematics (acceleration feed-forward) and wheel-turn-time
  compensation are not yet implemented — the current loop treats each frame
  as an instantaneous target.
