# ArduPilot MCOUT input

moteus can act as the actuator controller for an underactuated
(swashplateless) propeller flown by an ArduPilot flight controller.
ArduPilot runs the attitude controller and publishes its normalized
control state over a serial port using the AP_MotorControlOutput
("MCOUT") protocol; moteus receives that stream on an AUX UART and
performs the vehicle specific mixing:

* The normalized throttle sets the rotor velocity target (the
  default), or a base (collective) torque, per
  `mcout.throttle_mode`.
* The roll/pitch command vector sets the amplitude and phase of an
  angle-synchronous "cyclic" torque, evaluated at the full control
  rate and phase locked to the measured rotor angle.  Accelerating
  and decelerating within each revolution tilts the thrust vector.
* Yaw commands are ignored; a single underactuated propeller has no
  yaw authority.

The data path is:

```
ArduPilot (attitude control, arming, MCOUT @ up to 400Hz, 921600 baud)
   | UART, one way
   v
AuxPort (uart.mode = mcout) -> McoutReceiver (DMA + frame parser)
   |
   v
FlightControl (mcout.* config: gating, mapping), one command per frame
   |
   v
BldcServo position mode command (velocity + feedforward + cyclic)
   |
   v
servo.cyclic.* ISR term: base + amplitude * sin(2*pi*(harmonic*(angle
   - zero_angle) + phase)), at the PWM control rate
```

## Wiring

Connect the ArduPilot UART transmit to the moteus AUX UART receive,
and connect grounds:

```
ArduPilot UART TX -> moteus AUX UART RX
ArduPilot GND     -> moteus GND
```

The link is one way; nothing is transmitted back on the UART.

On moteus-r4 boards built with `MOTEUS_R4_AUX2_RT`, the AUX2 UART is
available on the SER R/T pads (RX on the "R" pad).

## ArduPilot configuration

```
SERIALx_PROTOCOL = 50    # MotorControlOutput
SERIALx_BAUD     = 921
MCOUT_RATE       = 400
```

See `libraries/AP_MotorControlOutput/README.md` in the ArduPilot tree
for the packet format and protocol details.

## moteus configuration

The motor must already be calibrated, and for a repeatable cyclic
phase the output position must come from an absolute source (the
onboard magnetic encoder with a 1:1 rotor to output ratio is the
normal case).

Example, using AUX2 pins 1 and 2 as the UART at the default 921600
baud:

```
conf set aux2.pins.1.mode 3        # uart
conf set aux2.pins.2.mode 3        # uart
conf set aux2.uart.mode 7          # mcout
conf set aux2.uart.baud_rate 921600

conf set servopos.position_min nan # continuous rotation
conf set servopos.position_max nan

conf set servo.cyclic.harmonic 1
conf set servo.cyclic.zero_angle 0.0

conf set mcout.throttle_mode 1           # velocity (default)
conf set mcout.throttle_max_velocity 40  # rev/s at full throttle
conf set mcout.idle_velocity 5           # ground idle spin
conf set mcout.cyclic_max_torque_Nm 0.8
conf set mcout.cyclic_phase_offset 0.0
conf set mcout.max_torque_Nm 2.0     # default 0 leaves the servo torqueless
conf set mcout.timeout_s 0.1
conf set mcout.watchdog_timeout_s 0.1
conf set mcout.enable 1

conf write
```

`mcout.enable` is the master switch: with it at 0, the stream is
parsed and visible in telemetry but the servo is never commanded.
That is the recommended state for first bring-up.

## Command mapping

With normalized inputs `throttle` (clamped to [0, 1]) and the
sign-adjusted roll/pitch vector `(r, p)` (interpreted in the moteus
reference frame):

```
velocity = max(idle_velocity, throttle * throttle_max_velocity)
base_Nm  = max(idle_torque_Nm, throttle * throttle_max_torque_Nm)
amp_Nm   = min(1, sqrt(r^2 + p^2)) * cyclic_max_torque_Nm
phase    = atan2(r, p) / (2*pi) + cyclic_phase_offset   [cycles]
```

In the default velocity throttle mode this mirrors the bench
scripts: the servo tracks `velocity` through the kd term
(`kp_scale` 0, `kd_scale` 20 by default) with `mcout.accel_limit`
shaping the spool-up ramp, while `base_Nm` is an optional
feedforward assist and the cyclic torque modulates about it.  With
`mcout.throttle_mode` 0 the throttle drives `base_Nm` instead and
the velocity target is the fixed `mcout.velocity`.

A pure +pitch command gives phase 0; a pure +roll command gives phase
0.25.  Physical alignment between the flight controller frame, the
propeller's zero, and rotor phase lag is tuned with
`mcout.cyclic_phase_offset` together with `servo.cyclic.zero_angle`.
Use `mcout.invert_roll` or `mcout.invert_pitch` when one control axis
is mirrored instead of only phase-shifted.

For each newly received frame while engaged (up to 400Hz),
FlightControl sends a position mode command with `position` NaN,
the mapped `velocity`, `feedforward_Nm = base_Nm`, the cyclic
amplitude/phase above, the torque limit from `mcout.max_torque_Nm`
(default 0 — the servo stays torqueless until it is configured), a
watchdog of `mcout.watchdog_timeout_s`, and the configured
`kp_scale`/`kd_scale`.  The servo evaluates the cyclic sine at the
PWM control rate directly from the encoder angle, so the torque
profile stays phase locked between flight controller updates.

## Safety gating

Following the MCOUT receiver requirements, the servo is only driven
when all of the following hold:

* `mcout.enable` is set and an AUX UART is in mcout mode,
* the newest valid packet is fresher than `mcout.timeout_s`,
* the packet flags contain MOTORS_ARMED, SOFT_ARMED, INTERLOCK and
  OUTPUT_ENABLED, and
* EMERGENCY_STOP is clear.

The flag rules are the defaults of `mcout.required_flags` /
`mcout.disallow_flags` and can be adjusted for bench setups.

When any of these stops holding — disarm, interlock, emergency stop,
stale or lost link — a single stop command is sent (motor driver
disengaged, propeller coasts) and the servo is then released.
Because commands stop being sent while disengaged, normal CAN and
diagnostic control remain fully usable on the bench while the vehicle
is disarmed.

If the servo is latched in a fault when ArduPilot commands output,
the arming action is treated as an explicit operator request to clear
the fault: one stop command is sent (which clears it), and engagement
proceeds on the next cycle.

## Telemetry

The `mcout` telemetry channel reports the gating state (`state`:
disabled/no_link/standby/active), link statistics (valid frames, CRC
errors, header errors, missed frames from the sequence counter, frame
age), the decoded roll/pitch/yaw/throttle and flags, and the mapped
base/cyclic torque commands.  The instantaneous applied cyclic torque
is visible as `servo_stats.cyclic_torque_Nm`.

In tview, watch `mcout` while moving the sticks with the vehicle
disarmed (`mcout.enable` can remain 0): the decoded fields should
follow, and `state` should sit at `standby` until armed.

## Bench testing without a flight controller

The cyclic torque support is a plain extension of position mode and
can be exercised over the diagnostic protocol.  The `d pos` command
accepts two new options: `y` (cyclic amplitude in Nm) and `h` (cyclic
phase in cycles).  For example, velocity mode at 20Hz with kd damping
plus a 0.5Nm once-per-rev cyclic:

```
d pos nan 20 2.0 p0 d20 f0.1 y0.5 h0.25
```

This is the same regime the `single_actuator` bench scripts use, with
the sine evaluated on-controller instead of streamed over CAN.

## Building

`utils/remote_build.sh` pushes the current commit to a build server,
checks it out in a dedicated detached git worktree (leaving other
checkouts untouched), runs the host tests, and builds the target
firmware:

```
utils/remote_build.sh [user@host] [/path/to/moteus]
```

Or locally: `tools/bazel test --config=host //:host` and
`tools/bazel build --config=target //:target`.

## Current limitations

* The cyclic amplitude/phase are not yet exposed as CAN registers;
  the MCOUT stream, or the diagnostic `d pos ... y/h` options, are
  the ways to command them.
* The throttle to torque map is affine (idle floor + linear scale);
  no thrust linearization is applied.
* The cyclic phase is not automatically advanced with rotor speed;
  use `mcout.cyclic_phase_offset` to compensate at your operating
  point.
