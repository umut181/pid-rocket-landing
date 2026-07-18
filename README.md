# pid-rocket-landing

A PyBullet simulation of an autonomous rocket booster performing a powered
vertical landing, using cascaded PID control for attitude and horizontal
position, with a full landing-leg touchdown sequence.

**Current Status: complete. the rocket lands successfully and consistently.**

## Overview

The rocket starts airborne with an initial tilt disturbance and must:

1. Null out its attitude error (pitch/roll) using thrust-vector gimbaling
2. Correct horizontal drift (X/Y) back toward the landing pad
3. Descend under a hover-thrust controller with velocity damping
4. Detect ground contact and hold thrust through touchdown, then cut
   engines once contact is sustained

## Control Architecture

Cascaded PID, two loops deep:

```
        Position error (X, Y)
                |
        [ x_pos_pid / y_pos_pid ]
                |
        target Pitch / Roll  (clamped to MAX_TARGET_TILT)
                |
        [ pitch_pid / roll_pid ]
                |
        gimbal angle (X, Y)   (clamped to +/-15 deg)
                |
        Thrust vector applied at nozzle
```

Altitude is handled separately by a hover-thrust + vertical-velocity damper, with a
ground-contact hold-down that cuts thrust after landing.

## Files

| File                 | Purpose                                                       |
|-----------------------|--------------------------------------------------------------|
| `sim.py`              | Main simulation: PID controllers, physics loop, telemetry     
| `rocket.urdf`         | Rocket body (1m cylinder), 4 landing legs,       


A red landing pad marker is created procedurally inside `sim.py`.

## Key Fixes Along the Way

- **Gimbal Y sign bug** — thrust applied below the center of mass couples
  X-force into Y-torque and Y-force into X-torque with *opposite* signs
  (an artifact of the `r x F` cross product). The pitch loop's sign
  correction doesn't carry over to roll; roll needed the opposite sign,
  or it drove itself into positive feedback and tipped over.
- **Position loop Y sign bug** — same root cause one level up: pitch and
  roll couple into world-frame X/Y translation with opposite sign
  conventions, so the Y position loop needed its own sign flip even
  after the gimbal fix.
- **Ground contact height** — recalculated after adding landing legs;
  contact now happens at leg-tip height (~0.72m at the COM), not the
  bare cylinder's base (0.5m).

## Live Telemetry

Position (X, Y, Z) and orientation (roll, pitch) are rendered as a
floating in-sim panel that tracks the rocket, updated in place each
frame via `addUserDebugText` with `replaceItemUniqueId`.

## Possible Extensions
Not needed for this project's goal, but natural next steps if picked
back up later:

- A proper descent trajectory / altitude-scheduling loop (currently just
  hover-thrust + damping, with no planned descent profile)
- Disturbance rejection (wind, sensor noise) to stress-test the gains
