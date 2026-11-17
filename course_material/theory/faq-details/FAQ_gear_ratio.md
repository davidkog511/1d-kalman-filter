# FAQ: Gear Ratio and Encoder Readings

> **Note (Nov 2025):** This FAQ was updated. The previous version incorrectly
> advised multiplying by the gear ratio in Python - **do not do this**, it
> would give values ~28× too large. The firmware already handles conversion.
> The encoder _is_ on the motor shaft (see `mecotron.h`), but its already being
> converted for you.

> **Scope:** This FAQ is relevant for **Assignment 2 (Identification)** only.
> For Milestone 2 and beyond, you work directly with the calibrated sensor
> values and this conversion is handled internally.

---

## Where is the encoder mounted?

The encoder is mounted on the **motor shaft**, not on the wheel. However, **the
firmware already converts motor shaft readings to wheel-level values**.

## What is the gear ratio?

The gear ratio **n = 256/9 ≈ 28.44** is provided in the assignment APPENDIX.
The motor shaft rotates 28.44 times for every wheel rotation.

## Do I need to apply the gear ratio in Python?

**No!** The Arduino firmware already applies the conversion factor `ENC_TO_RAD`
which includes the gear ratio. From `mecotron.h`:

```c
#define ENC_TO_RAD  M_PI / 28.0 / 16.0 / 2.0  // 1792 enc per 2*pi (wheel)
```

This means the logged GPIO data is already in **wheel-level units**:

| Channel   | Contains              | Units       |
|-----------|-----------------------|-------------|
| ValueIn0  | Wheel position A      | rad (wheel) |
| ValueIn1  | Wheel position B      | rad (wheel) |
| ValueIn6  | Wheel angular speed A | rad/s (wheel) |
| ValueIn7  | Wheel angular speed B | rad/s (wheel) |

Simply use the data directly:

```python
# Data is ALREADY in wheel-level units - no conversion needed!
theta_A = df['ValueIn0']   # Wheel position (rad)
theta_B = df['ValueIn1']
omega_A = df['ValueIn6']   # Wheel angular velocity (rad/s)
omega_B = df['ValueIn7']
```

---

## How do I verify my data is correct?

**Sanity checks:**

1. **Velocity magnitude:** Should be ~10-20 rad/s for typical motor commands
2. **Linear velocity:** v = omega * r ≈ 0.3-0.7 m/s (with r = 0.0325 m)
3. **Position:** After 2 seconds at 8V, expect ~20-40 rad of wheel rotation

If your velocities are ~28x too large or too small, you may have accidentally
applied (or failed to apply) a gear ratio conversion somewhere.

## When IS the gear ratio needed?

The gear ratio is needed when converting between **motor-level** and
**wheel-level** quantities in your model equations. For example:

- Motor inertia J_motor vs. effective wheel inertia J_wheel
- Motor torque vs. wheel torque

Refer to the assignment APPENDIX for the correct model formulation.
