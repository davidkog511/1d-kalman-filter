# FAQ: How Many States Do I Need in My Kalman Filter?

> **⚠️ APPLIES TO: Assignment 3 - 1D Linear Kalman Filter**

## TL;DR

**One state is enough.** You can use more, but it adds complexity without
benefit for this assignment.

## Why one state works

You have two sensors:
- **IR sensor**: measures distance (noisy)
- **Encoders**: measure velocity (precise)

Use encoder velocity as **input**, not as a state to estimate:

```cpp
// State: x = [distance]  (scalar)
x_pred = x_est - dt * v_encoder;   // Use encoder velocity directly
```

The Kalman filter only needs to estimate what you **don't** measure well.

## What about more states?

**Two states (distance + velocity):**
- More parameters to tune (2×2 covariance matrices)
- More computation
- Same or worse performance (you're ignoring good encoder data)

**When would you use more states?**
- No velocity sensor (e.g., GPS-only tracking)
- Don't trust velocity measurements (wheel slip, uncertain wheel radius)
- Multi-dimensional estimation (X-Y position + heading)

> **Looking ahead:** In **Assignment 4 (EKF)**, you will estimate the robot's
> X-Y position in a 2D world frame. There, multiple states (position, heading)
> become essential.

## Summary

| States | When to use |
|--------|-------------|
| 1 (distance) | **Recommended.** Encoder velocity as input. |
| 2+ | Valid but unnecessary for this assignment. |
