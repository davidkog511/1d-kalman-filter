# FAQ: Implementing State Feedback Control (robot.cpp)

> **⚠️ APPLIES TO: Assignment 3 - 1D Linear Kalman Filter**

## TL;DR

The assignment focus is **using Kalman estimates for control**, not advanced control theory. You only need simple proportional feedback. The velocity controller is already provided.

## What you need to implement

**One line of code:**
```cpp
desired_velocity = K * (xref - _xhat);
```

That's it! This is called **state feedback** - you use the state estimate `_xhat` to calculate a control action.

## Understanding the two-layer structure

The template has **two controllers**:

### 1. Position Controller (YOU implement this)
```cpp
// Line 46-47 in robot.cpp
K(0) = 1.0;  // State feedback gain (tune this!)
desired_velocity = K * (xref - _xhat);
```

**What it does:**
- Reads reference position: `xref` (from QRoboticsCenter channel 0)
- Compares to Kalman estimate: `_xhat` (distance from wall)
- Calculates error: `xref - _xhat`
- Outputs desired velocity: `K * error`

**Your job:** Tune the gain `K` so the robot reaches the reference without oscillating.

### 2. Velocity Controller (ALREADY PROVIDED)
```cpp
// Lines 50-55 in robot.cpp
float eA = desired_velocity(0) - spA;  // Velocity error
uA = n0d0*eA + n1d0*errorA[0] + ...    // PID-like controller
```

**What it does:**
- Takes `desired_velocity` as input
- Compares to measured velocity from encoders
- Outputs motor voltage `uA`, `uB`

**Your job:** Nothing! It's already implemented.

## Common confusion

### "Do I need to design a PID controller?"
**No!** The velocity controller (lines 50-65) is already a discrete-time controller. Don't touch it.

### "Should I implement advanced control theory?"
**No!** This assignment is about **state estimation**, not control design. Simple proportional gain is enough.

### "What are n0d0, n1d0, d2d0?"
These are pre-calculated controller coefficients (from Assignment 2 motor model). **You don't need to change them.**

### "The robot oscillates, should I redesign the velocity controller?"
**No!** Tune your position controller gain `K` instead. Start with `K = 0.5` and adjust.

## What the assignment is really about

**Focus:** Using your **Kalman filter estimate** `_xhat` to control the robot.

```cpp
// State estimation (Kalman filter)
_xhat(0) = /* estimated distance from wall */

// State feedback (control)
desired_velocity = K * (xref - _xhat);  // Use the estimate!
```

**The point:** Your Kalman filter gives you a good distance estimate even with noisy sensors. You use this estimate for control instead of the raw noisy measurement.

---

**For tuning guidance (K, Q, R parameters), see:** [FAQ_kalman_tuning.md](FAQ_kalman_tuning.md)
