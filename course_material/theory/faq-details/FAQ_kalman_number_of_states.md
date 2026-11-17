# FAQ: How Many States Do I Need in My Kalman Filter?

> **⚠️ APPLIES TO: Assignment 3 - 1D Linear Kalman Filter**

## TL;DR

**One state is enough** for this assignment. You can use more if you want, but
it's not required and adds complexity.

## The short answer

For Assignment 3, you need to estimate **distance to the wall**. That's one
quantity → one state is sufficient:

```cpp
// State: x = [distance]  (scalar)
```

## Why one state works

You have two sensors:
- **IR sensor**: measures distance (noisy)
- **Encoders**: measure velocity (relatively precise)

With encoder velocity as an **input** to your prediction model, you don't need
to estimate velocity — you already know it! The Kalman filter only needs to
estimate what you **don't** directly measure well.

```cpp
// Prediction: d[k+1] = d[k] - dt * v_encoder
// Measurement: z = d_IR

x_pred = x_est - dt * v_encoder;   // Use encoder velocity directly
// ... Kalman update with IR measurement
```

## When would you use more states?

**Two states (distance + velocity)** makes sense when:
- You don't have velocity measurements (no encoders)
- You don't trust your velocity measurements (wheel slip, uncertain wheel
  radius)
- You want to fuse multiple velocity sources optimally

**Three or more states** are needed when estimating multi-dimensional quantities:
- **X-Y position + heading** (e.g., 2D localization)
- Acceleration (if you want to model changing velocity)
- Sensor biases (if sensors have unknown offsets)

For Assignment 3, more than one state is overkill.

### How about assignment 4

> **Looking ahead:** In **Assignment 4 (EKF)**, you will estimate the robot's
> X-Y position in a 2D world frame. There, multiple states (position, heading,
> possibly velocity) become essential.

## Common misconception

> "The textbook uses position and velocity, so I should too"

Textbook examples often track vehicles with **GPS only**. They must estimate
velocity because they can't measure it. You have encoders — use them!

**Bottom line:** One state is enough. The choice is yours, but simpler is often
better.
