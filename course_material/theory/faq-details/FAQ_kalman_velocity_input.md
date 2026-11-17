# FAQ: Using Measured Velocity in Your Kalman Filter

> **⚠️ APPLIES TO: Assignment 3 - 1D Linear Kalman Filter**

## TL;DR

You have wheel encoders that measure velocity. You can **choose** to use this as
an **input to your prediction model** rather than estimating velocity as a
state. This is simpler and often sufficient for this assignment.

## Three valid approaches

### Approach 1: Estimate velocity

In full state-space form:

```
x[k+1] = F*x[k] + B*u[k]
z[k]   = H*x[k]
```

With velocity as a state variable:

```cpp
// State: x = [distance, velocity]^T
// No control input used (u = 0, B = 0)

F = [[1, -dt],    // d[k+1] = d[k] - dt*v[k]
     [0,  1]]     // v[k+1] = v[k]  (constant velocity model)

H = [[1, 0]]      // Measure distance only

// Note: No Bu term here! Velocity is part of the state, not an input.
// Prediction: x[k+1] = F*x[k]
// Correction: uses distance measurement only
```

**Option:** You could use motor commands (PWM, voltage) as input:

```cpp
// State: x = [distance, velocity]^T
// Input: u = motor command

F = [[1, -dt],
     [0, alpha]]   // alpha < 1: velocity decays (friction)

B = [[0],
     [beta]]       // motor command affects velocity

// Prediction: x[k+1] = F*x[k] + B*u[k]
```

**Valid?** Yes, mathematically correct.

**Optimal?** Not for this assignment, because you're ignoring encoder
measurements that give you velocity directly.

**When IS this approach useful?** Estimating velocity as a state becomes
valuable when:
- You don't trust the encoder → velocity transformation (e.g., uncertain wheel
  radius, slip, or varying terrain)
- The "angle constant" relating encoder ticks to velocity is influenced by
  external factors
- You want to fuse multiple noisy velocity estimates rather than trusting a
  single source

In such cases, the Kalman filter can optimally combine position measurements
with a velocity model to get a better velocity estimate than any single sensor
provides.

### Approach 2: Velocity in state + encoder as measurement

You can keep velocity as a state AND incorporate encoder readings:

```cpp
// State: x = [distance, velocity]^T
// Measurements: both IR distance AND encoder velocity

F = [[1, -dt],
     [0,  1]]

H = [[1, 0],     // IR measures distance
     [0, 1]]     // Encoder measures velocity

z = [d_IR, v_encoder]^T  // Measurement vector

// Note: No Bu term here! The encoder velocity is a measurement,
// not an input. Prediction uses x[k+1] = F*x[k], and the encoder
// reading is incorporated in the update/correction step.
```

**Option:** You could use motor commands (PWM, voltage) as input:

```cpp
// State: x = [distance, velocity]^T
// Input: u = motor command
// Measurements: both IR distance AND encoder velocity

F = [[1, -dt],
     [0, alpha]]   // alpha < 1: velocity decays (friction)

B = [[0],
     [beta]]       // motor command affects velocity

H = [[1, 0],       // IR measures distance
     [0, 1]]       // Encoder measures velocity

// Prediction: x[k+1] = F*x[k] + B*u[k]
// Correction: uses both IR and encoder measurements
```

Alternatively, use encoder velocity as an input to the velocity dynamics:

```cpp
// State: x = [distance, velocity]^T
// Input: u = v_encoder

F = [[1, -dt],
     [0, alpha]]   // alpha < 1: partial trust in previous velocity

B = [[0],
     [1-alpha]]    // Blends encoder input with velocity state

H = [[1, 0]]       // Only IR as measurement
```

**When useful?** When you want to:
- Smooth noisy encoder readings through the state estimate
- Fuse encoder velocity with distance-derived velocity estimates
- Handle occasional encoder dropouts or glitches gracefully

### Approach 3: Use measured velocity as input

**NOTE:**: this approach feels most natural for this particular example.

```cpp
// State: x = [distance] (scalar)
// Input: u = measured velocity (positive = moving toward wall)

F = 1;
B = -dt;     // Distance decreases when velocity is positive
H = 1;       // IR sensor measures distance

// Prediction: d[k+1] = d[k] - dt * v_measured
u = (encoder_left + encoder_right) / 2;  // Average wheel velocity
x_pred = F * x_est + B * u;
```

**Valid?** Yes, mathematically correct.

**Simpler?** Yes, only one state to estimate.

**Practical?** Yes, uses encoder information directly in prediction.

## Understanding "input" in Kalman filters

The term **u** in the prediction equation is traditionally called the "control input",
but it doesn't have to be something you're commanding - it can be any **known quantity**
that affects the state.

```
x[k+1] = F*x[k] + B*u[k]
         ↑   ↑    ↑
         |   |    └── input term (known)
         |   └─────── state (estimated)
         └─────────── dynamics matrix (sometimes also called A)
```

### What can u be?

**u** is any **known information** you want to include in the prediction:

- **Option 1:** u = measured velocity from encoders (what we use here)
- **Option 2:** u = commanded control signal (e.g., motor voltage)
- **Option 3:** u = nothing (set B=0, model autonomous dynamics only)

**Key:** u must be **known/measured** at time k, and the equation must correctly
describe how u affects x[k+1].

### Why use measured velocity as u?

```cpp
// Physics: distance to wall decreases when moving forward
d[k+1] = d[k] - dt * v[k]

// You KNOW v[k] from encoders → put it in u!
u[k] = v_measured[k]
x[k+1] = A*x[k] + B*u[k]  where A = 1, B = -dt
```

This is **not** about commanding the robot - it's about **using known information** to make better predictions.

## Why does the textbook show velocity as a state?

**Textbook scenario:** Tracking a vehicle with GPS only
- GPS → position measurements
- No speedometer → must estimate velocity

**Your scenario:** Robot with encoders AND distance sensor
- Encoders → velocity measurements (precise!)
- IR sensor → distance measurements (noisy)

You have **more sensors** than the textbook. Exploit them!

---

## Implementation guideline

**Approach 3:**
```cpp
// System matrices (scalars for 1D case)
float A = 1.0;
float B = -dt;  // Negative: distance decreases when moving forward
float H = 1.0;

// Read encoders
float v_left = encoder_left.getVelocity();
float v_right = encoder_right.getVelocity();
float v = (v_left + v_right) / 2.0;  // Average velocity (positive = forward)

// Prediction step
x_pred = A * x_est + B * v;      // d[k+1] = d[k] - dt * v
P_pred = A * P_est * A + Q;      // Scalar form of A*P*A^T + Q

// Correction step (measurement from IR sensor)
float z = distance_sensor.read();
// ... standard Kalman update
```

The choice is yours, but this is the **practical engineering choice**.
