# FAQ: Kalman Filter Loop vs Control Loop

> **⚠️ APPLIES TO: Assignment 3 - 1D Linear Kalman Filter**

## Short Answer

**NO, they are separate!**

- **Kalman filter loop:** Estimates states
- **Control loop:** Uses estimated states to compute motor voltages (or some
  input mapped to motor voltages)

They work together but serve different purposes.

---

## Two Separate Loops in robot.cpp

### Loop 1: State Estimation (Kalman Filter)

```cpp
if(StateEstimationEnabled()) {  // Button 1
    // Correction step
    distance_measurement(0) = getFrontDistance();
    CorrectionUpdate(distance_measurement, _xhat, _nu, _Phat);
}

// ... control happens in between ...

if(StateEstimationEnabled()) {  // Button 1
    // Prediction step
    PredictionUpdate(speedMotor, _xhat, _Phat);
}
```

**What it does:**
- Reads distance sensor
- Updates state estimate `_xhat`
- Updates covariance `_Phat`

**Output:** Estimated distance `_xhat(0)`

---

### Loop 2: Control

```cpp
if(controlEnabled()) {  // Button 0
    // Position controller
    xref(0) = readValue(0)*Meas2State;
    desired_velocity = K * (xref - _xhat);  // Uses Kalman estimate!

    // Velocity controller
    float eA = desired_velocity(0) - spA;
    uA = n0d0*eA + n1d0*errorA[0] + ...;

    setVoltageMotorA(uA);
    setVoltageMotorB(uB);
}
```

**What it does:**
- Reads reference from QRoboticsCenter
- Compares to Kalman estimate `_xhat`
- Computes control signal
- Sends voltage to motors

**Output:** Motor voltages `uA`, `uB`

---

## How They Work Together

```
┌─────────────────┐
│ Distance Sensor │ (noisy measurement)
└────────┬────────┘
         ↓
┌────────────────────┐
│ KALMAN FILTER      │ ← Encoders (velocity input)
│ (State Estimation) │
└────────┬───────────┘
         ↓ _xhat (estimated distance)
┌────────────────────┐
│ CONTROLLER         │ ← Reference (desired distance)
│ (Control Logic)    │
└────────┬───────────┘
         ↓ uA, uB (motor voltages)
┌────────────────────┐
│ Motors             │
└────────────────────┘
```

**Data flow:**
1. Sensors → Kalman filter → Estimated state
2. Estimated state → Controller → Motor commands
3. Motors → Robot motion → Sensors (closes loop)

---

## You Can Enable Them Independently!

### Only Kalman Filter (Button 1)
- Robot doesn't move
- Filter estimates distance
- Good for **testing/debugging** the Kalman filter
- Watch `_xhat` track measurements in QRoboticsCenter

### Only Control (Button 0)
- Control uses Kalman estimate (if available)
- If Kalman disabled, uses uninitialized `_xhat` ❌
- **Don't do this!**

### Both Enabled (Button 0 + Button 1)
- Full system operational 
- Kalman estimates distance
- Controller maintains reference distance
- **This is what you want for the final demo**

---

## In Code Terms

**State estimation (state_estimator.cpp):**
```cpp
void PredictionUpdate(...) {
    _xhat = F*_xhat + B*u;  // Predict state
    _Phat = F*_Phat*(~F) + Q;  // Predict covariance
}

void CorrectionUpdate(...) {
    K = _Phat*(~H)*Inverse(S);  // Kalman gain
    _xhat = _xhat + K*_nu;      // Correct state
    _Phat = (I - K*H)*_Phat;    // Correct covariance
}
```

**Control (robot.cpp):**
```cpp
void Robot::control() {
    // Use estimated state:
    desired_velocity = K * (xref - _xhat);

    // Compute motor command:
    uA = controller(desired_velocity, measured_velocity);
}
```

**Separate functions, separate purposes!**

---
