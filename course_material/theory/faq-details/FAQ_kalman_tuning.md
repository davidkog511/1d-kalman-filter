# FAQ: Tuning Kalman Filter Parameters (Q, R, K)

> **⚠️ APPLIES TO: Assignment 3 - 1D Linear Kalman Filter**

## Three parameters to tune

1. **R** - Measurement noise (sensor uncertainty)
2. **Q** - Process noise (model uncertainty)
3. **K** - Control gain (response speed)

**Tune in this order:** R → Q → K

---

## R: Measurement Noise Covariance

### How to determine R

**Empirical method (recommended):**
1. Place robot at fixed distance (e.g., 0.3m)
2. Record N measurements while stationary
3. Calculate variance: `R = np.var(measurements)`

**Example:** 6072 samples → σ ≈ 3.5mm → **R ≈ 1.25×10⁻⁵ m²**

**Typical range:** R ≈ 10⁻⁶ to 10⁻⁴ m² for IR sensors

### What happens when wrong

| R too small | R correct | R too large |
|-------------|-----------|-------------|
| Noisy estimates | Smooth & accurate | Sluggish response |
| Follows sensor exactly | Balanced | Ignores measurements |

---

## Q: Process Noise Covariance

### How to determine Q

**When using measured velocity as input:**

Your prediction model is: `d[k+1] = d[k] + dt × v_measured[k]`

Q comes from **encoder uncertainty**:

```python
# From encoder datasheet or calibration:
σ_encoder = 0.01    # Encoder velocity uncertainty (rad/s)
r_wheel = 0.03      # Wheel radius (m)
σ_v = σ_encoder * r_wheel  # Linear velocity uncertainty (m/s)

# Propagate through integration:
dt = 0.01           # Sampling time (s)
Q = (dt * σ_v)²     # ≈ 10⁻⁸ m²
```

**If encoder specs unknown:** Start with `Q = 0.01 × R`, then adjust.

**Typical range:** Q ≈ 10⁻⁹ to 10⁻⁵ m²

### The Q/R ratio matters

```
Kalman Gain ∝ sqrt(Q/R)
```

- **Q ≪ R**: Trust model, smooth estimates
- **Q ≈ R**: Balanced
- **Q ≫ R**: Trust measurements, ignore model

---

## K: State Feedback Gain (Not Part of Kalman Filter!)

> **Note:** K is the **controller gain**, not a Kalman filter parameter. You can
> keep the provided default value — tuning K is optional for this assignment.

### How to tune K (optional)

**Procedure:**
1. Start with `K = 0.5`
2. Set reference distance (QRC channel 0)
3. Enable controller, observe response
4. Adjust:
   - **Oscillates?** → Reduce K
   - **Too slow?** → Increase K

**Typical range:** K ≈ 0.3 to 2.0

**Good tuning:** Smooth approach, settles in 1-2s, minimal overshoot

---

## Quick Diagnostic

| Symptom | Likely cause | Fix |
|---------|--------------|-----|
| Estimate very noisy | R too small | Increase R |
| Estimate doesn't track changes | Q too small | Increase Q |
| Control oscillates | K too high | Reduce K |
| Control too slow | K too low | Increase K |

---

## Example Parameter Sets

**Conservative (smooth):**
```cpp
R = 1e-5;  Q = 1e-7;  K = 0.5;
```

**Moderate (balanced):**
```cpp
R = 1e-5;  Q = 1e-6;  K = 1.0;
```

**Aggressive (fast):**
```cpp
R = 1e-5;  Q = 5e-6;  K = 1.5;
```

---

## Remember

- **Measure R** (stationary test), don't guess
- **Estimate Q** from model, tune if needed
- **Tune K** by trial and error
- **One parameter at a time!**
