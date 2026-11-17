# FAQ

Frequently Asked Questions for Mobile Robotics Course

---

## General

| Question | Description | Link |
|----------|-------------|------|
| | | |

---

## Assignment 2: Motor Parameter Identification

| Question | Description | Link |
|----------|-------------|------|
| How do I apply the gear ratio correctly? | Understanding and applying n=256/9 to encoder readings for correct parameter identification | [Details](faq-details/FAQ_gear_ratio.md) |

---

## Assignment 3: 1D Linear Kalman Filter

| Question | Description | Link |
|----------|-------------|------|
| Should I estimate velocity or use it as input? | Both approaches are valid; using measured velocity as input is simpler and performs better | [Details](faq-details/FAQ_kalman_velocity_input.md) |
| How many states should I use? | Anything you want, but only 1D required; more states = unnecessary complexity | [Details](faq-details/FAQ_kalman_state_dimension.md) |
| Is Kalman filter loop same as control loop? | No - estimation (Kalman) and control are separate; controller uses estimated states | [Details](faq-details/FAQ_estimation_vs_control_loop.md) |
| How do I implement control in robot.cpp? | Simple state feedback is sufficient; focus is on using Kalman estimates, not advanced control theory | [Details](faq-details/FAQ_simple_state_feedback.md) |
| How do I tune Q, R, and K parameters? | Systematic approach: measure R empirically, estimate Q from model, tune K by trial and error | [Details](faq-details/FAQ_kalman_tuning.md) |

---

## Assignment 4

| Question | Description | Link |
|----------|-------------|------|
| | | |

---

## Troubleshooting

| Question | Description | Link |
|----------|-------------|------|
| My robot keeps running in the same direction | Did you press both Button 1 AND Button 2? Both estimator and control loop must be running | [Details](faq-details/FAQ_robot_runs_same_direction.md) |

---

## Contributing

Have a question that's not answered here? Contact the course instructors or create an issue.
