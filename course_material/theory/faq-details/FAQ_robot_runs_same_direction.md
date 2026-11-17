# FAQ: My Robot Keeps Running in the Same Direction

> **⚠️ APPLIES TO: Assignment 3 & 4**

## TL;DR

**Did you press both Button 1 AND Button 2?** You need both the estimator AND
the control loop running.

## The problem

Your robot drives in one direction and never stops or turns around, even though
your Kalman filter and controller code looks correct.

## The solution

**Check the README!** You need to activate **two** things:

1. **Button 1**: Starts the **state estimator** (Kalman filter)
2. **Button 2**: Starts the **control loop**

Both must be running for the robot to respond to distance estimates.

## Why this happens

- If only the **estimator** is running: the robot has no control commands
- If only the **controller** is running: the controller receives no valid state
  estimates (or initial/default values)

Either case results in the robot not behaving as expected.
