#include <Adafruit_BusIO_Register.h>
#include <Adafruit_GenericDevice.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_I2CRegister.h>
#include <Adafruit_SPIDevice.h>
#include <Adafruit_MPU6050.h>

#include "robot.h"

// ======================= ASSIGNMENT 3 TESTS ==============================
// Button 0 = STOP (motors off, cancels any test)
// Button 1 = drive towards the wall   -> W-recordings (5 s still, then drive)
// Button 2 = voltage steps            -> M-recordings (robot LIFTED, wheels free)
// Static sensor recordings (S): press nothing, just record.
// =========================================================================

// Drive towards the wall (Sharp sensor range ~4-30 cm: start at ~30 cm)
const float WALL_VOLTAGE   = 3.5;   // V on both motors (slow)
const float WALL_PAUSE_S   = 5.0;   // standing still before driving (bias window)
const float WALL_DRIVE_S   = 2.0;   // max driving time (safety: always stops after this)
const float WALL_STOP_M    = 0.08;  // also stop when the sensor reads closer than this [m]

// Voltage steps for the motor model: each level is held STEP_S seconds
const float STEP_S = 1.5;
const int   N_STEPS = 12;
const float STEP_V[N_STEPS] = {0, 3, 6, 9, 6, 3, 0, -3, -6, -9, -3, 0};

const float STEPS_PER_SECOND = 100.0;   // control loop every 10 ms

bool Robot::init() {
  MECOtron::init();
  i = 0;
  test = 0;
  wall_reached = false;
  return true;
}

void Robot::control() {

  for(int k=0; k<12; k++) out[k] = 0.0;
  for(int k=0; k<2; k++) MV[k] = 0.0;

  float t = i / STEPS_PER_SECOND;   // seconds since the test button was pressed

  if (test == 1) {
    // Drive towards the wall: stand still, drive, stop
    float t_end = WALL_PAUSE_S + WALL_DRIVE_S;
    if (t >= WALL_PAUSE_S && t < t_end && !wall_reached) {
      MV[0] = WALL_VOLTAGE;
      MV[1] = WALL_VOLTAGE;
      float d = getFrontDistance();
      if (d > 0.03 && d < WALL_STOP_M) {   // > 3 cm ignores bad readings
        wall_reached = true;
        message("Wall reached, stopping.");
      }
    }
    if (t >= t_end + 3.0) {
      test = 0;
      message("Wall test finished.");
    }
  }
  else if (test == 2) {
    // Voltage steps, same voltage on both motors (robot lifted!)
    int s = (int)(t / STEP_S);
    if (s < N_STEPS) {
      MV[0] = STEP_V[s];
      MV[1] = STEP_V[s];
    } else {
      test = 0;
      message("Step test finished.");
    }
  }

  if (test != 0 && i < 30000) i++;

  // Signals sent to QRoboticsCenter
  out[0]  = getVoltageMotorA();   // applied voltage A [V]
  out[1]  = getVoltageMotorB();   // applied voltage B [V]
  out[2]  = getCurrentMotorA();   // motor current A
  out[3]  = getCurrentMotorB();   // motor current B
  out[4]  = getSpeedMotorA();     // wheel speed A [rad/s]
  out[5]  = getSpeedMotorB();     // wheel speed B [rad/s]
  out[6]  = getPositionMotorA();  // wheel angle A [rad]
  out[7]  = getPositionMotorB();  // wheel angle B [rad]
  out[8]  = getFrontDistance();   // front distance sensor [m]; drop-outs read ~24.78 m
  out[9]  = getIMUAccelX();       // accelerometer x (IMU x points backward)
  out[10] = getIMUAccelY();       // accelerometer y
  out[11] = getIMUGyroZ();        // gyro z (yaw rate)

  for(int k=0; k<12; k++) writeValue(k, out[k]);

  setVoltageMotorA(MV[0]);
  setVoltageMotorB(MV[1]);
}

bool Robot::controlEnabled() {
  return test != 0;
}

void Robot::button0callback() {
  test = 0;
  i = 0;
  message("STOPPED.");
}

void Robot::button1callback() {
  test = 1;
  i = 0;
  wall_reached = false;
  message("Wall test started: 5 s still, then driving to the wall.");
}

void Robot::button2callback() {
  test = 2;
  i = 0;
  message("Step test started: robot must be LIFTED.");
}

void Robot::button3callback() {
  message("Button 3 not used.");
}
