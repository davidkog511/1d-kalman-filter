#include <Adafruit_BusIO_Register.h>
#include <Adafruit_GenericDevice.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_I2CRegister.h>
#include <Adafruit_SPIDevice.h>
#include <Adafruit_MPU6050.h>

#include "robot.h"

// ============================ BUTTONS ====================================
// Button 0 = STOP (motors off, cancels any test)
// Button 1 = straight-drive test  -> recording 4 (30 s still, then drive)
// Button 2 = rotation test        -> recording 5 (5 s still, slow spin, fast spin)
// Recordings 1-3 (still / front lifted / left lifted): press nothing, just record.
// =========================================================================

// Test settings (adjust if the robot doesn't move or drives too far)
const float STRAIGHT_VOLTAGE   = 6.0;   // V on both motors
const float STRAIGHT_PAUSE_S   = 30.0;  // standing still before driving
const float STRAIGHT_DRIVE_S   = 2.0;   // driving time

const float ROT_SLOW_VOLTAGE   = 4.0;   // V, motors opposite
const float ROT_FAST_VOLTAGE   = 8.0;   // V, motors opposite
const float ROT_PAUSE_S        = 5.0;   // standing still before spinning
const float ROT_PHASE_S        = 8.0;   // duration of slow phase and of fast phase

// The control loop runs every 10 ms (see .ino: 10000 us) -> 100 steps per second
const float STEPS_PER_SECOND = 100.0;

bool Robot::init() {
  MECOtron::init(); // initialize the MECOtron
  i = 0;
  test = 0;
  return true;
}

void Robot::control() {

  // Reset motor voltages and out values (motors are 0 V unless a test sets them)
  for(int k=0; k<12; k++) out[k] = 0.0;
  for(int k=0; k<2; k++) MV[k] = 0.0;

  float t = i / STEPS_PER_SECOND;   // seconds since the test button was pressed

  if (test == 1) {
    // Straight drive: stand still, drive, then stop
    float t_end = STRAIGHT_PAUSE_S + STRAIGHT_DRIVE_S;
    if (t >= STRAIGHT_PAUSE_S && t < t_end) {
      MV[0] = STRAIGHT_VOLTAGE;
      MV[1] = STRAIGHT_VOLTAGE;
    }
    if (t >= t_end + 2.0) {           // 2 s extra standing still, then finished
      test = 0;
      message("Straight test finished.");
    }
  }
  else if (test == 2) {
    // Rotation in place: stand still, slow spin, fast spin, then stop
    float t1 = ROT_PAUSE_S;
    float t2 = t1 + ROT_PHASE_S;
    float t3 = t2 + ROT_PHASE_S;
    if (t >= t1 && t < t2) {
      MV[0] =  ROT_SLOW_VOLTAGE;
      MV[1] = -ROT_SLOW_VOLTAGE;
    } else if (t >= t2 && t < t3) {
      MV[0] =  ROT_FAST_VOLTAGE;
      MV[1] = -ROT_FAST_VOLTAGE;
    }
    if (t >= t3 + 2.0) {              // 2 s extra standing still, then finished
      test = 0;
      message("Rotation test finished.");
    }
  }

  if (test != 0 && i < 30000) i++;    // count time only while a test runs

  // Signals sent to QRoboticsCenter
  out[0]  = getVoltageMotorA();   // applied voltage A [V]
  out[1]  = getVoltageMotorB();   // applied voltage B [V]
  out[2]  = getSpeedMotorA();     // wheel speed A
  out[3]  = getSpeedMotorB();     // wheel speed B
  out[4]  = getPositionMotorA();  // wheel angle A
  out[5]  = getPositionMotorB();  // wheel angle B
  out[6]  = getIMUAccelX();       // accelerometer x
  out[7]  = getIMUAccelY();       // accelerometer y
  out[8]  = getIMUAccelZ();       // accelerometer z
  out[9]  = getIMUGyroX();        // gyro x
  out[10] = getIMUGyroY();        // gyro y
  out[11] = getIMUGyroZ();        // gyro z

  // Send values to QRoboticsCenter and voltages to the motors
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
  message("Straight test started: 30 s still, then driving.");
}

void Robot::button2callback() {
  test = 2;
  i = 0;
  message("Rotation test started: 5 s still, slow spin, fast spin.");
}

void Robot::button3callback() {
  message("Button 3 not used.");
}
