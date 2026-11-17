/*
 * STATE ESTIMATION AND STATE FEEDBACK CONTROL TEMPLATE
 *
 * This is a template to get you started with the implementation of the state estimation and state feedback control
 * on your own cart.
 *
 */
#include <Adafruit_BusIO_Register.h>
#include <Adafruit_GenericDevice.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_I2CRegister.h>
#include <Adafruit_SPIDevice.h>
#include <Adafruit_MPU6050.h>

#include "robot.h"

// Input sanity check (not in the template): the distances typed in ValueOut0 (reference) and ValueOut1 (start)
// are in metres and must lie in the sensor range; anything else is a typo (cm instead of m, a missing value, ...)
static const float D_INPUT_MIN = 0.04f;   // [m]
static const float D_INPUT_MAX = 0.30f;   // [m]
static bool xref_warned = false;          // show the ValueOut0 message once, not every 10 ms
static bool validDistance(float d) { return d >= D_INPUT_MIN && d <= D_INPUT_MAX; }

bool Robot::init() {
  MECOtron::init(); // Initialize the MECOtron

  resetStateEstimator();
  resetController();
  xref(0) = readValue(0)*Meas2State;      // ValueOut0 in metres, converted to the state unit (rad)

  return true;
}

void Robot::control() {

  // Get wheel speeds
  float spA = getSpeedMotorA();
  float spB = getSpeedMotorB();
  float uA = 0.0;
  float uB = 0.0;
  BLA::Matrix<1> desired_velocity; //control signal
  BLA::Matrix<2> speedMotor; //control signal

  desired_velocity.Fill(0); //Initialize matrix with zeros
  speedMotor.Fill(0); //Initialize matrix with zeros
  speedMotor= {spA,spB};
  
  // State estimation
  if(StateEstimationEnabled()) {
    // Correction step
    BLA::Matrix<1> distance_measurement;                                 // define a vector of length 1
    distance_measurement(0) = getFrontDistance();                   // front distance
    CorrectionUpdate(distance_measurement, _xhat, _nu,_Phat);  // do the correction step -> update _xhat, _Phat, _nu

    // Send useful outputs to QRC. After correction: a posteriori estimate
    writeValue(9, _xhat(0));
    writeValue(10, _nu(0));        // innovation [m]; 0 when the reading was rejected by the gate
    writeValue(11, _Phat(0,0));    // a posteriori variance [rad^2]
  }

  
  

  if(controlEnabled() && StateEstimationEnabled()) {   // only do this if controller is enabled (triggered by pushing 'Button 0' in QRoboticsCenter)

    // POSITION CONTROLLER
    xref(0) = readValue(0)*Meas2State;  // use channel 0 in QRoboticsCenter to provide the constant position reference (in metres, converted to rad)
    K(0) = -2.0;                        // state feedback gain K, [CHANGE THIS VALUE!!!] Negative: x = -d/R_WHEEL grows when driving forward
    desired_velocity = K * (_xhat - xref);       // calculate the state feedback signal, (i.e. the input for the velocity controller)
    if (!validDistance(readValue(0))) {           // input check: a reference outside 0.04-0.30 m is ignored, the robot holds
      desired_velocity(0) = 0.0;
      if (!xref_warned) { message("ValueOut0 (reference) outside 0.04-0.30 m: ignored, robot holds."); xref_warned = true; }
    } else {
      xref_warned = false;
    }

    // VELOCITY CONTROLLER [DON'T CHANGE THIS PART!!!]
    float eA = desired_velocity(0)-spA;                 //  calculate the velocity error of motor A (in radians/s)
    float eB = desired_velocity(0)-spB;                 //  calculate the velocity error of motor B (in radians/s)

    // the actual control algorithm
    uA = n0d0*eA + n1d0*errorA[0] + n2d0*errorA[1] - d2d0*controlA[1]; // equation (2), the difference equation
    uB = n0d0*eB + n1d0*errorB[0] + n2d0*errorB[1] - d2d0*controlB[1]; // equation (2), the difference equation

    uA = constrain(uA, -20.0, 20.0);
    uB = constrain(uB, -20.0, 20.0);

    // store the new errors and new control signals in a member variable
    for(int k=0; k<1; k++) {
      errorA[k+1] = errorA[k];        // shift the memorized errors of motor A with 1 sample
      errorB[k+1] = errorB[k];        // shift the memorized errors of motor B with 1 sample
      controlA[k+1] = controlA[k];    // shift the memorized control signals of motor A with 1 sample
      controlB[k+1] = controlB[k];    // shift the memorized control signals of motor B with 1 sample
    }
    errorA[0] = eA; errorB[0] = eB; controlA[0] = uA; controlB[0] = uB;    // append the new values

    // Send wheel speed command
    setVoltageMotorA(uA);
    setVoltageMotorB(uB);
  }
  else                      // do nothing since control is disabled
  {
    desired_velocity(0) = 0;
    setVoltageMotorA(0.0);
    setVoltageMotorB(0.0);
  }

  // State estimation
  if(StateEstimationEnabled()) {
    // Prediction step 
    PredictionUpdate(speedMotor, _xhat,_Phat);                     // do the prediction step -> update _xhat
  }

  // Send useful outputs to QRC
  writeValue(0, uA);
  writeValue(1, uB);
  writeValue(2, getPositionMotorA());
  writeValue(3, getPositionMotorB());
  writeValue(4, spA);
  writeValue(5, spB);
  writeValue(6, xref(0));
  writeValue(7, getFrontDistance());
  writeValue(8, _xhat(0)); // After prediction: a priori estimate for the next sample
}

void Robot::resetController(){
  // Set all errors and control signals in the memory back to 0
  for(int k=0; k<2; k++) {
    errorA[k] = 0.0;
    errorB[k] = 0.0;
    controlA[k] = 0.0;
    controlB[k] = 0.0;
  }
}

void Robot::resetStateEstimator() {
  // Initialize state covariance matrix
  _Phat.Fill(0);      // Initialize the covariance matrix
  _Phat(0,0) = P0_STATE;  // (0.005 m / R_WHEEL)^2: ruler uncertainty of the start distance, see state_estimator.h

  // Initialize state estimate
  float d_start = readValue(1);          // initial distance [m] typed in ValueOut1 (ruler)
  if (validDistance(d_start)) {
    _xhat(0) = d_start*Meas2State;       // converted to rad
  } else {                               // input check: ignore it, start in the middle of the range with a wide P
    _xhat(0) = 0.5f*(D_INPUT_MIN + D_INPUT_MAX)*Meas2State;   // so the first sensor readings are accepted
    _Phat(0,0) = P_RESYNC;
    message("ValueOut1 (start) outside 0.04-0.30 m: ignored, estimator syncs to the sensor.");
  }

  _nu(0) = 0.0;
  ResetGate();        // forget gated readings from a previous run
}

bool Robot::controlEnabled() {
  return _button_states[0];       // The control is enabled if the state of button 0 is true
}

bool Robot::StateEstimationEnabled() {
  return _button_states[1];
}

void Robot::button0callback() {
  if(toggleButton(0)) {           // Switches the state of button 0 and checks if the new state is true
    resetController();
    message("Controller armed; enable the state estimator to run control.");    // Display a message in the status bar of QRoboticsCenter
  }
  else {
    message("Control disabled.");
  }
}

void Robot::button1callback() {
  if(toggleButton(1)){
      resetStateEstimator();            // Reset the state estimator
      message("State estimator reset and enabled.");
  }
  else
  {
    message("State estimator disabled.");
  }
}

void Robot::button2callback() {
    init();                         // Reset the MECOtron and reinitialize the Robot object
    resetController();              // Reset the Controller
    _button_states[0] = 0;
    resetStateEstimator();            // Reset the state estimator
    _button_states[1] = 0;
    message("MECOtron reinitialized.");
}
