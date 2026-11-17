#ifndef ROBOT_H
#define ROBOT_H

/*
 * ROBOT Class
 *
 * Class incorporating the robot. This class is used to define state machines,
 * control algorithms, sensor readings,...
 * It should be interfaced with the communicator to send data to the world.
 *
 */

#include "mecotron.h" // Include MECOTRON header

class Robot : public MECOtron {
  private:
    // Variables
    int i;        // step counter since the current test started (10 ms per step)
    int test;     // 0 = no test (motors off), 1 = drive towards wall, 2 = voltage steps
    bool wall_reached;  // wall test: stop early when the sensor sees the wall close

    float out[12] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    float MV[2] = {0.0, 0.0};

  public:
    // Constructor
    Robot() { }

    void control();

    // General functions
    bool init();  // Set up the robot

    bool controlEnabled();

    void button0callback();   // STOP
    void button1callback();   // start drive-towards-wall test
    void button2callback();   // start voltage-step test (robot lifted)
    void button3callback();   // not used

};

#endif // ROBOT_H
