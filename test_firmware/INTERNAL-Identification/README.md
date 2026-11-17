# INTERNAL-Identification

Use this Arduino project to record the values you need to complete the prediction model.


You can make the following adjustments to robot.cpp:
- Define the voltage control sequence. Currently the script alternates between putting +8V on both motors for 2 seconds, and -8V on the motors for 2 seconds. Adjust this to obtain the profile you want.
- Define which variables are sent to QRoboticsCenter.


Run the software:
1. Flash the project on the Arduino.
2. Open the QRoboticsCenter.
3. Press 'Button 0' to toggle the control on/off.