# About
Python Object to control a quadruped robot. The robot contains 4 legs with each leg containing two motors.  This object is the main parent object. 

# Program structure
* quadrupedMK3 (Parent object)
  * Robot_state_display

# How to use

1. Please ensure to setup up circuitpython kit from adafruit for controlling Servo motors
2. Then please ensure arrangement of the servo motor address (For this project top motors are numbered in clockwise direction starting from left front limb followed by the bottom motors)
3. Then please set up the degree of freedom for the servo motors to avoid leg collisions by setting up the min and max range values in the "\__init\__" routine
4. Then pass all the motor commands using pass_servo_cmd
5. Developed by Tanumoy Saha; push/pull: @ tanumoysaha5@gmail.com.

# CAD files
https://a360.co/3bknqii
