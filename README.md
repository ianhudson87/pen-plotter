# pen-plotter
This is code for controlling a 2-DOF robotic arm pen plotter capable of plotting G-Code instructions.

## Demo

## Implementation

The arm is made up of two servos which are controlled by an Arudino. The Arduino is programmed to move the arm in straight lines. The Raspberry Pi takes the G-Code commands and instructs the Arudino on which direction to move in. The Raspberry Pi also hosts a web page and server that converts a drawn image into G-Code and plots it.

The kinematic equations for moving the arm in straight lines are derived by first solving for the position of the printer head in terms of the motor angles. By solving for the inverse Jacobian of that equation we can calculate the rate at which to rotate the motors to move the printer head in a straight line.

## Credits
