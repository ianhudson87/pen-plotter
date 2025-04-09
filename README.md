# pen-plotter
This is code for controlling a 2-DOF robotic arm pen plotter capable of plotting G-Code instructions.

## Demo

## Implementation

The arm is made up of two servos which are controlled by an Arudino. The Arduino is programmed to move the arm in straight lines. The Raspberry Pi takes the G-Code commands and instructs the Arudino on which direction to move in. The Raspberry Pi also hosts a web page and server that converts a drawn image into G-Code and plots it.

The kinematic equations for moving the arm in straight lines are derived by first solving for the position of the printer head in terms of the motor angles. Then by solving for the inverse Jacobian we can calculate the rate at which to rotate the motors given the desired velocity of the printer head.

## Usage
1. upload the src/Plotter/Plotter.ino file to the arudino.
2. start the worker on the pi by navigate to the src/PlotterController folder and running ```python Worker.py opStatus.txt ./gcodeFiles/current.gcode /dev/ttyUSB0``` or replace /dev/ttyUSB0 with the port the arudino is plugged into.
3. start the web server on the pi by navigating to the src/WebServer directory and running ```python server.py ../PlotterController/opStatus.txt ../PlotterController/gcodeFiles/current.gcode``` and worker on the pi by runner these commands
4. navigate to

## Credits
