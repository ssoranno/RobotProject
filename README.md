# Line-Following Robot Project
Line-Following Robot C code used on a Digilent MX7 development board.<br>
This project was created for Microprocessor Systems class.

## Materials Required
1 PIC32MX7 Development Board<br>
4 IR sensors<br>
1 Digilent Robotics Kit<br>
2 Servo Motors<br>
1 PmodMIC (Digilent pmod microphone)<br>
2 Seven Segment Displays<br>
1 Digilent Pmod 8LD<br>

## MX7 Port Connections
The SSDs are attached to the top rows of ports JA & JB and the top rows of Ports JC & JD<br>
The Pmod 8LD is connected to port JF<br>
The IR sensors are connected to the top rows of port JE<br>
The 2 servo motors is connected to the bottom of port JD<br>

## Program Inputs
Microphone<br>
IR Sensors

## Program Outputs
On-board LEDs<br>
8 LEDs on 8 LD pmod<br>
Seven-segment Displays<br>
R/C Servos<br>

## Robot Functionality
 * Follows a 2” wide printed black line on a white background.
 * Makes turns based on the black line path
 * Uses an analog microphone input into and ADC10 pin to trigger the start of the robot off of two load signals (hand claps) within a 1 second time span. 
 * Uses the CoreTimer, calculate the time it takes to complete the course in tenths of a second (i.e. 100s of milliseconds) and display it on the 4 SSDs such that the numbers are right side up.
 * The Robot then stops at a solid black end line.

## Design Approach
The initial state of the robot program is a diagnostic state that puts the robot in standby mode.  This mode allows the user to set the IR sensor to the right distance from the paper to see the black line and prepare the robot for starting the journey.  When the user presses BTN1 on the MX7 board, the robot program enters state 2.  In this state, the robot is ready to begin its journey.  When the microphone registers two hand claps in this state, the robot begins to move forward and starts its journey.  Also when the robot hears two claps, it uses its core timer to begin capturing the elapsed time in tenths of a second.  While the robot is running, it has six modes to allow it to traverse the track: move forward, sharp left, sharp right, quick right, quick left, and stop.  The forward mode moves the robot forward when the two middle IR sensors see a black line.  The sharp left and right modes make sharp turns by complete stopping one wheel and leaving the other wheel on.  The robot enters this mode when 3 of the IR sensor recognize a black line.  The quick left and right modes make quick turns by spinning the two wheels in opposite directions.  The robot enters this mode when only 2 of the IR sensors recognize a black line.  Finally when the all 4 IR Sensors hit a black line, the robot enter the stop mode which stops the robot, ends the timer, and enters state 1 (diagnostic state). 
