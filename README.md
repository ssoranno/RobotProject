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
Microphone
IR Sensors

## Program Outputs
On-board LEDs
8 LEDs on 8 LD pmod
Seven-segment Displays
R/C Servos

## Robot Functionality
 * Follows a 2” wide printed black line on a white background.
 * Makes turns based on the black line path
 * Uses an analog microphone input into and ADC10 pin to trigger the start of the robot off of two load signals (hand claps) within a 1 second time span. 
 * Uses the CoreTimer, calculate the time it takes to complete the course in tenths of a second (i.e. 100s of milliseconds) and display it on the 4 SSDs such that the numbers are right side up.
 * The Robot then stops at a solid black end line.
