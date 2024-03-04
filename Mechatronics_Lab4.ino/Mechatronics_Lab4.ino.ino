//team 3

#include <Arduino.h>
#include <SPI.h>
#include <Pixy2.h>
#include <DualMAX14870MotorShield.h>

DualMAX14870MotorShield motors;
Pixy2 pixy;

//PID constants for centering
double Kpc = 1; //proportional
double Kic = 0; //integral
double Kdc = 0; //derivative

//PID constants for distance
double Kpd = 1; //proportional
double Kid = 0; //integral
double Kdd = 0; //derivative

//PID constants for turning
double Kpt = 1; //proportional
double Kit = 0; //integral
double Kdt = 0; //derivative



//ping sensor initialization
int signal = 49; //digital pin
float distance;
int distThresh = 10;
unsigned long pulseDuration; //USS

//pixy signiatures, with redundant ones
const int SIGNATURE_LEFT = 1;
const int SIGNATURE_RIGHT = 2;
const int SIGNATURE_TURN_AROUND = 3;
const int SIGNATURE_RIGHT_Light = 4;
const int SIGNATURE_RIGHT_Light2 = 5;

//states
enum Action {
  FORWARD,
  BACKWARD,
  TURN_LEFT,
  TURN_RIGHT,
  TURN_AROUND,
  PIXY_READ
};

//initial state
Action volatile currentState = FORWARD;

void setCarState(Action newState) {
  currentState = newState;
}

void setup() {
  Serial.begin(115200); //for the pixy
  Serial.print("Starting...\n"); //for the pixy
  pixy.init(); //for the pixy

}

void loop() {
  // put your main code here, to run repeatedly:

}


//PID Functions
//setpoint for whatever PID we are currently using
//input is the process variable (angle or distance), output is the control variable

//Centering- Process variable: angle (IMU). Control variable: motor speed. Purpose: prevents robot from drifting when moving forward.
//Distance- Process variable: distance (ultrasonic). Control variable: motor speed. Purpose: prevents robot from drifting when moving forward
//Turning- Process variable: angle (IMU). Control variable: motor speed. Purpose: Get the robot to turn 90 or 180 degrees
double aPID(double Kp, double Ki, double Kd, double setpoint, double input) {

  double integral = 0; //cumulative der
  double derivative; //change in error over change in time
  double old_err = 0; //error from previous time step
  unsigned long rr = 100; //refresh rate, time it takes between PID running
  unsigned long oldTime = 0; //time the last pid ran
  unsigned long now = millis();  //get current time

  if (now - oldTime >= rr) { //if enough time has passed since the last pid call
    oldTime = now; //update oldTime
    double error = setpoint - input; //find error
    integral = integral + (error * (rr / 1000.0)); //calculate integral
    derivative = (error - old_err) / (rr / 1000.0); //calc deriv
    output = (Kp * error) + (Ki * integral) + (Kd * derivative); //calc output
    old_err = err; //updates old error to current error
  }
}
