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

  measureDistance();
  //find gyro angle

  if (distance > 10 { //if high distance, move forward
  setCarState(FORWARD);
  }
  else if (distance <= 10 and distance > 5) { //if correct distance range, pixy read
  setCarState(PIXY_READ);
  }
  else { //otherwise, too close, back up
    setCarState(BACKWARD);
  }

  switch (currentState) {


  case FORWARD:
    //move motors forward, IMU centering

    break;

  case  BACKWARD:
    //move motors backward, maybe IMU centering

    break;

  case TURN_LEFT:
    //move motors until turned 90 degrees left

    break;

  case TURN_RIGHT:
    //move motors until turned 90 degrees right

    break;

  case TURN_AROUND:
    //move motors until turned 180 degrees

    break;

  case PIXY_READ:
    //read color

    break;

}
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
  double output; //output value of PID to motor

  if (now - oldTime >= rr) { //if enough time has passed since the last pid call
    oldTime = now; //update oldTime
    double error = setpoint - input; //find error
    integral = integral + (error * (rr / 1000.0)); //calculate integral
    derivative = (error - old_err) / (rr / 1000.0); //calc deriv
    output = (Kp * error) + (Ki * integral) + (Kd * derivative); //calc output
    old_err = error; //updates old error to current error
  }
  return output;
}


void measureDistance() {

  //set pin as output so we can send a pulse
  pinMode(signal, OUTPUT);
  //set output to LOW
  digitalWrite(signal, LOW);
  delayMicroseconds(5);

  //now send the 5uS pulse out to activate the PING
  digitalWrite(signal, HIGH);
  delayMicroseconds(5);
  digitalWrite(signal, LOW);


  //now we need to change the digital pin
  //to input to read the incoming pulse
  pinMode(signal, INPUT);

  //finally, measure the length of the incoming pulse
  pulseDuration = pulseIn(signal, HIGH);
  distance = (pulseDuration * 0.0001 * 343) / 2; //conversion for the distance
}
