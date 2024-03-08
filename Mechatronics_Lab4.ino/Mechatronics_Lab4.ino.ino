//team 3

#include <Arduino.h>
#include <SPI.h>
#include <Pixy2.h>
#include <DualMAX14870MotorShield.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

DualMAX14870MotorShield motors;
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire);
Pixy2 pixy;

// IMU last angle
double lastAngle = 0; //previous angle
imu::Vector<3> euler;
double threshold_stra = 0.05; //degree threshold for straight PID
double threshold_turn = 0.05; //degree threshold for turning PID

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

double usualSpeed = 200;
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
  motors.enableDrivers(); //for the motors
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  lastAngle = euler.x(); //x angle of the robot in setup
}

void loop() {
  double setpoint;
  // put your main code here, to run repeatedly:

  measureDistance();
  //find gyro angle

  if (distance > 10) { //if high distance, move forward
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
      motors.setM1Speed(usualSpeed); //set M1 to the usual speed
      motors.setM2Speed(-1 * usualSpeed); //make M2 the opposite of M1
      imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER); //get the x angle
      if (abs(euler.x() - lastAngle) > threshold_stra) { //if the current x angle is more than the threshold away from the startup angle, enter straightening PID
        aPID_STRAIGHT(Kpc, Kic, Kdc, lastAngle, usualSpeed); //run straightening PID
      }
      break;

    case  BACKWARD: //TODO: not implemented, implement if necessary
      // //move motors backward, maybe IMU centering
      // motors.setM1Speed(- 1 * usualSpeed);
      // motors.setM2Speed(usualSpeed);
      // imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
      // if (abs(euler.x()-lastAngle) > threshold_stra){
      //   aPID_STRAIGHT(Kpc, Kic, Kdc, lastAngle, usualSpeed);
      // }
      break;

    case TURN_LEFT:
      //move motors until turned 90 degrees left
      euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
      setpoint = euler.x() - 90; //setpoint is 90 degrees left of the current angle
      if (setpoint < 0) { //if the setpoint is negative, rollover by 360
        setpoint = 360 + setpoint;
      }
      aPID_TURNING(Kpt, Kit, Kdt, setpoint); //run turning PID
      lastAngle = setpoint; //after turning is completed, set the current angle (setpoint) as the new "0" angle for the centering PID to follow
      break;

    case TURN_RIGHT:
      //move motors until turned 90 degrees right
      euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
      setpoint = euler.x() + 90;
      if (setpoint > 360) {
        setpoint = setpoint - 360;
      }
      aPID_TURNING(Kpt, Kit, Kdt, setpoint);
      lastAngle = setpoint;
      break;

    case TURN_AROUND:
      //move motors until turned 180 degrees
      euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
      setpoint = euler.x() + 180;
      if (setpoint > 360) {
        setpoint = setpoint - 360;
      }
      aPID_TURNING(Kpt, Kit, Kdt, setpoint);
      lastAngle = setpoint;
      break;

    case PIXY_READ:
      //read color and set the state based on what is read
      pixy.ccc.getBlocks();
      if (pixy.ccc.numBlocks) {
        if (pixy.ccc.blocks[0].m_signature == 1) {
          setCarState(TURN_LEFT);
        } else if (pixy.ccc.blocks[0].m_signature == 2 || pixy.ccc.blocks[0].m_signature == 4) {
          setCarState(TURN_RIGHT);
        } else if (pixy.ccc.blocks[0].m_signature == 3) {
          setCarState(TURN_AROUND);
        }
      }
      break;
  }
}


//PID Functions
//setpoint for whatever PID we are currently using
//input is the process variable (angle or distance), output is the control variable

//Centering- Process variable: angle (IMU). Control variable: motor speed. Purpose: prevents robot from drifting when moving forward.
//Distance- Process variable: distance (ultrasonic). Control variable: motor speed. Purpose: prevents robot from drifting when moving forward
//Turning- Process variable: angle (IMU). Control variable: motor speed. Purpose: Get the robot to turn 90 or 180 degrees
void aPID_TURNING(double Kp, double Ki, double Kd, double setpoint) {
  double integral = 0; //cumulative der
  double derivative; //change in error over change in time
  double old_err = 0; //error from previous time step
  double settleTime = 500;
  unsigned long settleCount = 0; //counter for settling time
  unsigned long oldSettleCount = 0; //counter for settling time
  unsigned long rr = 100; //refresh rate, time it takes between PID running
  unsigned long oldTime = 0; //time the last pid ran
  unsigned long now = millis();  //get current time
  double output; //output value of PID to motor
  double input;
  double speed; //speed output of the motors
  while (1) {
    now = millis();
    if (now - oldTime >= rr) { //if enough time has passed since the last pid call
      euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER); //get the new angle
      input = euler.x();
      oldTime = now; //update oldTime
      double error = setpoint - input; //find error
      if (abs(error) < threshold_turn) { // if euler.x() is pretty close to the setpoint, and it is within the threshold for the settling time
        settleCount = millis(); //update the current settle time counter
        if (settleCount - oldSettleCount >= settleTime) { //if the error has been within the settle threshold for enough time, exit the turning PID
          // stop and exit the function
          motors.setM1Speed(speed);
          motors.setM2Speed(speed);
          return;
        }
      }
      else { //if we ever fall outside the threshold
        oldSettleCount = millis(); //resets the starting counter for settleCount
      }
      integral = integral + (error * (rr / 1000.0)); //calculate integral
      derivative = (error - old_err) / (rr / 1000.0); //calc deriv
      output = (Kp * error) + (Ki * integral) + (Kd * derivative); //calc output
      old_err = error; //updates old error to current error
      speed = constrain(output, -400, 400);
      motors.setM1Speed(speed); //wheels fed same speed, turn in opposite directions
      motors.setM2Speed(speed);
    }
  }
}

// TODO: ISSUE: STUCK IN THE LOOP WITHOUT MEASURING FRONT DISTANCE
void aPID_STRAIGHT(double Kp, double Ki, double Kd, double setpoint, double currSpeed) {
  double integral = 0; //cumulative der
  double derivative; //change in error over change in time
  double old_err = 0; //error from previous time step
  unsigned long rr = 100; //refresh rate, time it takes between PID running
  unsigned long oldTime = 0; //time the last pid ran
  unsigned long now = millis();  //get current time
  double output; //output value of PID to motor
  double speedAdjust;
  double speed; //speed output of the motors
  double input;
  while (1) {
    measureDistance();
    if (distance < 8) {//if distance is low enough leave PID, go to the beginning of loop, and case will become PIXY_READ
      motors.setM1Speed(0);
      motors.setM2Speed(0);
      return;
    }
    now = millis();
    if (now - oldTime >= rr) { //if enough time has passed since the last pid call
      euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
      input = euler.x();
      oldTime = now; //update oldTime
      double error = setpoint - input; //find error
      integral = integral + (error * (rr / 1000.0)); //calculate integral
      derivative = (error - old_err) / (rr / 1000.0); //calc deriv
      output = (Kp * error) + (Ki * integral) + (Kd * derivative); //calc output
      old_err = error; //updates old error to current error
      speedAdjust = constrain(output, -100, 100);
      // TODO: NEED TO CALIBRATE
      // HERE, WANT currSpeed to be positve
      motors.setM1Speed(abs(currSpeed + speedAdjust)); //set motor speed to the normal speed plus some PID adjustment
      motors.setM2Speed(-1 * abs(currSpeed - speedAdjust)); //set the other motor to have the opposite change of the first one, magnifying the effect
    }
  }
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
