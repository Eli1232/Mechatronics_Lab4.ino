//team 3
#include <RH_RF69.h>
#include <SPI.h>
const int INT_pin = 2;
const int CS_pin = 53;
const int RST_pin = 3;
#include <Arduino.h>
#include <SPI.h>
#include <Pixy2.h>
#include <DualMAX14870MotorShield.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
// Since we are using workstation 4
const int freqInMHz = 950;
// Create an instance of the transceiver object
RH_RF69 yourTransceiver(CS_pin, INT_pin);

DualMAX14870MotorShield motors;
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire);
Pixy2 pixy;

// IMU last angle
double firstAngle = 0;
double lastAngle = 0; //previous angle (updated in setup and after turns
imu::Vector<3> euler;
double threshold_stra = 0.05; //degree threshold for straight PID
double threshold_turn = 0.05; //degree threshold for turning PID

//PID constants for centering
double Kpc = 14; //proportional
double Kic = 8; //integral
double Kdc = 2.1; //derivative

//PID constants for distance
double Kpd = 1; //proportional
double Kid = 0; //integral
double Kdd = 0; //derivative

//PID constants for turning
double Kpt = 5; //proportional
double Kit = 0; //integral
double Kdt = 0; //derivative

int state;


//ping sensor initialization
int signal = 27; //digital pin
float distance;
int distThresh = 10;
unsigned long pulseDuration; //USS

//pixy signiatures, with redundant ones
const int SIGNATURE_LEFT = 1;
const int SIGNATURE_RIGHT = 2;
const int SIGNATURE_TURN_AROUND = 3;
const int SIGNATURE_RIGHT_Light = 4;
const int SIGNATURE_RIGHT_Light2 = 5;

double usualSpeed = -150;

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
  euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  firstAngle = euler.x(); //x angle of the robot in setup
  lastAngle = euler.x(); //x angle of the robot in setup

  // Set up the pin that is connected to the transceiver’s RST pin to be an output,
  // and set the voltage to be low
  pinMode(RST_pin, OUTPUT);
  // Manually reset the transceiver by setting the RST pin to low for 100ms, high
  // for 10ms, then back to low for 10ms.
  digitalWrite (RST_pin, LOW);
  delay(100);
  digitalWrite (RST_pin, HIGH);
  delay(10);
  digitalWrite (RST_pin, LOW);
  // Initialize the transceiver
  if (yourTransceiver.init()) {
    Serial.println("Initialization succeeds");
  } else {
    Serial.println("Initialization failed");
  }
  // Set the transceiver's frequency
  if (yourTransceiver.setFrequency(freqInMHz)) {
    Serial.println("Set frequency successfully");
  } else {
    Serial.println("Failed in setting frequency");
  }
  // Set the transceiver's power level
  yourTransceiver.setTxPower(16, true);
  state = 1;
}

void loop() {
  if (yourTransceiver.available()) {
    // Buffer large enough to hold the expected message plus a null terminator
    uint8_t buf[RH_RF69_MAX_MESSAGE_LEN] = {0};
    uint8_t len = sizeof(buf) - 1; // Reserve space for null terminator
    if (yourTransceiver.recv(buf, &len)) {
      // Null-terminate the received message to safely use C string functions
      buf[len] = '\0';
      // Serial.print("Received message: ");
      // Serial.println((char*)buf); // Assumes message is ASCII text
      // Variables to hold the extracted values
      int p, i, d;
      // sscanf to extract the values from the received message
      if (sscanf((char*)buf, "%d %d %d", &p, &i, &d) == 3) {
        Kpt = p / 10.0;
        Kit = i / 10.0;
        Kdt = d / 10.0;
        // Serial.print("Extracted values: P=");
        // Serial.print(p);
        // Serial.print(", I=");
        // Serial.print(i);
        // Serial.print(", D=");
        // Serial.println(d);
      } else {
        Serial.println("Error parsing message");
      }
    } else {
      Serial.println("Receive failed");
    }
  }
  Serial.println(state);
  double setpoint;
  // put your main code here, to run repeatedly:

  // distance = getAverageDistance(5);
  //find gyro angle

  switch (state) {
    case 1:
      Serial.println("Forward");
      //move motors forward, IMU centering
      motors.setM1Speed(usualSpeed); //set M1 to the usual speed
      motors.setM2Speed(-1 * usualSpeed); //make M2 the opposite of M1
      euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER); //get the x angle
      if (abs(euler.x() - lastAngle) > threshold_stra) { //if the current x angle is more than the threshold away from the startup angle, enter straightening PID
        aPID_STRAIGHT(Kpc, Kic, Kdc, lastAngle, usualSpeed); //run straightening PID
        Serial.println("Freedom!!!!!!!!!!!!!!!!!!!!");
        state = 6;
        motors.setM1Speed(0); //set M1 to the usual speed
        motors.setM2Speed(0); //make M2 the opposite of M1
        break;
      }
      break;

    case 2: //Move backwards slowly, no PID, until distance is high enough, then it goes to PIXY_READ
      Serial.println("Backward");
      motors.setM1Speed(- 1 * (usualSpeed / 2));
      motors.setM2Speed(usualSpeed / 2);
      state = 1;
      break;

    case 3:
      //move motors until turned 90 degrees left
      Serial.println("Left");
      setpoint = lastAngle - 90; //setpoint is 90 degrees left of the previous angle (based on the first global angle)
      if (setpoint < 0) { //if the setpoint is negative, rollover by 360
        setpoint = 360 + setpoint;
      }
      lastAngle = setpoint; //set lastangle to the one set in setpoint
      aPID_TURNING(Kpt, Kit, Kdt, setpoint); //run turning PID
      state = 1;
      break;

    case 4:
      //move motors until turned 90 degrees right
      Serial.println("Right");
      setpoint = lastAngle + 90;
      if (setpoint > 360) {
        setpoint = setpoint - 360;
      }
      lastAngle = setpoint;
      aPID_TURNING(Kpt, Kit, Kdt, setpoint);
      state = 1;
      break;

    case 5:
      //move motors until turned 180 degrees
      Serial.println("Around");
      setpoint = lastAngle + 180;
      if (setpoint > 360) {
        setpoint = setpoint - 360;
      }
      lastAngle = setpoint;
      aPID_TURNING(Kpt, Kit, Kdt, setpoint);
      state = 1;
      break;

    case 6:
      Serial.println("Pixy Read");
      delay(4000);
      //read color and set the state based on what is read
      pixy.ccc.getBlocks();
      if (pixy.ccc.numBlocks) {
        if (pixy.ccc.blocks[0].m_signature == 1) {
          state = 3;
          Serial.println("Turn left");
        } else if (pixy.ccc.blocks[0].m_signature == 2 || pixy.ccc.blocks[0].m_signature == 4) {
          state = 4;
          Serial.println("Turn right");
        } else if (pixy.ccc.blocks[0].m_signature == 3) {
          state = 5;
          Serial.println("Turn around");
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
      if (error < 0) {
        if (error > -180) {
          error = error;
        } else {
          error = error + 360;
        }
      }
      else {
        if (error < 180) {
          error = error;
        } else {
          error = 360 - error;
        }
      }
      Serial.print(setpoint);
      Serial.print(" ");
      Serial.print(input);
      Serial.print(" ");
      Serial.print(error);
      Serial.print(" ");
      if (abs(error) < threshold_turn) { // if euler.x() is pretty close to the setpoint, and it is within the threshold for the settling time
        settleCount = millis(); //update the current settle time counter
        if (settleCount - oldSettleCount >= settleTime) { //if the error has been within the settle threshold for enough time, exit the turning PID
          // stop and exit the function
          motors.setM1Speed(0);
          motors.setM2Speed(0);
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
      Serial.println(speed);
      motors.setM1Speed(-1 * speed); //wheels fed same speed, turn in opposite directions
      motors.setM2Speed(-1 * speed);
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
    distance = getAverageDistance(5);
    Serial.print("distance: ");
    Serial.println(distance);
    if (distance < 8) {//if distance is low enough leave PID, go to the beginning of loop, and case will become PIXY_READ
      motors.setM1Speed(0);
      motors.setM2Speed(0);
      Serial.println("Exit");
      return;
    }
    now = millis();
    if (now - oldTime >= rr) { //if enough time has passed since the last pid call
      euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
      input = euler.x();
      oldTime = now; //update oldTime
      double error = setpoint - input; //find error
      if (error < 0) {
        if (error > -180) {
          error = error;
        } else {
          error = error + 360;
        }
      }
      else {
        if (error < 180) {
          error = error;
        } else {
          error = 360 - error;
        }
      }
      integral = integral + (error * (rr / 1000.0)); //calculate integral
      derivative = (error - old_err) / (rr / 1000.0); //calc deriv
      output = (Kp * error) + (Ki * integral) + (Kd * derivative); //calc output
      old_err = error; //updates old error to current error
      speedAdjust = constrain(output, -100, 100);
      // TODO: NEED TO CALIBRATE
      // HERE, WANT currSpeed to be positve
      // Serial.print(setpoint);
      // Serial.print(" ");
      // Serial.print(input);
      // Serial.print(" ");
      // Serial.print(error);
      // Serial.print(" ");
      // Serial.println(speedAdjust);
      motors.setM1Speed(currSpeed - speedAdjust); //set motor speed to the normal speed plus some PID adjustment
      motors.setM2Speed(- 1 * currSpeed - speedAdjust); //set the other motor to have the opposite change of the first one, magnifying the effect
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

float getAverageDistance(int numReadings) {
  float sumDistance = 0;
  for (int i = 0; i < numReadings; ++i) {
    measureDistance();  // Assuming this function updates 'distance'
    sumDistance += distance;
    delay(10);  // Add a small delay between readings
  }
  return sumDistance / numReadings;
}
