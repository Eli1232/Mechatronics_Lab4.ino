//team 3

#include <SPI.h>
#include <Pixy2.h>
#include <DualMAX14870MotorShield.h>

DualMAX14870MotorShield motors;
Pixy2 pixy;

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
