#include <RH_RF69.h>
#include <SPI.h>
const int INT_pin = 2;
const int CS_pin = 4;
const int RST_pin = 5;
int pPin = A0;
int iPin = A2;
int dPin = A1;
int p;
int i;
int d;
// Since we are using workstation 4
const int freqInMHz = 950;
// Create an instance of the transceiver object
RH_RF69 yourTransceiver(CS_pin, INT_pin);

void setup() {
  Serial.begin(9600);
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
  if(yourTransceiver.init()){
    Serial.println("Initialization succeeds");
  } else{
    Serial.println("Initialization failed");
  }
  // Set the transceiver's frequency
  if(yourTransceiver.setFrequency(freqInMHz)){
    Serial.println("Set frequency successfully");
  } else{
    Serial.println("Failed in setting frequency");
  }
  // Set the transceiver's power level
  yourTransceiver.setTxPower(16, true);
}


void loop() {
  p = analogRead(pPin);
  i = analogRead(iPin);
  d = analogRead(dPin);
  p = map(p, 0, 1023, 0, 50);
  i = map(i, 0, 1023, 0, 50);
  d = map(d, 0, 1023, 0, 50);
  String message = String(p/10.0) + " " + String(i/10.0) + " " + String(d/10.0);
  char text[32];
  message.toCharArray(text, sizeof(text));
  
  // Send a message
  Serial.println("Transmitting: " + String(text));
  yourTransceiver.send((uint8_t *)text, strlen(text));
  yourTransceiver.waitPacketSent();
  
  // Delay between messages to avoid continuous transmission
  delay(500); // Adjust this delay as necessary for your application.
}
