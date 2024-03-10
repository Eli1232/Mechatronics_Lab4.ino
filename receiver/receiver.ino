#include <RH_RF69.h>
#include <SPI.h>
const int INT_pin = 2;
const int CS_pin = 53;
const int RST_pin = 3;
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
  if (yourTransceiver.available()) {
      // Buffer large enough to hold the expected message plus a null terminator
      uint8_t buf[RH_RF69_MAX_MESSAGE_LEN] = {0};
      uint8_t len = sizeof(buf) - 1; // Reserve space for null terminator
      if (yourTransceiver.recv(buf, &len)) {
          // Null-terminate the received message to safely use C string functions
          buf[len] = '\0';
          Serial.print("Received message: ");
          Serial.println((char*)buf); // Assumes message is ASCII text
          // Variables to hold the extracted values
          int p, i, d;
          // sscanf to extract the values from the received message
          if (sscanf((char*)buf, "%d %d %d", &p, &i, &d) == 3) {
              Serial.print("Extracted values: P=");
              Serial.print(p);
              Serial.print(", I=");
              Serial.print(i);
              Serial.print(", D=");
              Serial.println(d);
          } else {
              Serial.println("Error parsing message");
          }
      } else {
          Serial.println("Receive failed");
      }
  }
}

