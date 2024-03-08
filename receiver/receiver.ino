#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
int Kp = 0;
int Ki = 0;
int Kd = 0;
RF24 radio(7, 8); // CE, CSN

const byte address[6] = "00001";

void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
}

void loop() {
  if (radio.available()) {
    char text[32] = "";
    radio.read(&text, sizeof(text));
    sscanf(text, "%d %d %d", &Kp, &Ki, &Kd);
    Serial.print(Kp);
    Serial.print(" ");
    Serial.print(Ki);
    Serial.print(" ");
    Serial.println(Kd);
  }
  
}