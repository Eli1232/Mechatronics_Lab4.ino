/*
* Arduino Wireless Communication Tutorial
*     Example 1 - Transmitter Code
*                
* by Dejan Nedelkovski, www.HowToMechatronics.com
* 
* Library: TMRh20/RF24, https://github.com/tmrh20/RF24/
*/

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

int pPin = A1;
int iPin = A5;
int dPin = A3;
int p;
int i;
int d;

RF24 radio(7, 8); // CE, CSN

const byte address[6] = "00001";

void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();
}

void loop() {
  p = analogRead(pPin);
  i = analogRead(iPin);
  d = analogRead(dPin);
  String message = String(p) + " " + String(i) + " " + String(d);
  char text[32];
  message.toCharArray(text, sizeof(text));
  radio.write(&text, sizeof(text));
  delay(500);
}