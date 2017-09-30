//#include <nRF24L01.h>
#include "RF24.h"
//#include <RF24_config.h>
#include <SPI.h>
#include "printf.h"

int PowerPin = 2;

//INPUTS
int OpenButton = 7;
int CloseButton = 6;
byte command = 0;

//OUTPTUS
int Red = 5;
int Green = 4;
int Blue = 3;

// Set up nRF24L01 radio on SPI bus plus pins 9 & 10
RF24 radio(9, 10);
byte addresses[][6] = {"1Node", "2Node"};

void setup(void)
{
  pinMode(PowerPin, OUTPUT);
  digitalWrite(PowerPin, HIGH);
  Serial.begin(115200);
  printf_begin();
  Serial.println("begin");

  pinMode(OpenButton, INPUT);
  pinMode(CloseButton, INPUT);
  
  command = digitalRead(OpenButton)+2*digitalRead(CloseButton);
  Serial.println(command);

  pinMode(Red, OUTPUT);
  pinMode(Green, OUTPUT);
  pinMode (Blue, OUTPUT);

  digitalWrite(Green, command==1);
  digitalWrite(Red, command==2);
  digitalWrite(Blue, command==3);

  radio.begin();
  radio.setPayloadSize(sizeof(command));
  Serial.println("radio.begin()");
  Serial.println(radio.getPayloadSize());
  radio.openWritingPipe(addresses[1]);
  radio.openReadingPipe(1, addresses[0]);

  radio.stopListening();
  int count = 10;
  while (count) {
    Serial.println(count);
    radio.openWritingPipe(addresses[1]);
    radio.write(&command, sizeof(command));
    delay(37);
    count--;
  }
}

void loop()
{
  digitalWrite(PowerPin, LOW);
}



