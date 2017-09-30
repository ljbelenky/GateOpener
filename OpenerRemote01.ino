#include <SPI.h>
#include "RF24.h"

/* Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins 9 & 10 */
RF24 radio(9, 10);


byte addresses[][6] = {"1Node", "2Node"};

// Used to control whether this node is sending or receiving
bool role = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("radio.begin");
  Serial.println(radio.begin());
  
  radio.setPALevel(RF24_PA_MAX);
  radio.openWritingPipe(addresses[1]);
  radio.openReadingPipe(1, addresses[0]);
}

void loop() {
  Serial.println(F("Now sending"));

  unsigned long message = 1L;                             
  if (!radio.write( &message, sizeof(unsigned long) )) Serial.println(F("failed"));

  delay(10000);
}

