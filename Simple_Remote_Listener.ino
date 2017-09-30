#include <nRF24L01.h>
#include <RF24.h>
#include <RF24_config.h>
#include <SPI.h>
//#include "printf.h"


// Set up nRF24L01 radio on SPI bus plus pins 9 & 10
RF24 radio(9, 10);
const uint64_t pipe = 0xE8E8F0F0E1LL;

uint8_t buttons[3];


void setup(void)
{
  Serial.begin(115200);
//  printf_begin();
  Serial.println("begin");

  radio.begin();
  radio.openReadingPipe(1, pipe);
  radio.startListening();
  Serial.println("Radio Listening");


}

void loop(void)
{
  while (radio.available()) {
    radio.read(buttons, sizeof(buttons));
    Serial.println("Reading from radio");
  }
  if (buttons[0]) {
    Serial.println("Remote Open Button");
  }
  if (buttons[1]){
     Serial.println("Remote Close Button");
  }
}



