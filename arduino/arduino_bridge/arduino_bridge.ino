#include "can.h"
#include "commands.h"
#include "driver.h"

void setup() {
  Serial.begin(9600);
  setupCAN();
  motor::setup();
  // enable pin up/downs
}

void loop() {
  delay(500);
  if (readCAN()) {
    decipherMessage();
  }
}

void decipherMessage() {
  // python is hex(-7420924 & 0xFFFFFFFF)
  char command = canMsg.data[0];
  int32_t value = ((int32_t) canMsg.data[1]<<24) + ((int32_t) canMsg.data[2]<<16) + ((int32_t) canMsg.data[3]<<8) + ((int32_t) canMsg.data[4]);
  // Serial.println(value);

  switch (command) {
    case E_STOP:
      motor::stop();
      break;
    case SET_ACCEL:
      motor::setAcceleration(value);
      break;
    case SET_SPEED:
      motor::setAcceleration(value);
      break;
    case SET_POSITION:
      motor::setPosition(value);
      break;  
    case GET_STATE:
      canMsg.can_id = 0x200;
      canMsg.can_dlc = 8;
      int32_t position = motor::getPosition();
      int32_t speed = motor::getSpeed();
      canMsg.data[0] = position >> 24;
      canMsg.data[1] = position >> 16;
      canMsg.data[2] = position >> 8;
      canMsg.data[3] = position;
      canMsg.data[4] = speed >> 24;
      canMsg.data[5] = speed >> 16;
      canMsg.data[6] = speed >> 8;
      canMsg.data[7] = speed;
      writeCAN();
      break;
    case ECHO:
      canMsg.can_id = 0x200;
      canMsg.can_dlc = 2;
      canMsg.data[0] = command;
      canMsg.data[1] = canMsg.data[1];
      writeCAN();
      break;
  }

}
