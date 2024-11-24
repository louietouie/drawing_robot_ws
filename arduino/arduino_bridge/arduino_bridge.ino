#include "can.h"
#include "commands.h"
#include "driver.h"

void setup() {
  setupCAN();
  // motor::setup()
  // setupDriver();

  // enable pin up/downs

}

void loop() {
  if (readCAN()) {
    decipherMessage();
  }
}

void decipherMessage() {
  char command = canMsg.data[0];

  switch (command) {
    case E_STOP:
      break;
    case SET_ACCEL:
      motor::setAcceleration(1000);
      break;
    case SET_SPEED:
      break;
    case SET_POSITION:
      break;  
    case GET_STATE:
      canMsg.can_id = 0x200;
      canMsg.can_dlc = 4;
      canMsg.data[0] = 0x00;
      canMsg.data[1] = 0x00;
      canMsg.data[2] = 0x00;
      canMsg.data[3] = 0x00;
      writeCAN();
      break;
    case ECHO:
      canMsg.can_id = 0x200;
      canMsg.can_dlc = 4;
      canMsg.data[0] = command;
      writeCAN();
      break;
  }

}
