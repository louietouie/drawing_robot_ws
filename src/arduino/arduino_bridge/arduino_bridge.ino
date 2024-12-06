#include "can.h"
#include "commands.h"
#include "driver.h"
#include "environment_variables.h"

const float convertRadianToStep = (1./(2 * pi)) * stepperSPR * microstep;
uint32_t lastHeartbeat;
bool isHeartbeatActive = true;

void setup() {
  Serial.begin(9600);
  setupCAN();
  motor::setup();
  lastHeartbeat = micros();
}

void loop() {
  motor::checkLimits();

  if (readCAN()) {
    decipherMessage();
  }

  if (isHeartbeatActive && heartbeat()) {
    sendState();
  }
}

void decipherMessage() {

  if ((canMsg.can_id & maskCanID) != acceptedCanID) {return;}
  int commandID = canMsg.can_id & ~maskCanID;
  // Serial.println(commandID, HEX);
  float value;
  if (canMsg.can_dlc == 4) {
    memcpy(&value, canMsg.data, sizeof(float));
    if (isnan(value)) {return;}
  }
  // Serial.println(value);

  switch (commandID) {
    case E_STOP:
      motor::stop();
      break;
    case HEARTBEAT:
      isHeartbeatActive = canMsg.data[0];
      break;
    case GET_STATE:
      sendState();
      break;
    case SET_ACCEL:
      motor::setAcceleration(radianToStep(value));
      break;
    case SET_POSITION:
      motor::setPosition(radianToStep(value));
      break;
    case SET_SPEED_LIMIT:
      motor::setMaxSpeed(radianToStep(value));
      break;
    case SET_POSITION_LIMIT:
      motor::setMaxPosition(radianToStep(value));
      break;
    case SET_POSITION_LIM_FLAG:
      motor::setIsPositionLimited(canMsg.data[0]);
      break;
    case ECHO:
      canMsg.can_id = 0x200;
      canMsg.can_dlc = 2;
      canMsg.data[0] = canMsg.data[0];
      canMsg.data[1] = canMsg.data[1];
      writeCAN();
      break;
  }

}

void sendState() {
  canMsg.can_id = heartbeatCanID;
  canMsg.can_dlc = 8;
  float position = radianToStep(motor::getPosition());
  float speed = radianToStep(motor::getSpeed());
  byte posA[4];
  byte speA[4];
  memcpy(posA, &position, sizeof(float));
  memcpy(speA, &speed, sizeof(float));
  canMsg.data[0] = posA[0];
  canMsg.data[1] = posA[1];
  canMsg.data[2] = posA[2];
  canMsg.data[3] = posA[3];
  canMsg.data[4] = speA[0];
  canMsg.data[5] = speA[1];
  canMsg.data[6] = speA[2];
  canMsg.data[7] = speA[3];
  writeCAN();
}

bool heartbeat() {
  if (micros() - lastHeartbeat > heartbeatMicroPeriod) {
    lastHeartbeat = micros();
    return true;
  }
  return false;
}

int32_t radianToStep(float radians) {
  return round(radians * convertRadianToStep);
}

float radianToStep(int32_t radians) {
  return radians / convertRadianToStep;
}