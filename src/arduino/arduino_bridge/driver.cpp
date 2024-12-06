#include "driver.h"
#include "environment_variables.h"

FastAccelStepper *motor::stepper;
FastAccelStepperEngine motor::engine;
TMC2209Stepper motor::uartDriver = TMC2209Stepper(SW_RX, SW_TX, R_SENSE, DRIVER_ADDRESS);
uint32_t motor::_handoverSteps;
uint32_t motor::_jumpstartSteps;
uint32_t motor::_maxSpeedStepsS;
uint32_t motor::_maxAccelStepsSS;
int32_t  motor::_positionStepLimit;
bool     motor::_isPositionLimited;

void motor::setup() {
  engine = FastAccelStepperEngine();
  engine.init();
  stepper = engine.stepperConnectToPin(STEP_STEPPER_PIN);
  while (!stepper) {};

  _handoverSteps = 100;
  _jumpstartSteps = 0;
  _maxSpeedStepsS = 10000;
  _maxAccelStepsSS = 1000;

  _isPositionLimited = false;

  initializeMotorProperties();
  initializeDriverProperties();
}

void motor::stop() {
  stepper->forceStop();
}

int32_t motor::getPosition() {
  return stepper->getCurrentPosition();
}

int32_t motor::getSpeed() {
  return stepper->getCurrentSpeedInMilliHz(false)/1000;
}

void motor::setPosition(int32_t request) {
  int32_t clamped = clampPosition(request);
  stepper->moveTo(clamped, false);
}

void motor::setAcceleration(int32_t request) {
  // NEED TO CHECK IF BOUNDS WORKS FOR THIS
  int32_t clamped = clampAcceleration(request);
  //stepper->moveByAcceleration(clamped, true);
}

void motor::setMaxSpeed(int32_t request) {
  stepper->setSpeedInHz(abs(request));
}

void motor::setMaxPosition(int32_t request) {
  _positionStepLimit = abs(request);
}

void motor::setIsPositionLimited(bool request) {
  _isPositionLimited = request;
}

void motor::checkLimits() {
  if (!_isPositionLimited) return;
  if (isCurrentPositionAcceptable()) return;
  forceStopOnLimit();
}

// _____________________________________________________________________________

bool motor::isPositionExceeded() {
  int32_t currentPosition = getPosition();
  return (currentPosition > _positionStepLimit) || (currentPosition < -_positionStepLimit);
}

bool motor::isCurrentPositionAcceptable() {
  if (!_isPositionLimited) return true;
  if (!isPositionExceeded()) return true;
  if ((getPosition() > 0 && getSpeed() <= 0) || (getPosition() < 0 && getSpeed() >= 0)) return true; // AKA if past limit but returning, OK
  return false;
}

bool motor::forceStopOnLimit() {
  // makes assumption that this is being called because limit was hit, and sets new position to that limit
  int32_t newPosition = getPosition() > 0 ? _positionStepLimit : -_positionStepLimit;
  stepper->forceStopAndNewPosition(newPosition);
}

int32_t motor::clampAcceleration(int32_t request) {
  if (!isPositionExceeded()) return request;
  int32_t currentPosition = getPosition();
  if (request > 0 && (currentPosition < -_positionStepLimit)) return request;
  if (request < 0 && (currentPosition > _positionStepLimit)) return request;
  return 0;
}

int32_t motor::clampPosition(int32_t request) {
  if (!_isPositionLimited) return request;
  if (request > _positionStepLimit) return _positionStepLimit;
  if (request < -_positionStepLimit) return -_positionStepLimit;
  return request;
}

void motor::initializeMotorProperties() {
  stepper->setDirectionPin(DIRECTION_STEPPER_PIN);
  stepper->setEnablePin(ENABLE_STEPPER_PIN);
  stepper->setAutoEnable(true);
  stepper->setLinearAcceleration(_handoverSteps);
  stepper->setSpeedInHz(_maxSpeedStepsS);
  stepper->setAcceleration(_maxAccelStepsSS);
  stepper->setJumpStart(_jumpstartSteps);
}

void motor::initializeDriverProperties() {
  uartDriver.begin();
  uartDriver.rms_current(2000);
  uartDriver.toff(5);
  uartDriver.microsteps(microstep);
  uartDriver.en_spreadCycle(1);
}
