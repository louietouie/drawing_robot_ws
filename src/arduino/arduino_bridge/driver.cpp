#include "driver.h"

FastAccelStepper *motor::stepper;
FastAccelStepperEngine motor::engine;
TMC2209Stepper motor::uartDriver = TMC2209Stepper(SW_RX, SW_TX, R_SENSE, DRIVER_ADDRESS);
uint32_t motor::_handoverSteps;
uint32_t motor::_jumpstartSteps;
uint32_t motor::_maxSpeedStepsS;
uint32_t motor::_maxAccelStepsSS;

void motor::setup() {
  engine = FastAccelStepperEngine();
  engine.init();
  stepper = engine.stepperConnectToPin(STEP_STEPPER_PIN);
  while (!stepper) {};

  _handoverSteps = 100;
  _jumpstartSteps = 0;
  _maxSpeedStepsS = 10000;
  _maxAccelStepsSS = 1000;

  initializeMotorProperties();
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
  stepper->moveTo(request, false);
}

void motor::setSpeed(int32_t request) {

}

void motor::setAcceleration(int32_t request) {
    stepper->moveByAcceleration(request, true);
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
