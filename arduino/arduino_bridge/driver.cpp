#include "driver.h"

using namespace motor;
FastAccelStepper *stepper;
FastAccelStepperEngine engine;
TMC2209Stepper uartDriver = TMC2209Stepper(SW_RX, SW_TX, R_SENSE, DRIVER_ADDRESS);

void initializeMotorProperties() {
    stepper->setDirectionPin(DIRECTION_STEPPER_PIN);
    stepper->setEnablePin(ENABLE_STEPPER_PIN);
    stepper->setAutoEnable(true);
    stepper->setLinearAcceleration(0);
    stepper->setSpeedInHz(10000);
    // stepper->setAcceleration(acceleration_steps_ss);
    stepper->setJumpStart(0);
}

void motor::setup() {
  engine = FastAccelStepperEngine();
  engine.init();
  stepper = engine.stepperConnectToPin(STEP_STEPPER_PIN);
  while (!stepper) {};
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

void motor::setPosition(int32_t request) {}

void motor::setSpeed(int32_t request) {}

void motor::setAcceleration(int32_t request) {
    stepper->moveByAcceleration(request, true);
}
