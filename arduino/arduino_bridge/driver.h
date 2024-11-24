#ifndef stepper_h

    #define stepper_h
    #include <TMCStepper.h>
    #include "FastAccelStepper.h"

    #define STEP_STEPPER_PIN 9
    #define DIRECTION_STEPPER_PIN 8
    #define ENABLE_STEPPER_PIN 5
  
    #define SW_RX            7 // TMC2208/TMC2224 SoftwareSerial receive pin
    #define SW_TX            6 // TMC2208/TMC2224 SoftwareSerial transmit pin
    #define DRIVER_ADDRESS 0b00 // TMC2209 Driver address according to MS1 and MS2
    #define R_SENSE 0.11f // Match to your driver
                        // SilentStepStick series use 0.11
                        // UltiMachine Einsy and Archim2 boards use 0.2
                        // Panucatt BSD2660 uses 0.1
                        // Watterott TMC5160 uses 0.075

  // I know only one instance of this driver will exist, so don't need to make it a class that can be declared multiple times.
  // Similar to a utility class of static functions.
  namespace motor {

    extern FastAccelStepper *stepper;
    extern FastAccelStepperEngine engine;
    extern TMC2209Stepper uartDriver; // why does this constructor need to be called in header, but engine constructor called in cpp?

    void setup();
    void stop();
    int32_t getPosition();
    int32_t getSpeed();
    void setPosition(int32_t);
    void setSpeed(int32_t);
    void setAcceleration(int32_t);

  }

#endif