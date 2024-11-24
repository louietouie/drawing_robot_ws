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

#endif

class StepperDriver {
    public:
        StepperDriver();
        void setup();
        void stop();
        int32_t getPosition();
        int32_t getVelocity();
        void setPosition(int16_t);
        void setSpeed(int16_t);
        void setAcceleration(int16_t);
        
    private:
        void initializeMotor();
        void initializeMotorProperties();
        void initializeDriver();
        // TMC2209Stepper uartDriver;
        FastAccelStepperEngine engine;
        FastAccelStepper *stepper;
        uint32_t acceleration_steps_ss;
        uint32_t handover_steps;
        uint32_t jump_start_steps;
        uint32_t max_speed_steps_s;
};

StepperDriver stepper;