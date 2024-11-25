Inspired by
https://github.com/joshnewans/ros_arduino_bridge/blob/main/ROSArduinoBridge/ROSArduinoBridge.ino
https://github.com/joshnewans/serial_motor_demo

CAN Message Format
Each byte is of the form 0x2F
First byte establishes message purpose, as per commands.h

I decided not to make the motor a class instance, like [josh](https://github.com/joshnewans/ros_arduino_bridge/blob/210a12273b23e4bb51f639906eadc59b31ba8888/ROSArduinoBridge/servos.h#L4) does for SweepServo,
because I know that I can only fit one TCP2209 on the arduino anyways, so I think a static utility class or namespace would make more sense.