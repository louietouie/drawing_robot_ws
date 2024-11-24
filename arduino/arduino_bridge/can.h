#ifndef CAN_H
  #define CAN_H

  // #include <SPI.h>
  #include <mcp2515.h>

  #define CS_PIN 10

  extern struct can_frame canMsg; // extern is "telling the compiler that you have already [declared] the [variable] somewhere in your project" Some reason .h file not working

  void setupCAN();
  void writeCAN();
  bool readCAN();

#endif

