#ifndef CAN_H
  #define CAN_H

  // #include <SPI.h>
  #include <mcp2515.h>

  #define CS_PIN 10

#endif #CAN_H

struct can_frame canMsg;

void setupCAN();
void writeCAN();
bool readCAN();