#include "can.h"

MCP2515 mcp2515(10);
struct can_frame canMsg;

void setupCAN() {
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();
}

void writeCAN() {
  // canMsg.can_id = 0x200;
  // canMsg.can_dlc = 4;
  mcp2515.sendMessage(&canMsg);
}

bool readCAN() {
  return mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK;

  // if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
  //   Serial.print(canMsg.can_id, HEX); // print ID
  //   Serial.print(": "); 
  //   Serial.print(canMsg.can_dlc, HEX); // print DLC
  //   Serial.print(" ");
  //   for (int i = 0; i<canMsg.can_dlc; i++)  {  // print the data
  //     Serial.print(canMsg.data[i],HEX);
  //     Serial.print(" ");
  //   }
  //   return true;
  // }
  // return false;
}