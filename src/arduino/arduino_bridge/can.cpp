#include "can.h"

MCP2515 mcp2515(10);
struct can_frame canMsg;

void setupCAN() {
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();
}

void writeCAN() {
  mcp2515.sendMessage(&canMsg);
}

bool readCAN() {
  return mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK;
}