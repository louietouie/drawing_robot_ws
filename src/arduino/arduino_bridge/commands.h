#ifndef COMMANDS_H
  #define COMMANDS_H

  #define E_STOP                0x000
  #define HEARTBEAT             0x001
  #define GET_STATE             0x002
  #define SET_ACCEL             0x003
  #define SET_POSITION          0x004
  #define SET_SPEED_LIMIT       0x005
  #define SET_POSITION_LIMIT    0x006
  #define SET_POSITION_LIM_FLAG 0x007
  #define ECHO                  0x01f // max allowable (00000011111)
  // #define GET_CAN_BITRATE
  // #define SET_MICROSTEPS
  // #define GET_LIMIT_STATE

#endif