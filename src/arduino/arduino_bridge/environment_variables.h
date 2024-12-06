#ifndef environment_defined

    #define environment_defined

    #define stepperSPR 200
    // #define stepperPulleyTeeth 20
    // #define stepperBeltPitchMM 2
    #define microstep 4
    #define pi 3.14159265

    // based on 11 bit identifer. 6 node ID bits and 5 command ID bits
    #define acceptedCanID 0x760
    #define heartbeatCanID 0x660
    #define maskCanID 0x7E0
    #define heartbeatMicroPeriod 100000ul

#endif

