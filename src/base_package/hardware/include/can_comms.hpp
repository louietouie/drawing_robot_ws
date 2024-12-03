#ifndef CAN_COMMS
#define CAN_COMMS

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

class CanBusComms {
    public:
        CanBusComms() {}
        // CanBusComms(int32_t bit_rate) {

        // }

        void setup();
        void sendMotorCommand(double command);
        void readMotorPosition(double &position);
        void shutdown();
        bool isConnected();

    private:
        int s;
        struct sockaddr_can addr;
        struct can_frame frame;
        struct ifreq ifr;

        void sendRaw();
};

#endif