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
        void sendFrame();
        void readFrame();
        void shutdown();

    private:
        int ret;
        int s, nbytes;
        struct sockaddr_can addr;
        struct ifreq ifr;
        struct can_frame frame;
};

#endif