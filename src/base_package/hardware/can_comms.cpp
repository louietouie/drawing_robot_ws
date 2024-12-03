// Raspberry Pi RS485 CAN communication
// https://www.waveshare.com/wiki/RS485_CAN_HAT

#include "can_comms.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

void CanBusComms::setup() {

    memset(&frame, 0, sizeof(struct can_frame));

    system("sudo ip link set can0 type can bitrate 1000000");
    system("sudo ifconfig can0 up");
    printf("this is a can send demo\r\n");
        
    //1.Create socket
    s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) {
        perror("socket PF_CAN failed");
        printf("poop");
    }

    //2.Specify can0 device
    strcpy(ifr.ifr_name, "can0");
    ret = ioctl(s, SIOCGIFINDEX, &ifr);
    if (ret < 0) {
        perror("ioctl failed");
        printf("poop");
    }

    //3.Bind the socket to can0
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    ret = bind(s, (struct sockaddr *)&addr, sizeof(addr));
    if (ret < 0) {
        perror("bind failed");
        printf("poop");
    }

    //4.Disable filtering rules, do not receive packets, only send
    setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);

    //5.Set send data
    frame.can_id = 0x123;
    frame.can_dlc = 8;
    frame.data[0] = 1;
    frame.data[1] = 2;
    frame.data[2] = 3;
    frame.data[3] = 4;
    frame.data[4] = 5;
    frame.data[5] = 6;
    frame.data[6] = 7;
    frame.data[7] = 8;

    printf("can_id  = 0x%X\r\n", frame.can_id);
    printf("can_dlc = %d\r\n", frame.can_dlc);
    int i = 0;
    for(i = 0; i < 8; i++)
        printf("data[%d] = %d\r\n", i, frame.data[i]);

    //6.Send message
    nbytes = write(s, &frame, sizeof(frame)); 
    if(nbytes != sizeof(frame)) {
        printf("Send Error frame[0]!\r\n");
        system("sudo ifconfig can0 down");
    }

}

void CanBusComms::sendFrame() {
    
}

void CanBusComms::readFrame() {
    
}

void CanBusComms::shutdown() {
    close(s);
    system("sudo ifconfig can0 down");
}

// class CanBusComms {
//     public:
//         CanBusComms() {}
//         CanBusComms(int32_t bit_rate) {

//         }

//         void setup();
//         void sendFrame();
//         void readFrame();
//         void shutdown();

//     private:
//         int ret;
//         int s, nbytes;
//         struct sockaddr_can addr;
//         struct ifreq ifr;
//         struct can_frame frame;
// }

