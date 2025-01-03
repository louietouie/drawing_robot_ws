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

    s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) {
        perror("Socket creation failed");
        return;
    }

    strcpy(ifr.ifr_name, "can0");
    ioctl(s, SIOCGIFINDEX, &ifr);
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("Bind failed");
        return;
    }

    // Test message.
    frame.can_id = 0x123;
    frame.can_dlc = 4;
    frame.data[0] = 0x11;
    frame.data[1] = 0x06;
    frame.data[2] = 0x35;
    frame.data[3] = 0x09;

    sendRaw();

}

void CanBusComms::sendMotorCommand(double command) {

    float arduinoFriendly = (float) command;
    memcpy(frame.data, &arduinoFriendly, sizeof(float));

    frame.can_id = 0x764;
    frame.can_dlc = 4;

    sendRaw();
}

void CanBusComms::readMotorPosition(double &position) {
    position = 1;
    readRaw();

    float convertedPosition;
    memcpy(&convertedPosition, frame.data, sizeof(float));
    position = convertedPosition;
}

void CanBusComms::shutdown() {
    close(s);
    printf("closed\n");
}

void CanBusComms::sendRaw() {
    if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        perror("Write failed");
        return;
    }
}

void CanBusComms::readRaw() {

    struct can_filter rfilter[1];
    rfilter[0].can_id = 0x660;
    rfilter[0].can_mask = CAN_SFF_MASK;
    setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));

    nbytes = read(s, &frame, sizeof(struct can_frame)); // read message into frame

    if (nbytes < 0) {
        perror("Read");
        return;
    }
}

bool CanBusComms::isConnected() {
    return true;
}