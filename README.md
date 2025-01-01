## Purpose

The purpose of this repo is to control a 3D printed robot arm (with one ODrive actuator and one NEMA17 stepper motor). It uses ROS2 control hardware interfaces to send commands and recieve states from the actuators, and a ROS2 control controller interface to generate joint position commands from cartesian position commands published over a ROS topic. The transformation from end-effector cartesian space to joint-space is done with the help of the Drake library, and uses differential inverse kinematics.

## Repo Summary

This repository consists of 3 ROS2 packages and code to run on the hardware to act as a CAN reciever and stepper motor controller. 2 of the ROS packages are almost direct copies from the [ODrive ROS2 Control](https://github.com/odriverobotics/ros_odrive) github repo.

## Setup on RPi 5

It took me a while to figure out how to get ROS2 running and Drake installed on an RPi 5 running Ubuntu 22.04.

Below are some noteable things that I got stuck on...

#### Installing Drake

I first tried building Drake via *bazel build...* after installing bazel with bazelisk. Even though the build was successful, I had a lot of trouble connecting the library up to my ROS2 project, which used CMake. It was recommended that since my ROS2 project is built with CMake, I should build Drake with CMake too.

Building Drake via CMake worked better...

First, I added `build --jobs=1` to the text file `$HOME/.bazelrc`, as described in the [Drake documentation](https://drake.mit.edu/troubleshooting.html). Without this, the build failed a few hours in with the error `cc: fatal error: Killed signal terminated program cc1plus`.

```
git clone --filter=blob:none https://github.com/RobotLocomotion/drake.git
mkdir drake-build
cd drake-build
cmake ../drake -DCMAKE_INSTALL_PREFIX=/the/destination/you/want
make install
```

Then, my launch files work as long as I add LD_PRELOAD.

`LD_PRELOAD=/home/louis/Documents/drake-build/install/lib/libdrake.so ros2 launch base_packager master.launch.py`

I have not yet figured out how to connect the Drake library within `base_package`'s `CMakeLists.txt` file.

#### RPi RS485 CAN Setup

[Waveshare](https://www.waveshare.com/wiki/RS485_CAN_HAT) has good install instructions, however I had to make one change since the RPi5 is running Ubuntu, not RPiOS.

Instead of `sudo nano /boot/config.txt`, I did `sudo nano /boot/firmware/config.txt`

#### Starting CAN on RPi5

Before launching the `master.launch.py` file, you also need to begin CAN. Make sure the bitrates match on the Arduino's MCP2515, the ODrive S1, and the RPi5 RS485.

`sudo ip link set can0 up type can bitrate 250000`

#### Publishing Commands to the ROS Topic

```
ros2 topic pub /my_ik_controller/commands std_msgs/msg/Float64MultiArray "data:
- 0.5
- 0.5"
```
