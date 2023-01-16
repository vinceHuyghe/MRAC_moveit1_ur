# ROS Noetic Moveit with UR10e

## Description

Contents of container:

- ROS noetic
- Moveit

  - IK
    - KDL
    - TRAC-IK

  - Planning pipelines
    - OMPL
    - CHOMP
    - STOMP
    - Pilz Industrial Motion Planner

- UR10e moveit_config (with IAAC calibration) and ur driver. There is a moveit_config with and without end effector.
- [RQT joint trajectory plot](https://github.com/tork-a/rqt_joint_trajectory_plot)
- [industrial reconstruction](https://github.com/gavanderhoorn/industrial_reconstruction) (ros1 branch)
- [realsense-ros](https://github.com/rjwb1/realsense-ros) (with D405 support in ROS1)
- [orbec astra](https://github.com/orbbec/ros_astra_camera)

## Docker

For docker instructions see README in the .docker directory of this repo.

## Detailed instructions

See instructions directory

## Use

### Running in simulation

```shell
roslaunch ur10e_moveit_config demo.launch
```

&nbsp;

### Running on the robot

In order to ensure optimal performance of the ur_driver without the realtime kernel the ur_robot-driver is run with a high priority.

&nbsp;

#### Process priority

When not using a realtime kernel it is recommended to run the UR driver with high process priority.

See process [nice](https://en.wikipedia.org/wiki/Nice_(Unix)) for more information on process niceness.

This requires the user to have the permissions to change the niceness of processes.

Add the following to /etc/security/limits.conf on the Docker host

`$USER - nice -15`

Once inside the container run `su $USER` in order for the permissions to be loaded.

&nbsp;

#### Udev rules

Download rules and place them into /etc/udev/rules.d/ on the Docker host

- [realsense](https://github.com/IntelRealSense/librealsense/blob/master/config/99-realsense-libusb.rules)

- [asra](https://github.com/orbbec/astra/blob/master/install/orbbec-usb.rules)

Reboot host if camera is not detected

&nbsp;

#### Network configuration

These instructions assume

- Robot IP: 192.168.56.101
- Docker Host IP: 192.168.56.1

If URcaps fails to connect add the following rule to the firewall (ufw) on the docker host.

```bash
sudo ufw allow from 192.168.56.101 to 192.168.56.1
```

Launch the robot bringup, this file sets the robot IP and loads the kinematics calibration for the IAAC UR10e.  

- without endeffector

    ```bash
    ur10e_moveit_config ur10e_iaac_bringup.launch 
    ```

- with endeffector

    ```bash
    ur10e_ee_moveit_config ur10e_ee_iaac_bringup.launch 
    ```

On the ur pendant load and start URcaps

When the robot is connected you should see the following in the terminal used to launch the bringup

```bash
[ INFO]  Sent program to robot
[ INFO]  Robot connected to reverse interface. Ready to receive control commands.
```

You can use the `top` or rqt-top command to check the ur driver is running unnicely (-15)
