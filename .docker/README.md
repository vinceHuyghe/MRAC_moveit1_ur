# Docker instructions

The .docker folder of this repo contains convenience shell scripts for building and running the Docker container. These should be run from the root of the repo.

The Dockefile uses multiple stages to enable features such as STOMP and RGBD cameras, see Dockerfile comments for more information.

## Build Image

Execute the build image shell script

```shell
.docker/build_image.sh
```

Or

```shell
DOCKER_BUILDKIT=1 \
docker build --pull --rm -f ./.docker/Dockerfile_staged \
--build-arg BUILDKIT_INLINE_CACHE=1 \
--target bash \
--tag moveit1_ur:latest .
```

## Run Image

note: On Ubuntu 20.04 --privileged flag is required on Ubuntu 22 it can be omitted

The instructions below will give the container access to the users home directory and log in as the user with their password and x sever access.  

Execute the run image shell script

```shell
.docker/run_user.sh
```

or

```shell
docker run -it \
    --user=$(id -u $USER):$(id -g $USER) \
    --group-add sudo \
    --env="DISPLAY" \
    --env=QT_X11_NO_MITSHM=1 \
    --workdir="/dev_ws" \
    --volume="/home/$USER:/home/$USER" \
    --volume="/etc/group:/etc/group:ro" \
    --volume="/etc/passwd:/etc/passwd:ro" \
    --volume="/etc/shadow:/etc/shadow:ro" \
    --volume="/etc/sudoers.d:/etc/sudoers.d:ro" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="/dev:/dev" \
    --net=host \
    --cap-add=sys_nice \
    moveit1_ur:latest
```

For nvidia-docker

```shell
.docker/run_user_nvidia.sh
```

or

```shell
docker run -it \
    --user=$(id -u $USER):$(id -g $USER) \
    --group-add sudo \
    --env="DISPLAY" \
    --env=QT_X11_NO_MITSHM=1 \
    --workdir="/dev_ws" \
    --volume="/home/$USER:/home/$USER" \
    --volume="/etc/group:/etc/group:ro" \
    --volume="/etc/passwd:/etc/passwd:ro" \
    --volume="/etc/shadow:/etc/shadow:ro" \
    --volume="/etc/sudoers.d:/etc/sudoers.d:ro" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="/dev:/dev" \
    --net=host \
    --privileged \
    --cap-add=sys_nice \
    --gpus 'all,"capabilities=compute,display,graphics,utility"' \
    moveit1_ur:latest
```

## Use

### terminal

#### multiple terminals

terminator is installed in the container and can be run with `terminator`.

#### sourcing the ROS environment

Add the following to your .bashrc (located in home/$USER) to automatically source the ROS environment when opening a new shell in the container.

```bash
if [ -f "/dev_ws/setup.bash" ]; then
    source /dev_ws/setup.bash
fi
```

or if you are using zsh to your .zshrc

```zsh
if [ -f "/dev_ws/setup.zsh" ]; then
    source /dev_ws/setup.zsh
fi
```

### vscode integration

requires the following vscode extensions to be installed

- [docker](https://code.visualstudio.com/docs/containers/overview)
- [remote development](https://code.visualstudio.com/docs/remote/remote-overview)

devcontainer.json is included in the .docker directory of this repo.

#### Attach vscode to container

In vs code go to the Docker tab in the side bar. Right click on the container named moveit1_ur:latest. Select attach vscode.

#### When attaching to the container for first time

In vs code open the command palette (Ctrl-Shift-P). Select Remote-containers: Open attached container configuration file. Copy paste content of devcontainer.json and save. Close the vscode window and reattach.

## Interfacing with hardware

### Process priority for UR driver

When not using a realtime kernel it is recommended to run the UR driver with high process priority. The bringup launch file will attempt to start the UR driver with hight process priority.

See process [nice](https://en.wikipedia.org/wiki/Nice_(Unix)) for more information on process niceness.

This requires the user to have the permissions to change the niceness of processes.

Add the following to /etc/security/limits.conf on the Docker host

`$USER - nice -15`

Once inside the container run `su $USER` in order for the permissions to be loaded.

### Udev rules

Download rules and place them into /etc/udev/rules.d/ on the Docker host

- [realsense](https://github.com/IntelRealSense/librealsense/blob/master/config/99-realsense-libusb.rules)

- [asra](https://github.com/orbbec/astra/blob/master/install/orbbec-usb.rules)

### Connecting to the robot

The bringup launch file will attempt to start the UR driver with hight process priority.  

Ensure both the user on the host and container have the required permissions to do so.  
See section "Process priority for UR driver"

The launch files assume

- Robot IP: 192.168.56.101
- Docker Host IP: 192.168.56.1

Launch the robot bringup, this file sets the robot IP and loads the kinematics calibration for the IAAC UR10e.  

- without endeffector

    ```shell
    ur10e_moveit_config ur10e_iaac_bringup.launch 
    ```

- with endeffector

    ```shell
    ur10e_ee_moveit_config ur10e_ee_iaac_bringup.launch 
    ```

On the ur pendant start URcaps

If URcaps fails to connect add the following rule to the firewall (ufw) on the docker host u

```shell
sudo ufw allow from 192.168.56.101 to 192.168.56.1
```

When the robot is connected you should see the following in the terminal you launched the bringup launchfile from.

```shell
[ INFO]  Sent program to robot
[ INFO]  Robot connected to reverse interface. Ready to receive control commands.
```

You can use the `top` command to check the ur driver is running unnicely
