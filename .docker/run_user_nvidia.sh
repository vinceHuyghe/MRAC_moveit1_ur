#!/usr/bin/env bash

# privileged flag required for ubuntu 20.04

echo -e "Starting up the user moveit1 container \n >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"
echo -e "This container will access to the users home directory and log in as the user with their password and x sever access.\nYou will not own the workspace though, use sudo chown -R $USER /dev_ws"
echo -e "In order for the ur_robot_driver to run unnicely use su $USER"
echo -e "Source the workspace with source devel/setup.bash"

docker run -it --privileged \
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
    --gpus 'all,"capabilities=compute,display,graphics,utility"' \
    moveit1_ur:latest