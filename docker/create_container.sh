#!/bin/bash

container_name="autsys_container_melodic" # Change this to your container name
catkin_ws_path="/autsys_ws/tum_autsys_project/catkin_ws" # Change this to your catkin_ws path

xhost +
sudo docker run \
  --net=host \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="$HOME/.Xauthority:/root/.Xauthority:rw" \
  --volume=/home/cemkgenc/autsys_ws:/autsys_ws \
  --device=/dev/dri:/dev/dri \
  --env="DISPLAY=$DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --name $container_name \
  -it \
  -d \
  autsys_image_melodic

sleep 2

# ANSI escape codes for colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
RESET='\033[0m'

sudo docker exec -it $container_name bash -c "
    sudo apt update && sudo apt upgrade -y
    if [ -f '/opt/ros/melodic/setup.bash' ]; then
        source /opt/ros/melodic/setup.bash && \
        if [ \$? -eq 0 ]; then
            echo -e '${GREEN}ROS distro sourced as${RESET}' \$ROS_DISTRO 
        else
            echo -e '${RED}Failed to source ROS distro${RESET}' 
        fi
    else
        echo -e '${YELLOW}/opt/ros/melodic/setup.bash does not exist yet. Skipping sourcing.${RESET}'
    fi && \
    if [ -f '$catkin_ws_path/devel/setup.bash' ]; then
        source $catkin_ws_path/devel/setup.bash && \
        if [ \$? -eq 0 ]; then
            echo -e '${GREEN}ROS workspace sourced as${RESET}' $catkin_ws_path
        else
            echo -e '${RED}Failed to source ROS workspace${RESET}' $catkin_ws_path
        fi
    else
        echo -e '${YELLOW}$catkin_ws_path/devel/setup.bash does not exist yet. Skipping sourcing.${RESET}'
    fi && \
    bash"

exit 0

# To run this script in terminal:
# sudo chmod +x create_container.sh
# ./create_container.sh