#!/bin/bash

container_name="aas_container" # Change this to your container name
catkin_ws_path="/aas_ws/tum_aas_project/catkin_ws" # Change this to your catkin_ws path

xhost +
sudo docker run \
  --net=host \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="$HOME/.Xauthority:/root/.Xauthority:rw" \
  --volume=/home/cemkgenc/aas_ws:/aas_ws \
  --device=/dev/dri:/dev/dri \
  --env="DISPLAY=$DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --name $container_name \
  -it \
  -d \
  aas_image

sleep 2

# ANSI escape codes for colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
RESET='\033[0m'

sudo docker exec -it $container_name bash -c \
  "source /opt/ros/noetic/setup.bash && \
   if [ \$? -eq 0 ]; then
     echo -e '${GREEN}ROS distro sourced as${RESET}' \$ROS_DISTRO 
   else
     echo -e '${RED}Failed to source ROS distro${RESET}' 
   fi && \
   source $catkin_ws_path/devel/setup.bash && \
   if [ \$? -eq 0 ]; then
     echo -e '${GREEN}ROS workspace sourced as${RESET}' $catkin_ws_path
   else
     echo -e '${RED}Failed to source ROS workspace${RESET}'
   fi && \
   bash"

exit 0

# To run this script in terminal:
# sudo chmod +x create_container.sh
# ./create_container.sh