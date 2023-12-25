#!/bin/bash

catkin_ws_path="/aas_ws/tum_aas_project/catkin_ws" # Change this to your catkin_ws path

source $catkin_ws_path/devel/setup.bash

# ANSI escape codes for colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
RESET='\033[0m'

if [ -n "$ROS_ROOT" ]; then
    echo -e "${GREEN}ROS workspace sourced as${RESET}" $catkin_ws_path
else
    echo -e "${RED}Failed to source ROS workspace${RESET}"
fi

exit 0

# To run this script in terminal:
# sudo chmod +x rosource.sh
# ./rosource.sh