#!/bin/bash

container_name="autsys_container_melodic" # Change this to your container name
catkin_ws_path="/autsys_ws/tum_autsys_project/catkin_ws" # Change this to your catkin_ws path

# ANSI escape codes for colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
RESET='\033[0m'

sudo docker exec -it $container_name bash -c "
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
# sudo chmod +x continue_container.sh
# ./continue_container.sh