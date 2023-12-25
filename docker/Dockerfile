# Taking the ROS noetic image as base
FROM osrf/ros:noetic-desktop-full

# Install the necessary packages
RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y python3-catkin-tools git apt-utils && \
    rm -rf /var/lib/apt/lists/*

# To have not questions during the installation
ENV DEBIAN_FRONTEND=noninteractive

# For Intel GPU support
RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y libgl1-mesa-glx libgl1-mesa-dri intel-opencl-icd intel-media-va-driver-non-free xserver-xorg-video-intel && \
    rm -rf /var/lib/apt/lists/*

# Reset the environment variable
ENV DEBIAN_FRONTEND=

# For easy sourcing of ROS workspace
COPY rosource.sh /usr/local/bin/rosource
RUN chmod +x /usr/local/bin/rosource

# To build the docker image, run the following command:
# sudo docker image build -t aas_image .

# To delete the old docker image, run the following command:
# sudo docker images
# sudo docker image rm <image_name>
