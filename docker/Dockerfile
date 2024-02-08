# Taking the ROS melodic image as base
FROM osrf/ros:melodic-desktop-full-bionic

# To have not questions during the installation
ENV DEBIAN_FRONTEND=noninteractive

# Install the basic packages
RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y git apt-utils python-catkin-tools wget libtool autoconf automake && \
    rm -rf /var/lib/apt/lists/*

# Install the necessary ROS packages for the project
RUN apt-get update && apt-get upgrade -y && \
    apt-get install ros-melodic-depth-image-proc -y && \
    apt-get install ros-melodic-octomap ros-melodic-octomap-mapping -y && \
    apt-get install ros-melodic-ompl -y && \
    rm -rf /var/lib/apt/lists/*

# For Intel GPU support
RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y libgl1-mesa-glx libgl1-mesa-dri xserver-xorg-video-intel && \
    rm -rf /var/lib/apt/lists/*

# Currently, not working packages with melodic
# intel-opencl-icd intel-media-va-driver-non-free

# Reset the environment variable
ENV DEBIAN_FRONTEND=

# For easy sourcing of ROS workspace
COPY rosource.sh /usr/local/bin/rosource
RUN chmod +x /usr/local/bin/rosource

# To build the docker image, run the following command:
# sudo docker image build -t autsys_image_melodic .

# To delete the old docker image, run the following command:
# sudo docker images
# sudo docker image rm <image_name>
