# Taking the ROS melodic image as base
FROM osrf/ros:melodic-desktop-full-bionic

# To have not questions during the installation
ENV DEBIAN_FRONTEND=noninteractive

# Use ARG - persists only during docker build
# https://github.com/moby/moby/issues/4032#issuecomment-192327844
ARG CATKIN_WORKSPACE=autsys_ws
ARG HOME=/root
ARG ROS_DISTRO=melodic

# # install ssh client and git
# RUN apt-get update && apt-get upgrade -y && \
#     apt-get install -y openssh-client git

# download public key for github.com
# RUN mkdir -p -m 0600 ~/.ssh && ssh-keyscan github.com >> ~/.ssh/known_hosts

# Install the basic packages
RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y git apt-utils python-catkin-tools wget libtool autoconf automake lsb-release gnupg2 dialog curl nano tzdata && \
    dpkg-reconfigure --frontend noninteractive tzdata && \
    rm -rf /var/lib/apt/lists/*

# # Clone and build Cartographer
# WORKDIR $HOME/$CATKIN_WORKSPACE/src
# RUN git clone git@github.com:larics/uav_ros_cartographer.git
# RUN cd  /$HOME/$CATKIN_WORKSPACE/src/uav_ros_cartographer/installation && ./install.sh
# RUN catkin build  --jobs 8

# Install the necessary ROS packages for the project
RUN apt-get update && apt-get upgrade -y && \
    apt-get install ros-melodic-depth-image-proc -y && \
    apt-get install ros-melodic-octomap ros-melodic-octomap-mapping -y && \
    apt-get install ros-melodic-ompl -y && \
    rm -rf /var/lib/apt/lists/*

# Install dependencies
RUN apt-get update && apt-get install -q -y \
    ros-$ROS_DISTRO-octomap-ros \
    ros-$ROS_DISTRO-octomap-ros \
    ros-$ROS_DISTRO-octomap \ 
    ros-$ROS_DISTRO-octomap-server \
    ros-$ROS_DISTRO-octomap-msgs \
    ros-$ROS_DISTRO-octomap-rviz-plugins \
    ros-$ROS_DISTRO-ompl git \
    ros-$ROS_DISTRO-moveit \
    ros-$ROS_DISTRO-moveit-visual-tools \
    ros-$ROS_DISTRO-dynamixel-workbench-msgs \
    libompl-dev \
    python-catkin-tools \
    libdlib-dev

# For Intel GPU support
RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y libgl1-mesa-glx libgl1-mesa-dri xserver-xorg-video-intel && \
    rm -rf /var/lib/apt/lists/*

# # Clone and build larics_motion_planning
# WORKDIR $HOME/$CATKIN_WORKSPACE/src
# RUN git clone git@github.com:larics/aerial_manipulators.git
# WORKDIR $HOME/$CATKIN_WORKSPACE/src/aerial_manipulators
# RUN git checkout devel
# WORKDIR $HOME/$CATKIN_WORKSPACE/src/
# RUN clone git@github.com:suiauthon/impedance_control.git
# RUN git clone git@github.com:larics/larics_gazebo_worlds.git
# RUN git clone git@github.com:larics/larics_motion_planning.git
# WORKDIR $HOME/$CATKIN_WORKSPACE/src/larics_motion_planning
# RUN git checkout 2d3230372e5a0a89
# RUN catkin build --jobs 8

# # Clone Gazebo models
# RUN mkdir ~/.gazebo \
#  && cd ~/.gazebo \
#  && git clone git@github.com:osrf/gazebo_models.git models

# # Clone and build uav_frontier_exploration_3d
# WORKDIR $HOME/$CATKIN_WORKSPACE/src
# RUN --mount=type=ssh git clone git@github.com:larics/uav_frontier_exploration_3d.git
# WORKDIR $HOME/$CATKIN_WORKSPACE/src/uav_frontier_exploration_3d
# RUN git checkout devel
# WORKDIR $HOME/$CATKIN_WORKSPACE/src
# RUN git clone git@github.com:larics/mean_shift_clustering.git
# RUN git clone git@github.com:AnaBatinovic/octomap-1.7.2.git
# RUN git clone git@github.com:OctoMap/octomap_ros.git
# RUN git clone git@github.com:larics/velodyne_simulator.git
    

# Currently, not working packages with melodic
# intel-opencl-icd intel-media-va-driver-non-free

# Reset the environment variable
ENV DEBIAN_FRONTEND=

# For easy sourcing of ROS workspace
COPY rosource.sh /usr/local/bin/rosource
RUN chmod +x /usr/local/bin/rosource

# RUN echo "export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:$HOME/.gazebo/models"
# RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
# RUN echo "source ~/$CATKIN_WORKSPACE/devel/setup.bash" >> ~/.bashrc
# RUN echo "octomap_DIR=~/$CATKIN_WORKSPACE/src/octomap-1.7.2/octomap/lib/cmake/octomap" >> ~/.bashrc
# RUN bash -c "source ~/.bashrc"
# RUN catkin build --jobs 8

# To build the docker image, run the following command:
# sudo docker image build -t autsys_image_melodic .

# To delete the old docker image, run the following command:
# sudo docker images
# sudo docker image rm <image_name>