# TUM AAS - Autonomous Systems - 2023 Group Project: Sub-Terrain Challange

### Group 4
- Cem Kücükgenc     (cem.kucukgenc@tum.de)
- Baran Özer        (baran.oezer@tum.de)
- Serdar Soyer      (serdar.soyer@tum.de)
- Erencan Aslakci   (ge97jed@mytum.de)
- Hünkar Suci       (hunkar.suci@tum.de)

### Table of contents
1. [Introduction](#introduction)
2. [Installation guide](#installation_guide)
    1. [System setup](#system_setup)
    2. [Installing dependencies](#installing_dependencies)
    3. [Building project](#building_project)
3. [Launching the simulation](#launching_the_simulation)

## 1. Introduction <a name="introduction"></a>

??????????????????????

## 2. Installation guide <a name="installation_guide"></a>

### 2.1. System setup <a name="system_setup"></a>
This project is developed for `Ubuntu 20.04` with `ROS Noetic`. Be sure your system has the same configuration. You can check the following links to install them:

 - `Ubuntu 20.04` setup guide link: https://releases.ubuntu.com/focal/ 
 - `ROS Noetic` setup guide link: https://wiki.ros.org/noetic/Installation/Ubuntu 

 It is required that the `ROS Noetic` should be sourced in each terminal to run the simulation as mentioned in the `ROS Noetic` setup guide link. To do that,
 ```
source /opt/ros/noetic/setup.bash
 ```
should be sourced in each terminal. To make it more easy, it can be added to the `.bashrc` file for automatic sourcing. Details can be found in the `ROS Noetic` setup guide link. Sourcing `ROS Noetic` will no longer be mentioned in further steps.

#### Bonus: Docker container setup (Bypass this section and continue from "2.2. [Installing dependencies](#installing_dependencies)" if you have a successful installation of `Ubuntu 20.04` with `ROS Noetic`)

If you have a different Linux or ROS distro, you may prefer to use a Docker container to run the simulation. To do that, an example Docker container implementation for a PC with `Intel i7-1165G7` CPU (no Discreet GPU) and `Ubuntu 22.04` will be explained. In any case, you should try to implement it according to your system with external sources. This method is not guaranteed to work perfectly in your system.

Install the Docker by following the link

- https://docs.docker.com/desktop/install/ubuntu/

Open a terminal (will be mentioned as T1) and create a directory preferably in the `home` location as
```
mkdir -p autsys_ws
cd autsys_ws
```
Clone the project repository (T1)
```
git clone git@github.com:cemkucukgenc/tum_autsys_project.git
```
On `docker/create_container.sh`, change line 4 to your `catkin_ws_path` and line 11 to your shared volume path.

On `docker/continue_container.sh`, change the line 4 as your `catkin_ws_path`.

Bonus: The `rosurce.sh` is created to source the `catkin_ws_path` easily. If you want to use that too; on `docker/rosurce.sh`, change line 3 to your `catkin_ws_path`.

Pull the corresponding Docker image (T1)
```
sudo docker pull osrf/ros:noetic-desktop-full
```

Create necessary Docker image `autsys_image` (T1)
```
cd tum_autsys_project/docker
sudo docker image build -t autsys_image .
```

Create the Docker container `autsys_container` (T1)
```
sudo chmod +x create_container.sh
sudo chmod +x continue_container.sh
./create_container.sh
```

Congrats! Your container is created and the shared volume is already including the project repository. Use this container to run the project.

Here are some useful commands that will be needed in the further steps of running the simulation. Docker container will no longer be explained, you should be able to set up, reach, or delete the containers by yourself with this information.

To reach the container on another terminal
```
cd autsys_ws/tum_autsys_project/docker
./continue_container.sh
```
or
```
sudo docker exec -it autsys_container bash
```

To list and delete the image
```
sudo docker images
sudo docker image rm autsys_image
```

To list and delete the container
```
sudo docker ps -a
sudo docker stop autsys_container
sudo docker rm autsys_container
```

Useful tutorials for your reference

- https://www.youtube.com/watch?v=qWuudNxFGOQ
- https://www.youtube.com/watch?v=oULAVsGlLe8&t

### 2.2. Installing dependencies <a name="installing_dependencies"></a>

To have a clear installation, updating and upgrading your system is recommended.
```
sudo apt-get update
sudo apt-get upgrade
```

For the installation of packages, basic tools are required. 
```
sudo apt install wget libtool apt-utils python3-catkin-tools python3-wstool
```

For the generation of the Point Cloud, `depth_image_proc` package has been utilized. 
```
sudo apt install ros-noetic-depth-image-proc
```

For the generation of the OctoMap, `octomap_mapping` package has been utilized. 
```
sudo apt-get install ros-noetic-octomap ros-noetic-octomap-mapping
```

For the path planning, `Open Motion Planning Library (OMPL)` is required. 
```
sudo apt-get install ros-noetic-ompl
```

### 2.3. Building project <a name="building_project"></a>

Open a terminal (will be mentioned as T1) and create a directory preferably in the `home` location as
```
mkdir -p autsys_ws
cd autsys_ws
```
Clone the project repository (T1)
```
git clone git@github.com:cemkucukgenc/tum_autsys_project.git
```
From the following link, download the `Simulation.zip` file from the Challange folder. (Since the content is large in size, it is not added to git.)

- https://syncandshare.lrz.de/getlink/fiQNaj3tQpvatqB8A5gAHV/

Extract the contents and copy all of them into
```
autsys_ws/tum_autsys_project/catkin_ws/src/simulation
```
Make the `Simulation.x86_64` as executable (T1)
```
cd tum_autsys_project/catkin_ws/src/simulation
chmod +x Simulation.x86_64 
```
To build the project in `autsys_ws/tum_autsys_project/catkin_ws` (T1)
```
cd ../..
catkin build
```

## 3. Launching the simulation <a name="launching_the_simulation"></a>

To run the simulation (T1)
```
source /autsys_ws/tum_autsys_project/catkin_ws/devel/setup.bash
roslaunch simulation simulation.launch
```
To run the drone controller, open a new terminal (T2)
```
source /autsys_ws/tum_autsys_project/catkin_ws/devel/setup.bash
roslaunch state_machine_pkg state_machine.launch
```


