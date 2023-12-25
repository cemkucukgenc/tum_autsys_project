# TUM Autonomous Systems (AAS) 
# 2023 Group Project: Sub-Terrain Challange

## Group 8
- Cem Kücükgenc
- Baran Özer
- Serdar Soyer
- Erencan Aslakci
- Ali Rabeh

## Introduction

??????????????????????

## Installation guide

### 1. System setup 
This project is developed for the `Ubuntu 20.04` with `ROS Noetic`. Be sure your system has the same configuration. You can check the following links to install them:

 - https://releases.ubuntu.com/focal/ 
 - https://wiki.ros.org/noetic/Installation/Ubuntu 

#### Bonus
If you have a different Linux or ROS distro, you can prefer to use a Docker container to run the simulation. To do that, an example Docker container implementation for a PC with `Intel i7-1165G7` CPU (no Discreet GPU) and `Ubuntu 22.04` will be explained. In any case, you should try to implement it according to your system with external sources. This method is not guarenteed to work perfectly in your system.

Install the Docker by following the link

- https://docs.docker.com/desktop/install/ubuntu/

Open a terminal (will be mentioned as T1) and create a directory preferably in `home` location as
```
mkdir -p aas_ws
cd aas_ws
```
Clone the project repository (T1)
```
git clone https://github.com/cemkucukgenc/tum_aas_project.git
```
On `docker/create_container.sh`, change the line 4 as your `catkin_ws_path` and line 11 as your shared volume path.

On `docker/continue_container.sh`, change the line 4 as your `catkin_ws_path`.

Bonus: The `rosurce.sh` is created to sourcing the `catkin_ws_path` easily. If you want to use that too; on `docker/rosurce.sh`, change the line 3 as your `catkin_ws_path`.

Pull the corresponding Docker image (T1)
```
sudo docker pull osrf/ros:noetic-desktop-full
```

Create necessary Docker image `aas_image` (T1)
```
cd docker
sudo docker image build -t aas_image .
```

Create the Docker container `aas_container` (T1)
```
sudo chmod +x create_container.sh
sudo chmod +x continue_container.sh
./create_container.sh
```

Congrats! Your container is created and the shared volume is already including the project repository. Use this container to run the project.

Here is some useful command that will be needed in the further steps of running the simulation. Docker container will be no longer explained, you should be able to setup, reach or delete the containers by yourself with this information.

To reach the container on another terminal
```
./continue_container.sh
```
or
```
sudo docker exec -it aas_container bash
```

To list and delete the image
```
sudo docker images
sudo docker image rm aas_image
```

To list and delete the container
```
sudo docker ps -a
sudo docker stop aas_container
sudo docker rm aas_container
```

Useful tutorials for your reference

- https://www.youtube.com/watch?v=qWuudNxFGOQ
- https://www.youtube.com/watch?v=oULAVsGlLe8&t

### 2. Building project

Open a terminal (will be mentioned as T1) and create a directory preferably in `home` location as
```
mkdir -p aas_ws
cd aas_ws
```
Clone the project repository (T1)
```
git clone https://github.com/cemkucukgenc/tum_aas_project.git
```