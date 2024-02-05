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
sudo apt install wget libtool apt-utils python3-catkin-tools
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

## 3. Installation of ORB-SLAM 3 on installed Ubuntu 20.04 Noetic
Install all liberay dependencies.
```shell

sudo add-apt-repository "deb http://security.ubuntu.com/ubuntu xenial-security main"
sudo apt update

sudo apt-get install build-essential
sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev

sudo apt-get install python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libdc1394-22-dev libjasper-dev

sudo apt-get install libglew-dev libboost-all-dev libssl-dev

sudo apt install libeigen3-dev

```

## 3.1 ORB-SLAM 3 
Now, we install ORB-SLAM3 Noetic

```shell
cd /autsys_ws/tum_autsys_project/catkin_ws/src
git clone https://github.com/RAFALAMAO/ORB_SLAM3_NOETIC](https://github.com/RAFALAMAO/ORB_SLAM3_NOETIC
cd ORB_SLAM3_NOETIC
```
## 3.2 CMake Installation (If it does not exist or not >= 3.0)

```shell
cd ~
sudo apt update
sudo apt install -y software-properties-common lsb-release build-essential
wget https://github.com/Kitware/CMake/releases/download/v3.28.0/cmake-3.28.0.tar.gz
tar -zxvf cmake-3.28.0.tar.gz
cd cmake-3.28.0/
./bootstrap
make
sudo make install
cmake --version
```
---
## 3.2 Install OpenCV 4.2.0
The ORB-SLAM 3 was test by  
```shell
cd /autsys_ws/tum_autsys_project/catkin_ws/src/ORB_SLAM3_NOETIC
git clone https://github.com/opencv/opencv.git
cd opencv
git checkout 4.2.0
```
Put the following at the top of header file `gedit ./modules/videoio/src/cap_ffmpeg_impl.hpp`  
`#define AV_CODEC_FLAG_GLOBAL_HEADER (1 << 22)`  
`#define CODEC_FLAG_GLOBAL_HEADER AV_CODEC_FLAG_GLOBAL_HEADER`  
`#define AVFMT_RAWPICTURE 0x0020`  
and save and close the file
```shell
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=Release -D WITH_CUDA=OFF -D CMAKE_INSTALL_PREFIX=/usr/local ..
make -j 3
sudo make install
```
> If you want to install to conda environment, use `CMAKE_INSTALL_PREFIX=$CONDA_PREFIX` instead.
---

## 3.3 Install Pangolin
Now, we install the Pangolin. I used the commit version 86eb4975fc4fc8b5d92148c2e370045ae9bf9f5d
```shell
cd cd /autsys_ws/tum_autsys_project/catkin_ws/src/ORB_SLAM3_NOETIC
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin 
mkdir build 
cd build 
cmake .. -D CMAKE_BUILD_TYPE=Release 
make -j 3 
sudo make install
```
> If you want to install to conda environment, add `CMAKE_INSTALL_PREFIX=$CONDA_PREFIX` instead.
---

In `CMakeLists.txt`, change the OpenCV version line to

    find_package(OpenCV 4.2)

We also need to make changes to `System.cc`. Open it with `gedit ./src/System.cc` or your favorite text editor, and change both lines 584 and 701 from

    Map* pBiggerMap;

to

    Map* pBiggerMap = nullptr;

This latter error allowed the examples to run fine, but they caused a segmentation fault at the very end when trying to save the map.

Now all compilation errors should be fixed, and we can build ORB_SLAM3. Run their provided shell script `build.sh`, which will build all of the third party programs as well as ORB_SLAM3 itself. 

    ./build.sh

It will likely show errors, but just run it a few times without changing anything, and it should succeed after a few attempts.

## Download Example Data (Optional)
We will use the EuRoC MH_01 easy dataset.

    cd ~
    mkdir -p Datasets/EuRoc
    cd Datasets/EuRoc/
    wget -c http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/machine_hall/MH_01_easy/MH_01_easy.zip
    mkdir MH01
    unzip MH_01_easy.zip -d MH01/

Two of the images in my download were corrupted, so for the program to run successfully, I replaced each of these with their nearest frame.

    cd ~/Datasets/EuRoc/MH01/mav0/cam0/data
    rm 1403636689613555456.png
    cp 1403636689663555584.png 1403636689613555456.png
    rm 1403636722213555456.png
    cp 1403636722263555584.png 1403636722213555456.png

If the example exits with a segmentation fault, retry the unzip step for your data, and pay attention for an image failing with a bad CRC; the image is likely corrupted and should be replaced in the manner I've done here. Note that each image must appear by name, so corrupt images must be replaced by an adjacent frame rather than simply deleted.

## Run Simulation with Examples (Optional)

    cd /autsys_ws/tum_autsys_project/catkin_ws/src/ORB_SLAM3_NOETIC

Then, choose one of the following to run. A map viewer as well as an image viewer should appear after it finishes setup.

    # Mono
    ./Examples/Monocular/mono_euroc ./Vocabulary/ORBvoc.txt ./Examples/Monocular/EuRoC.yaml ~/Datasets/EuRoc/MH01 ./Examples/Monocular/EuRoC_TimeStamps/MH01.txt dataset-MH01_mono

    # Mono + Inertial
    ./Examples/Monocular-Inertial/mono_inertial_euroc ./Vocabulary/ORBvoc.txt ./Examples/Monocular-Inertial/EuRoC.yaml ~/Datasets/EuRoc/MH01 ./Examples/Monocular-Inertial/EuRoC_TimeStamps/MH01.txt dataset-MH01_monoi

    # Stereo
    ./Examples/Stereo/stereo_euroc ./Vocabulary/ORBvoc.txt ./Examples/Stereo/EuRoC.yaml ~/Datasets/EuRoc/MH01 ./Examples/Stereo/EuRoC_TimeStamps/MH01.txt dataset-MH01_stereo

    # Stereo + Inertial
    ./Examples/Stereo-Inertial/stereo_inertial_euroc ./Vocabulary/ORBvoc.txt ./Examples/Stereo-Inertial/EuRoC.yaml ~/Datasets/EuRoc/MH01 ./Examples/Stereo-Inertial/EuRoC_TimeStamps/MH01.txt dataset-MH01_stereoi

## Validating Estimates vs Ground Truth (Optional)
We're using python 2.7, and need numpy and matplotlib. For this, we need the 2.7 version of pip.

    sudo apt install curl
    cd ~/Desktop
    curl https://bootstrap.pypa.io/pip/2.7/get-pip.py --output get-pip.py
    sudo python2 get-pip.py
    pip2.7 install numpy matplotlib

Now run and plot the ground truth:

    cd ~/Dev/ORB_SLAM3

    ./Examples/Stereo/stereo_euroc ./Vocabulary/ORBvoc.txt ./Examples/Stereo/EuRoC.yaml ~/Datasets/EuRoc/MH01 ./Examples/Stereo/EuRoC_TimeStamps/MH01.txt dataset-MH01_stereo

And now run and plot the estimate compared to ground truth:

    cd ~/Dev/ORB_SLAM3

    python evaluation/evaluate_ate_scale.py evaluation/Ground_truth/EuRoC_left_cam/MH01_GT.txt f_dataset-MH01_stereo.txt --plot MH01_stereo.pdf

Open the pdf `MH01_stereo.pdf` to see the results. This can be done with the command `evince MH01_stereo.pdf`.

---

# ROS Implementation




## 4. Launching the simulation <a name="launching_the_simulation"></a>

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


