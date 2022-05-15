.. OUTDOOR_NAV2 documentation master file, created by
   sphinx-quickstart on Tue Dec 22 16:24:53 2020.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Getting Started
========================================

* Install ROS2 foxy. 
Deb installation is strongly recomended. 
You can always find updated step to install [here](https://index.ros.org/doc/ros2/Installation/Foxy/Linux-Install-Debians/)
To install ROS2 foxy desktop ;

.. code-block:: bash

   sudo locale-gen en_US en_US.UTF-8
   sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
   export LANG=en_US.UTF-8
   sudo apt update && sudo apt install curl gnupg2 lsb-release
   curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
   sudo sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'
   sudo apt update
   sudo apt install ros-foxy-desktop
   source /opt/ros/foxy/setup.bash


* Get the project repository, source build deps and build deps first; 

.. code-block:: bash

   mkdir -p ~/ros2_ws/src
   source /opt/ros/foxy/setup.bash
   sudo apt install python3-colcon-common-extensions
   sudo apt install -y python3-rosdep2
   sudo apt-get install git
   rosdep update
   cd ~/ros2_ws/src
   git clone --recursive https://github.com/NMBURobotics/botanbot_sim.git
   cd ~/ros2_ws
   rosdep install -y -r -q --from-paths src --ignore-src --rosdistro foxy   
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 4

.. note::
   If for some reason you could not build vox_nav, a good place to seek for a solution is the github actions file that we have.
   After each pull-push github actions is setup to build the botanbot_sim on remote to ensure stability of builds. 
   Find a recent successful build and see the commands in .github/workflows/main.yml. The commands should more or less look as in this page.
