# Franka Emika Panda Simulation in ROS2-Gazebo

This project demonstrates ROS2 functionalities for a simulation of the Panda robotic manipulator by Franka Emika. The simulation includes both a visualization of the Panda in Gazebo as well as in RViz. The control algorithm features an inverse kinematics (IK) joint trajectory planner in order to reach a desired end effector target pose. Joint group position controllers from the ROS2 controllers library are used with an effort interface to the Panda's joints in order to reach the desired joint angle targets computed from IK. Target poses for the end effector are sampled at random continuously.

## Installation

First, please ensure that your system meets the basic requirements to build and run the simulation:

- Ubuntu 20.04 LTS
- ROS2 Foxy

This project depends [iDynTree](https://github.com/robotology/idyntree) library from the Italian Institute of Technology. DO NOT install it via pip; rather install it from source. It is also recommended that you do this outside your colcon workspace. For Debian/Ubuntu, first install the required and optional dependencies:

```
sudo apt-get install libeigen3-dev libxml2-dev coinor-libipopt-dev qtbase5-dev qtdeclarative5-dev qtmultimedia5-dev qml-module-qtquick2 qml-module-qtquick-window2 qml-module-qtmultimedia qml-module-qtquick-dialogs qml-module-qtquick-controls qml-module-qt-labs-folderlistmodel qml-module-qt-labs-settings
```

Then, proceed to build and install iDynTree as follows. You also need a working `swig` installation on your machine as well as an up-to-date installation of `scipy`. Don't forget to use the `IDYNTREE_USES_IPOPT` flag when running the `cmake` command!

```
$ git clone https://github.com/robotology/idyntree
$ cd idyntree
$ mkdir build && cd build
$ cmake -DIDYNTREE_USES_IPOPT ..
$ make
$ [sudo] make install
```

Now, clone this repository into your colcon workspace:
```
$ mkdir -p ~/workspace/src
$ cd ~/workspace/src
$ git clone https://github.com/nicholaspalomo/panda_ros2_gazebo.git
```

Before building the project, you need to also import the repositories specified in the `workspace.repos`:
```
$ vcs import < panda_ros2_gazebo/workspace.repos
```

Now you can proceed to build the code:
```
$ cd ..
$ colcon build --merge-install
```

After the build process has completed, source your colcon workspace:
```
$ source install/setup.bash
```

Launch the simulation:
```
$ ros2 launch panda_ros2_gazebo panda_ros2_gazebo.launch.py
```

This will bring up the Gazebo simulation and the RViz visualization. It will also spawn the joint controllers. Note: The robot will just be laying on the floor at the moment since the joint controllers have not actually received any setpoint yet!

To launch the node that will compute the inverse kinematics and publish the setpoints to the joint controllers, open a new terminal and navigate to the root of your colcon workspace. After sourcing your workspace, launch the node with:
```
$ ros2 launch panda_ros2_gazebo panda_pick_and_place.launch.py
```

You will see the arm start to move around, going from setpoint to setpoint.

## Repository Structure



## Contributing

To contribute, please make a fork of this repository and open a pull request if you would like to merge your contributions into this repository.

### Contributors
Nicholas Palomo, ETH Zurich (npalomo@student.ethz.ch)