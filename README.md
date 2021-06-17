# Franka Emika Panda Simulation in ROS2-Gazebo

This project demonstrates ROS2 functionalities for a simulation of the Panda robotic manipulator by Franka Emika. The simulation includes both a visualization of the Panda in Gazebo as well as in RViz. The control algorithm features an inverse kinematics (IK) joint trajectory planner in order to reach a desired end effector target pose. Joint group position controllers from the ROS2 controllers library are used with an effort interface to the Panda's joints in order to reach the desired joint angle targets computed from IK. Target poses for the end effector are sampled at random continuously.

![Alt Text](media/rviz.gif)

## Installation

First, please ensure that your system meets the basic requirements to build and run the simulation:

- Ubuntu 20.04 LTS
- ROS2 Foxy

This project depends [iDynTree](https://github.com/robotology/idyntree) library from the Italian Institute of Technology. For Debian/Ubuntu, first install the required and optional dependencies:

```
sudo apt-get install libeigen3-dev libxml2-dev coinor-libipopt-dev qtbase5-dev qtdeclarative5-dev qtmultimedia5-dev qml-module-qtquick2 qml-module-qtquick-window2 qml-module-qtmultimedia qml-module-qtquick-dialogs qml-module-qtquick-controls qml-module-qt-labs-folderlistmodel qml-module-qt-labs-settings
```
If you're not running Debian/Ubuntu on your machine, please consult the platform-specific installation instructions [here](https://github.com/robotology/idyntree#installation).

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
## Running the Examples

After the build process has completed, source your colcon workspace:
```
$ source install/setup.bash
```

Launch the simulation:
```
$ ros2 launch panda_ros2_gazebo gazebo.launch.py
```

This will bring up the Gazebo simulation and the RViz visualization. It will also spawn the joint controllers. Note: The robot will just be laying on the floor at the moment since the joint controllers have not actually received any setpoint yet!

To launch the node that will compute the inverse kinematics and publish the setpoints to the joint controllers, open a new terminal and navigate to the root of your colcon workspace. After sourcing your workspace, launch the node with:
```
$ ros2 launch panda_ros2_gazebo bringup.launch.py name:=<demo>
```
where `<demo>` is the name of the demo you wish to run. Note that at the moment there are three demos available:

- `follow` - the Panda will just follow a circular trajectory, 
- `picknplace` - the Panda will pick up cubes that are spawned at random locations in its workspace and attempt to stack them one on top of the other, and
- `teleop` - control the Panda with full teleoperation. See the documentation [here](https://github.com/nicholaspalomo/panda_teleop).

After launching the teleop example, open a new terminal and source your colcon workspace.

You can launch the teleop node control the end effector (including the grippers) after launching the node with:

```
$ ros2 launch panda_teleop panda_teleop_control.launch.py 
```
and following the instructions that appear in the terminal.

![Alt Text](media/pick_and_place.gif)
## Repository Structure

```bash
├── config                      # YAML configuration files
├── description                 # Panda description and world files
    ├── models                  # Robot URDFs and meshes
        └── panda               # Panda URDF and meshes
            └── meshes          # Panda meshes
                ├── collision   # Collision meshes
                └── visual      # Visual meshes
    └── worlds                  # Panda world definitions
├── launch                      # ROS launch scripts
├── panda_ros2_gazebo           # Node definition and IK scripts
    └── examples                # Example nodes to run
        ├── scripts             # IK scripts
            ├── model           # Panda forward/inverse kinematic model
            └── rbd             # Rigid body dynamics utility scripts and class definitions
                └── idyntree    # Python interface for iDyntree bindings
        └── helpers             # Helper scripts for the examples
├── resource                    # Resource directory for ROS2
└── rviz                        # RViz configuration
```

## Contributing

To contribute, please make a fork of this repository and open a pull request if you would like to merge your contributions into this repository.

### Contributors
Nicholas Palomo, ETH Zurich (npalomo@student.ethz.ch)