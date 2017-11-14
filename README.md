# 415autopilot_ws
A full rosplane/rosflight workspace for class demos and getting started

## Getting Started

These instructions will get you a copy of the project up and running on your local machine.

### Prerequisites

You will need to have installed Ubuntu Linux 16.04 on your computer and have ROS Kinetic installed and working.

### Installing and Building

Type the following into a terminal:

```
cd ~
git clone https://github.com/byuflowlab/415autopilot_ws.git
cd 415autopilot_ws
git submodule update --init --recursive
source /opt/ros/kinetic/setup.bash
catkin_make
```

### Running the autopilot simulator

Type the following into a terminal:

```
source ~/repos/415autopilot_ws/devel/setup.bash
roslaunch autopilot_me415 autopilot_simulator.launch
```

Click the play button in the gazebo window lower toolbar.
The plane should take off and do nothing.
