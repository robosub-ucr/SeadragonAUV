# Seadragon AUV

The Seadragon AUV (Autonomous Underwater Vehicle) is built by the University of California, Riverside's student organization: RoboSub UCR. The AUV was built to participate in the RoboNation's international competition, RoboSub. The AUV completes a number of aquatic objectives, such as object identification and interaction, dropping markers in targets, launching torpedoes at targets, and responding to sound signals, all without human interaction.

The RoboSub UCR's software team gives the submarine life, creating the artificial intelligence, computer vision, and software interface modules that allow the robot to see, sense, and move. We use Robot Operating System (ROS), Python, and C/C++ for our controls.

## Getting Started

### Prerequisites

* Ubuntu 18.04
* Python 2
* C++
* ROS Melodic
* OpenCV

### Installation

TODO

### Setup a ROS workspace

TODO


### Initializing ROS and nodes

Initialize ROS master node:
```
roscore
```

Initialie AHRS node:
```
sudo chmod 666 /dev/ttyACM0
rosrun myahrs_driver myahrs_driver _port:=/dev/ttyACM0
```

Initialize STM32:
```
sudo chmod 666 /dev/ttyACM2
rosrun rosserial_python serial_node.py _port:=/dev/ttyACM2
```

Launch GUI:
```
roslaunch motor_controllers attitude_control.launch
```

Find which device (jsX) is the Joystick:
```
ls /dev/input/
sudo jstest /dev/input/jsX
```

Initialize Joystick node:
```
ls -l /dev/input/jsX
sudo chmod a+rw /dev/input/jsX
rosparam set joy_node/dev "/dev/input/jsX"
rosrun joy joy_node
```
Note: A joystick controller must be connected to the computer.

### Testing publishers, subscribers, and state machines

1. Open a terminal on any directory and type `roscore`
2. Go to the `SeadragonAUV/Mission Control/competition_tasks/src/` folder.
3. Open a new terminal for each of the following:
```
python master.py
python gateTask.py
python buoyTask.py
python torpedoTask.py
```
Each terminal should constantly print out a message similar to this one: `[INFO] [123456789.987654]: State machine transitioning 'IDLE':'notready' --> 'IDLE'`

5. Open another terminal. Type of each these commands, each followed by ctrl-C:
```
rostopic pub /depth std_msgs/Int16 "data: 13"
rostopic pub /depth_control/state std_msgs/Int16 "data: 18"
rostopic pub /yaw_control/state std_msgs/Float64 "data: 1.57"
rostopic pub /gate_x std_msgs/Float64 "data: 200.0"
rostopic pub /gate_y std_msgs/Float64 "data: 150.0"
rostopic pub /gate_area std_msgs/Float64 "data: 15600.0"
rostopic pub /yaw_control/state std_msgs/Float64 "data: 1.57"
rostopic pub /yaw_control/state std_msgs/Float64 "data: 1.485"
rostopic pub /depth_control/state std_msgs/Int16 "data: 60"
rostopic pub /yaw_control/state std_msgs/Float64 "data: 0.765"
```
Tip: After typing the /topic_name and SPACE, you can press Tab twice to autofill the rest of the command.
These commands will move the master.py state machine from IDLE --> EXECUTE and the gateTask.py state machine through all of its states.

### Jetson + ROS tests
1. Open a terminal and `roscore`
2. Open another terminal and type:
```
sudo chmod 666 /dev/ttyACM0
rosrun rosserial_python serial_node.py /dev/ttyACM0
```
Note: if ACM0 doesnt work (an error occurs), try ACM1 or type `ls /dev/ttyACM` and tab to see which file # exists.

