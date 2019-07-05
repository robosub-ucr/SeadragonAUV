# UCR RoboSub

We are the University of California, Riverside's competitive autonomous underwater vehicle project, competing internationally in RoboNation's RoboSub Competition.

Each year we construct a submarine capable of completing a number of aquatic objectives entirely on its own. The sheer volume of design and construction needed requires a large interdisciplinary team working collaboratively to share expertise and comunication throughout the process.

Sponsored by UCR's IEEE and ASME chapters, the project exposes all students to utilizing their coursework skills, working with others outside their major, and granting experience with a large-scale project with real-world applications. For the 2018-2019 academic year, UCR RoboSub has about \~30 active members, across almost every engineering major.

### Software

The Software team gives the submarine life, creating the artificial intelligence, computer vision, and software interface modules that allow the robot to see, sense, and move. We use Robot Operating System (ROS), Python, and C/C++ for our controls.

## Getting Started

### Prerequisites

* Ubuntu 16.04
* Robot Operating System (ROS) 

### Installation

TODO

### Setup a ROS workspace

TODO

### Running tests

To start ROS: 
1. Open a terminal
2. Type `roscore`
3. Go to the `SeadragonAUV/Mission Control/competition_tasks/src/` folder.
4. Open another terminal.
5. Type `python master.py` or any other python file. The program corresponding task should start running. It should look similar to this:
`[INFO] [123456789.987654]: State machine transitioning 'IDLE':'notready' --> 'IDLE'`
6. Open another terminal.
7. To publish a topic, type `rostopic pub ` followed by any of the following topics:
```
/depth std_msgs/Int16 “data: 1”
/yaw_control/setpoint std_msgs/Float64 “data: 1.0”
/yaw_control/state std_msgs/Float64 “data: 1.0”
```
