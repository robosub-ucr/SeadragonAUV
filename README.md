# UCR RoboSub

RoboSub is a student organization at University of California, Riverside. Our goal is to...

## Getting Started

### Prerequisites

* Ubuntu 16.04
* ROS

### Installing

TODO: ROS installation guide

### Setup a ROS workspace



## Running tests

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


## Built with

* ROS
* Jetson Tx2
* Python

## Authors

* Rogelio Vasquez
* Edward Carrasco
* Marios Nicolaides
* Kori Ridenour