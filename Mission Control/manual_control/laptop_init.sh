export ROS_MASTER_URI=http://seadragon:11311
sudo chmod a+rw /dev/input/js0
rosparam set joy_node/dev "/dev/input/js0"
rosrun joy joy_node
