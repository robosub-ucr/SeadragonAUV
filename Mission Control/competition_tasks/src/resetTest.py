#!/usr/bin/env python

##------------------------------- IMPORTS ---------------------------------------##

import numpy as np
import rospy
import smach
import smach_ros
from std_msgs.msg import Bool, Float64, Int16


def main():

	# Initialize node with desired node name - ideally task name
	rospy.init_node('reset_test')

	taskReset_publisher	= rospy.Publisher('/reset', Bool, queue_size=1)
	taskReset		= Bool()
	taskReset.data		= True

	taskReset_publisher.publish(taskReset)
	rospy.spin()


if __name__ == '__main__':

	main()
