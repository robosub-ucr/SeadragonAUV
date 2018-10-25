#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import String, Empty, Float64, Int16, Bool
from sensor_msgs.msg import Imu
from time import time
import geometry_msgs.msg
import tf

rospy.init_node('yaw_control', anonymous=False)

# State value to be used by PID controller
state = Float64()
state.data = 0

# Yaw data container 
yawpwm = Int16()
yawpwm.data = 0

enabled = False

# Publishers to send pwm value to motors feedback
set_yawpwm = rospy.Publisher('/yaw_pwm_feedback', Int16, queue_size=10)

def vel_callback(effort_msg):
		yawpwm.data = effort_msg.data

		set_yawpwm.publish(yawpwm)		# Send pwm OUTPUT of PID controller to robot

def imu_callback(imu_msg):
        q = np.array([imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w])

        orientation = tf.transformations.euler_from_quaternion(q)

	state.data = orientation[2] 
	set_state.publish(state)			# Send yaw as FEEDBACK to PID controller

def reset_callback(reset_msg):
	enabled = reset_msg.data
	if enabled == False:
		yawpwm.data = 0
		set_yawpwm.publish(yawpwm)

reset_subscriber = rospy.Subscriber('/yaw_control/pid_enable',Bool,reset_callback)
imu_subscriber   = rospy.Subscriber('/imu/data', Imu, imu_callback)
vel_subscriber   = rospy.Subscriber('control_effort',Float64, vel_callback)
set_state        = rospy.Publisher('state',Float64, queue_size=10)

# Set Rate
rate = rospy.Rate(10)

def yaw_controller():

  while not rospy.is_shutdown():
    rate.sleep()

if __name__ == '__main__':
  try:
    yaw_controller()
  except rospy.ROSInterruptException:
    pass
