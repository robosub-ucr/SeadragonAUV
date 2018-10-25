#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import String, Empty, Float64, Int16, Bool
from time import time

rospy.init_node('depth_control', anonymous=False)

# State value to be used by PID controller
state = Float64()
state.data = 0

# Depth data container 
depthpwm = Int16()
depthpwm.data = 0

enabled = False

# Publishers to send pwm value to motors feedback
set_depthpwm = rospy.Publisher('/depth_pwm_feedback', Int16, queue_size=10)

def vel_callback(effort_msg):
                depthpwm.data = effort_msg.data
                set_depthpwm.publish(depthpwm)              # Send pwm OUTPUT of PID controller to robot

def depth_callback(depth_msg):
	state.data = depth_msg.data
        set_state.publish(state)             	            # Send depth as FEEDBACK to PID controller

def reset_callback(reset_msg):
        enabled = reset_msg.data
        if enabled == False:
                depthpwm.data = 0
                set_depthpwm.publish(depthpwm)

reset_subscriber = rospy.Subscriber('/depth_control/pid_enable',Bool,reset_callback)
depth_subscriber   = rospy.Subscriber('/depth', Int16, depth_callback)
vel_subscriber   = rospy.Subscriber('control_effort',Float64, vel_callback)
set_state        = rospy.Publisher('state',Float64, queue_size=10)

# Set Rate
rate = rospy.Rate(10)

def depth_controller():

  while not rospy.is_shutdown():
    rate.sleep()

if __name__ == '__main__':
  try:
    depth_controller()
  except rospy.ROSInterruptException:
    pass

