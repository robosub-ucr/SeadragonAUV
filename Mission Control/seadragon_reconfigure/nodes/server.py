#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64, Int16, Bool
from dynamic_reconfigure.server import Server
from seadragon_reconfigure.cfg import setpointConfig

depthEnable_pub = rospy.Publisher('/depth_control/pid_enable', Bool, queue_size=10)
depthPoint_pub  = rospy.Publisher('/depth_control/setpoint', Float64, queue_size=10)

pidEnable_pub = rospy.Publisher('/yaw_control/pid_enable', Bool, queue_size=10)
yawPoint_pub  = rospy.Publisher('/yaw_control/setpoint', Float64, queue_size=10)

yawPwm_pub    = rospy.Publisher('/yaw_pwm', Int16, queue_size=10)
depthPwm_pub  = rospy.Publisher('/depth_pwm', Int16, queue_size=10)
pan_servo_pub = rospy.Publisher('/pan_servo', Int16, queue_size=10)

depthEnable = Bool()
depthPoint = Float64()
	
pidEnable = Bool()
yawPoint = Float64()

yawPwm = Int16()
depthPwm = Int16()

panServo = Int16()

def callback(config, level):
#	rospy.loginfo("""Reconfigure Request: {yawpoint_param},{yawpwm_param}""".format(**config))
	
	#----- Depth Controller
	depthEnable.data = config["depthenable_param"]
	depthEnable_pub.publish(depthEnable)
	depthPoint.data = config["depthpoint_param"]
	depthPoint_pub.publish(depthPoint)
	
	#----- Yaw Controller
	pidEnable.data = config["yawenable_param"]
	pidEnable_pub.publish(pidEnable)	
	yawPoint.data = config["yawpoint_param"]	
	yawPoint_pub.publish(yawPoint)

	#----- Manual	
	yawPwm.data = config["yawpwm_param"]
	yawPwm_pub.publish(yawPwm)	
	
	depthPwm.data = config["depthpwm_param"]
	depthPwm_pub.publish(depthPwm)
	
	#----- pan servo
	panServo.data = config["pan_param"]
	pan_servo_pub.publish(panServo)
	
       	return config 

if __name__ == "__main__":
	rospy.init_node("seadragon_reconfigure")

	srv = Server(setpointConfig, callback)
	rospy.spin()

