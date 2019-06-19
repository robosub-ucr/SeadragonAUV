#!/usr/bin/env python

import numpy as np
import rospy
import smach
import smach_ros
from std_msgs.msg import Bool, Float64, Int16

CAMERA_WIDTH = 400
CAMERA_HEIGHT = 300
PADDING_X = 5
PADDING_Y = 5
YAW_INCREASE = 1
DEPTH_INCREASE = 1
FORWARD_THRUST_INCREASE = 1
AREA_THRESHOLD_LOW = 0.85
AREA_THRESHOLD_HIGH = 0.90

class StartState(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['ready', 'notready'])

		self.torpedo_task_subscriber = rospy.Subscriber('/torpedo_enable', Bool, self.torpedo_task_callback)
		self.torpedoEnabled = False

	def torpedo_task_callback(self, msg):
		self.torpedoEnabled = msg.data

	def execute(self, userdata):
		if self.torpedoEnabled:
			return 'ready'
		else:
			return 'notready'

class TrackObjectState(smach.State):
	def __init__(self, obj_topic):
		smach.State.__init__(self, outcomes=['completed', 'notcompleted'])

		self.object_x_subscriber = rospy.Subscriber(obj_topic['x'], Float64, self.object_x_callback)
		self.object_y_subscriber = rospy.Subscriber(obj_topic['y'], Float64, self.object_y_callback)
		self.object_area_subscriber = rospy.Subscriber(obj_topic['area'], Float64, self.object_area_callback)
		self.object_x = 0 # in pixels
		self.object_y = 0
		self.object_area = 0 # percentage

		self.yaw_current_subscriber = rospy.Subscriber('/yaw_control/state', Float64, self.yaw_callback) # current orientation
		self.yaw_setpoint_publisher = rospy.Publisher('yaw_control/setpoint', Float64, queue_size=10) # desired orientation
		self.yaw_current = 0 # in degrees

		self.depth_subscriber = rospy.Subscriber('/depth_control/state', Float64, self.depth_callback)
		self.depth_publisher = rospy.Publisher('/depth_control/setpoint', Float64, queue_size=10)
		self.depth_current = 0 # in inches

		self.forward_thrust_publisher	= rospy.Publisher('/yaw_pwm', Int16, queue_size=10)
		self.forward_thrust = 0

	def object_x_callback(self, msg):
		self.object_x = msg.data

	def object_y_callback(self, msg):
		self.object_y = msg.data

	def object_area_callback(self, msg):
		self.object_area = msg.data

	def yaw_callback(self, msg):
		self.yaw_current = msg.data

	def depth_callback(self, msg):
		self.depth_current = msg.data

	def execute(self, userdata):
		is_object_x_centered = self.adjust_yaw() 
		is_object_y_centered = self.adjust_depth()
		is_object_area_in_threshold = False

		if is_object_x_centered and is_object_y_centered:
			is_object_area_in_threshold = self.adjust_position() 

		# go to next state if the object is at the center of the camera frame and within certain distace of the submarine
		if is_object_x_centered and is_object_y_centered and is_object_area_in_threshold:
			self.resetValues()
			return 'completed'
		else:
			return 'notcompleted'

	def adjust_yaw(self):
		# rotate yaw until x is within center +/- padding
		new_yaw = Float64() # 0 to 180 degrees (counterclockwise) or -180 degrees (clockwise)
		if self.object_x > CAMERA_WIDTH/2 + PADDING_X:
			new_yaw.data = self.yaw_current - YAW_INCREASE
			self.yaw_setpoint_publisher.publish(new_yaw)
			return False
		elif self.object_x < CAMERA_WIDTH/2 - PADDING_X:
			new_yaw.data = self.yaw_current + YAW_INCREASE
			self.yaw_setpoint_publisher.publish(new_yaw)
			return False
		else:
			return True

	def adjust_depth(self):
		# change depth until y is within center +/- padding
		new_depth = Float64() # 0 to 60 inches
		if self.object_y > CAMERA_HEIGHT/2 + PADDING_Y:
			new_depth.data = self.depth_current - DEPTH_INCREASE
			self.depth_publisher.publish(new_depth)
			return False
		elif self.object_y < CAMERA_HEIGHT/2 - PADDING_Y:
			new_depth.data = self.depth_current + DEPTH_INCREASE
			self.depth_publisher.publish(new_depth)
			return False
		else:
			return True

	def adjust_position(self):
		# move forward/backward until object area is within threshold
		new_forward_thrust = Int16() # 0 to 280
		if self.object_area/(CAMERA_WIDTH*CAMERA_HEIGHT) < AREA_THRESHOLD_LOW:
			self.forward_thrust = self.forward_thrust + FORWARD_THRUST_INCREASE
			new_forward_thrust.data = self.forward_thrust
			self.forward_thrust_publisher.publish(new_forward_thrust)
			return False
		elif self.object_area/(CAMERA_WIDTH*CAMERA_HEIGHT) > AREA_THRESHOLD_HIGH:
			self.forward_thrust = self.forward_thrust - FORWARD_THRUST_INCREASE
			new_forward_thrust.data = self.forward_thrust
			self.forward_thrust_publisher.publish(new_forward_thrust)
			return False
		else:
			return True

	def change_forward_thrust(self, amount):
		pass

	def resetValues(self):
			self.object_x = 0
			self.object_y = 0
			self.object_area = 0
			self.yaw_current = 0
			self.depth_current = 0
		

class ShootTorpedoState(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['completed', 'notcompleted'])

		self.torpedo_shoot_publisher = rospy.Publisher('/torpedo_shoot', Bool, queue_size=10)

	def execute(self, userdata):
		shoot = Bool()
		shoot.data = True
		self.torpedo_shoot_publisher.publish(shoot)
		return 'completed'


def main():
	rospy.init_node('torpedo_task_state_machine')
	sm = smach.StateMachine(outcomes=['torpedo_task_complete'])
	sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
	sis.start()

	board_topic = {
		'x': '/torpedo_board_x',
		'y': '/torpedo_board_y',
		'area': '/torpedo_board_area'
	}

	heart_topic = {
		'x': '/torpedo_heart_x',
		'y': '/torpedo_heart_y',
		'area': '/torpedo_heart_area'
	}

	with sm:
		smach.StateMachine.add('StartState', StartState(), transitions={'ready':'TrackBoardState', 'notready':'StartState'})
		smach.StateMachine.add('TrackBoardState', TrackObjectState(board_topic), transitions={'completed':'TrackHeartState', 'notcompleted':'TrackBoardState'})
		smach.StateMachine.add('TrackHeartState', TrackObjectState(heart_topic), transitions={'completed':'ShootTorpedoState', 'notcompleted':'TrackHeartState'})
		smach.StateMachine.add('ShootTorpedoState', ShootTorpedoState(), transitions={'completed':'StartState', 'notcompleted':'ShootTorpedoState'})

	outcome = sm.execute()
	rospy.spin()
	sis.stop()

if __name__ == '__main__':
	main()