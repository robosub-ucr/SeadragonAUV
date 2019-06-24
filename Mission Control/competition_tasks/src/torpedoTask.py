#!/usr/bin/env python

import numpy as np
import rospy
import smach
import smach_ros
from std_msgs.msg import Bool, Float64, Int16

CAMERA_WIDTH = 400
CAMERA_HEIGHT = 300
CENTER_PADDING_X = 5
CENTER_PADDING_Y = 5
YAW_INCREASE = 0.017 # radians
DEPTH_INCREASE = 1
FORWARD_THRUST_INCREASE = 1
AREA_THRESHOLD_LOW = 0.85
AREA_THRESHOLD_HIGH = 0.90
TORPEDO_Y_OFFSET = 10
MAX_FORWARD_THRUST= 280

class StartState(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['ready', 'notready'])

		self.torpedo_task_enabled = False
		rospy.Subscriber('/torpedo_enable', Bool, self.task_enable_callback)

	def task_enable_callback(self, msg):
		self.torpedo_task_enabled = msg.data

	def execute(self, userdata):
		if self.torpedo_task_enabled:
			self.torpedo_task_enabled = False
			return 'ready'
		else:
			return 'notready'

class TrackObjectState(smach.State):
	def __init__(self, obj_topic, yoffset):
		smach.State.__init__(self, outcomes=['completed', 'notcompleted', 'reset', '001', '010', '011', '100', '101', '110'])

		self.yoffset = yoffset
		self.timer = 0

		self.object_x = 0 # in pixels
		self.object_y = 0 # in pixels
		self.object_area = 0 # object width * height
		rospy.Subscriber(obj_topic['x'], Float64, self.object_x_callback)
		rospy.Subscriber(obj_topic['y'], Float64, self.object_y_callback)
		rospy.Subscriber(obj_topic['area'], Float64, self.object_area_callback)
		
		self.yaw_current = 0 # in degrees
		rospy.Subscriber('/yaw_control/state', Float64, self.yaw_callback) # current orientation
		self.yaw_publisher = rospy.Publisher('yaw_control/setpoint', Float64, queue_size=10) # desired orientation

		self.depth_current = 0 # in inches
		rospy.Subscriber('/depth_control/state', Float64, self.depth_callback)
		self.depth_publisher = rospy.Publisher('/depth_control/setpoint', Float64, queue_size=10)

		self.forward_thrust_publisher = rospy.Publisher('/yaw_pwm', Int16, queue_size=10)
		self.forward_thrust = 0

		self.has_reset = False
		self.reset_subscriber = rospy.Subscriber('/reset', Bool, self.reset_callback)

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
	def reset_callback(self, msg):
		self.has_reset = msg.data

	def execute(self, userdata):
		self.timer = self.timer + 1
		if self.has_reset:
			self.resetValues()
			return 'reset'

		is_object_x_centered = self.adjust_yaw() 
		is_object_y_centered = self.adjust_depth()
		is_object_area_in_threshold = False

		if is_object_x_centered and is_object_y_centered:
			is_object_area_in_threshold = self.adjust_position() 

		# go to next state if the object is at the center of the camera frame and within certain distace of the submarine
		if is_object_x_centered and is_object_y_centered and is_object_area_in_threshold:
			self.resetValues()
			return 'completed'
		elif not is_object_y_centered and not is_object_y_centered and is_object_area_in_threshold:
			return '001'
		elif not is_object_y_centered and is_object_y_centered and not is_object_area_in_threshold:
			return '010'
		elif not is_object_y_centered and is_object_y_centered and is_object_area_in_threshold:
			return '011'
		elif is_object_y_centered and not is_object_y_centered and not is_object_area_in_threshold:
			return '100'
		elif is_object_y_centered and not is_object_y_centered and is_object_area_in_threshold:
			return '101'
		elif is_object_y_centered and is_object_y_centered and not is_object_area_in_threshold:
			return '110'
		else:
			return 'notcompleted'

	def resetValues(self):
		self.object_x = 0 # in pixels
		self.object_y = 0
		self.object_area = 0 # object width * height
		self.yaw_current = 0 # in degrees
		self.depth_current = 0 # in inches
		self.forward_thrust = 0
		self.has_reset = False
		self.timer = 0


	def adjust_yaw(self):
		# rotate yaw until x is within center +/- padding
		new_yaw = Float64() # 0 to 180 degrees (counterclockwise) or -180 degrees (clockwise)
		if self.object_x > CAMERA_WIDTH/2 + CENTER_PADDING_X:
			new_yaw.data = self.yaw_current - YAW_INCREASE
			self.yaw_publisher.publish(new_yaw)
			return False
		elif self.object_x < CAMERA_WIDTH/2 - CENTER_PADDING_X:
			new_yaw.data = self.yaw_current + YAW_INCREASE
			self.yaw_publisher.publish(new_yaw)
			return False
		else:
			return True

	def adjust_depth(self):
		# change depth until y is within center +/- padding
		new_depth = Float64() # 0 to 60 inches
		if self.object_y > CAMERA_HEIGHT/2 + self.yoffset + CENTER_PADDING_Y:
			new_depth.data = self.depth_current + DEPTH_INCREASE
			self.depth_publisher.publish(new_depth)
			return False
		elif self.object_y < CAMERA_HEIGHT/2 + self.yoffset - CENTER_PADDING_Y:
			new_depth.data = self.depth_current - DEPTH_INCREASE
			self.depth_publisher.publish(new_depth)
			return False
		else:
			return True

	def adjust_position(self):
		# move forward/backward until object area is within threshold
		if self.object_area/(CAMERA_WIDTH*CAMERA_HEIGHT) < AREA_THRESHOLD_LOW:
			self.change_forward_thrust(FORWARD_THRUST_INCREASE)
			return False
		elif self.object_area/(CAMERA_WIDTH*CAMERA_HEIGHT) > AREA_THRESHOLD_HIGH:
			self.change_forward_thrust(-FORWARD_THRUST_INCREASE)
			return False
		else:
			return True

	def change_forward_thrust(self, amount):
		# only increase/decrease thrust every 200 ticks
		if self.timer % 200 != 0:
			return

		# ensure thrust cannot exceed 280 or -280
		self.forward_thrust = self.forward_thrust + amount
		if self.forward_thrust > MAX_FORWARD_THRUST:
			self.forward_thrust = MAX_FORWARD_THRUST
		elif self.forward_thrust < -MAX_FORWARD_THRUST:
			self.forward_thrust = -MAX_FORWARD_THRUST

		# Publish the new forward thrust
		new_forward_thrust = Int16()
		new_forward_thrust.data = self.forward_thrust
		self.forward_thrust_publisher.publish(new_forward_thrust)


class ShootTorpedoState(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['completed', 'notcompleted', 'reset'])

		self.torpedo_shoot_publisher = rospy.Publisher('/torpedo_shoot', Bool, queue_size=10)
		self.task_complete_publisher = rospy.Publisher('/task_complete', Bool, queue_size=10)

	def execute(self, userdata):
		shoot = Bool()
		shoot.data = True
		self.torpedo_shoot_publisher.publish(shoot) # publishes True
		self.task_complete_publisher.publish(shoot)
		return 'completed'


class ResetState(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['restart', 'stay'])

		# self.has_reset = True
		# rospy.Subscriber('/reset', Bool, self.reset_callback)

	def reset_callback(self, msg):
		self.has_reset = msg.data

	def execute(self, userdata):
		# if self.has_reset:
		# 	return 'stay'
		# else:
		# 	self.has_reset = True
		# 	return 'restart'
		return 'restart'


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
		smach.StateMachine.add('TrackBoardState', TrackObjectState(board_topic, 0), transitions={'completed':'TrackHeartState', 'notcompleted':'TrackBoardState', 'reset':'ResetState', '001':'TrackBoardState', '010':'TrackBoardState', '011':'TrackBoardState', '100':'TrackBoardState', '101':'TrackBoardState', '110':'TrackBoardState'})
		smach.StateMachine.add('TrackHeartState', TrackObjectState(heart_topic, TORPEDO_Y_OFFSET), transitions={'completed':'ShootTorpedoState', 'notcompleted':'TrackHeartState', 'reset':'ResetState', '001':'TrackHeartState', '010':'TrackHeartState', '011':'TrackHeartState', '100':'TrackHeartState', '101':'TrackHeartState', '110':'TrackHeartState'})
		smach.StateMachine.add('ShootTorpedoState', ShootTorpedoState(), transitions={'completed':'StartState', 'notcompleted':'ShootTorpedoState', 'reset':'ResetState'})
		smach.StateMachine.add('ResetState', ResetState(), transitions={'restart':'StartState', 'stay':'ResetState'})

	outcome = sm.execute()
	rospy.spin()
	sis.stop()

if __name__ == '__main__':
	main()