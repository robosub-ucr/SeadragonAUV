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
WAIT_TIME = 10000

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
		smach.State.__init__(self, outcomes=['done', 'notdone', 'reset'])

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
			return 'done'
		else:
			return 'notdone'

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
		smach.State.__init__(self, outcomes=['done', 'notdone', 'reset'])

		self.torpedo_shoot_publisher = rospy.Publisher('/torpedo_shoot', Bool, queue_size=10)

	def execute(self, userdata):
		# TODO: add 5 second delay before moving onto next state
		
		# Send True, followed by False. To shoot the torpedo once
		shoot = Bool()
		shoot.data = True
		self.torpedo_shoot_publisher.publish(shoot)
		shoot.data = False
		self.torpedo_shoot_publisher.publish(shoot)
		return 'done'

class TimedWaitState(smach.State):
	def __init__(self, wait_time):
		smach.State.__init__(self, outcomes=['done', 'notdone', 'reset'])

		self.timer = 0
		self.wait_time = wait_time

		self.task_complete_publisher = rospy.Publisher('/task_complete', Bool, queue_size=10)

		self.has_reset = False
		self.reset_subscriber = rospy.Subscriber('/reset', Bool, self.reset_callback)

	def reset_callback(self, msg):
		self.has_reset = msg.data

	def execute(self, userdata):
		if self.has_reset:
			self.resetValues()
			return 'reset'

		self.timer += 1
		if self.timer >= self.wait_time:
			task_completed = Bool()
			task_completed.data = True
			self.task_complete_publisher.publish(task_completed)
			self.resetValues()
			return 'done'
		else:
			return 'notdone'

	def resetValues(self):
		self.timer = 0
		self.has_reset = False

class ResetState(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['restart', 'stay'])

	def reset_callback(self, msg):
		self.has_reset = msg.data

	def execute(self, userdata):
		return 'restart'


def main():
	rospy.init_node('torpedo_task_state_machine')
	sm = smach.StateMachine(outcomes=['torpedo_task_complete'])
	sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
	sis.IDLE()

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
		smach.StateMachine.add('IDLE', StartState(), 
			transitions={'ready':'TRACK_BOARD', 'notready':'IDLE'})
		smach.StateMachine.add('TRACK_BOARD', TrackObjectState(board_topic, 0), 
			transitions={'done':'TRACK_HEART', 'notdone':'TRACK_BOARD', 'reset':'RESET'})
		smach.StateMachine.add('TRACK_HEART', TrackObjectState(heart_topic, TORPEDO_Y_OFFSET), 
			transitions={'done':'SHOOT', 'notdone':'TRACK_HEART', 'reset':'RESET'})
		smach.StateMachine.add('SHOOT', ShootTorpedoState(), 
			transitions={'done':'WAIT', 'notdone':'SHOOT', 'reset':'RESET'})
		smach.StateMachine.add('WAIT', TimedWaitState(WAIT_TIME), 
			transitions={'done':'IDLE', 'notdone':'WAIT', 'reset':'RESET'})
		smach.StateMachine.add('RESET', ResetState(), 
			transitions={'restart':'IDLE', 'stay':'RESET'})

	outcome = sm.execute()
	rospy.spin()
	sis.stop()

if __name__ == '__main__':
	main()