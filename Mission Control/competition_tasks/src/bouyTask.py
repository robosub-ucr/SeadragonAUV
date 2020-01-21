#!/usr/bin/env python

import numpy as np
import rospy
import smach
import smach_ros
from std_msgs.msg import Bool, Float64, Int16

import seadragon_states as sd
import time

CAMERA_WIDTH = 400			#pixel width of display
CAMERA_HEIGHT = 300			#pixel height of display 
CENTER_PADDING_X = 15			#Target box length from the center
CENTER_PADDING_Y = 15			#Target box height from the center
YAW_INCREASE = 0.017			#Minimum in radians that the sub can turn (1 degree)
DEPTH_STEP = 1				#Minimum depth level (in inches)
FORWARD_THRUST_INCREASE = 60 		#Value that the sub will move forward to increase speed (bursts of acceleration)
AREA_THRESHOLD_LOW = 0.15 		#The threshold for the minimum closeness the object HAS to be
AREA_THRESHOLD_HIGH = 0.2		#The threshold for the maximum closeness the object COULD to be
TORPEDO_Y_OFFSET = 10 			#Offset of the torpedo position and the camera (target box). 
MAX_FORWARD_THRUST= 120 		#Maximum that the state machines will allow the thrusters to go.

class StartState(smach.State):	# This one is pretty self explanatory
	def __init__(self):
		smach.State.__init__(self, outcomes=['ready', 'notready'])

		self.enabled = False
		rospy.Subscriber('/buoy_enable', Bool, self.enabled_callback)

	def enabled_callback(self, msg):
		self.enabled = msg.data

	def execute(self, userdata):
		if self.enabled:
			self.enabled = False
			return 'ready'
		else:
			return 'notready'

class TrackObjectState(smach.State): #
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
		self.yaw_publisher = rospy.Publisher('/yaw_control/setpoint', Float64, queue_size=10) # desired orientation

		self.depth_current = 0 # in inches
		rospy.Subscriber('/depth', Int16, self.depth_callback)
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
		if self.has_reset:
			self.resetValues()
			return 'reset'

		self.timer = self.timer + 1

		is_object_x_centered = self.adjust_yaw() 
		is_object_y_centered = self.adjust_depth()
		is_object_area_in_threshold = False

		if is_object_x_centered and is_object_y_centered:
			is_object_area_in_threshold = self.adjust_position()
		else:
			new_forward_thrust = Int16()
			new_forward_thrust.data = 0
			self.forward_thrust_publisher.publish(new_forward_thrust)

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
		if self.object_y > CAMERA_HEIGHT/2 + self.yoffset + CENTER_PADDING_Y:		# Object is below us
			new_depth.data = self.depth_current + DEPTH_STEP			# So increase depth
			self.depth_publisher.publish(new_depth)
			return False
		elif self.object_y < CAMERA_HEIGHT/2 + self.yoffset - CENTER_PADDING_Y:		# Object is above us
			new_depth.data = self.depth_current - DEPTH_STEP			# So decrease depth
			self.depth_publisher.publish(new_depth)
			return False
		else:
			return True

	def adjust_position(self):
		# move forward/backward until object area is within threshold
		if self.object_area < AREA_THRESHOLD_LOW:
			self.change_forward_thrust(FORWARD_THRUST_INCREASE)
			return False
		elif self.object_area > AREA_THRESHOLD_HIGH:
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

class ChangeDepthState(smach.State):
	def __init__(self, depthAdjust, threshold):
		smach.State.__init__(self, outcomes=['done', 'notdone', 'reset'])

		# Local Change Depth State variables
		self.depthAdjust 	= depthAdjust
		self.threshold 		= threshold
		self.depth_current 	= 0 # in inches
		self.target_depth	= 0 # in inches
		self.target_set 	= False
		self.has_reset 		= False
		self.new_depth_target 	= Float64() # ROS depth msg container

		# Subscribers
		rospy.Subscriber('/depth', Int16, self.depth_callback)
		self.reset_subscriber = rospy.Subscriber('/reset', Bool, self.reset_callback)

		self.depth_publisher = rospy.Publisher('/depth_control/setpoint', Float64, queue_size=10)

	def depth_callback(self, msg):
		self.depth_current = msg.data
	def reset_callback(self, msg):
		self.has_reset = msg.data

	def reset_variables(self):
		self.has_reset     = False
		self.target_set    = False
		self.depth_current = 0
		self.target_depth  = 0

	def execute(self, userdata):
		print("depth_state", self.depth_current, "depth_setpoint", self.depthAdjust)
		if self.has_reset:
			self.reset_variables()
			return 'reset'

		if self.target_set == False and self.depth_current != 0:
			self.target_depth = self.depth_current + self.depthAdjust	# target_depth only for transition calculation	
			self.new_depth_target.data = self.target_depth			# ROS MSG container
			self.depth_publisher.publish(self.new_depth_target_data)	
			self.target_set = True
		
		# Sub stays in change depth state until it reaches its target depth or while it hasn't set a new depth target
		if (abs(self.depth_current - self.target_depth) > self.threshold) or (self.target_set == False):
			return 'notdone'
		else:
			self.reset_variables()
			return 'done'


class RotateYawState(smach.State):
	def __init__(self, yaw_change, variance):
		smach.State.__init__(self, outcomes=['done', 'notdone', 'reset'])

		self.yaw_published = False
		self.yaw_change = yaw_change
		self.yaw_variance = variance
		self.yaw_target = Float64()
		self.timer = 0

		self.yaw_current = 0 # in degrees, None if the callback hasnt triggered yet
		rospy.Subscriber('/yaw_control/state', Float64, self.yaw_callback) # current orientation
		self.yaw_publisher = rospy.Publisher('/yaw_control/setpoint', Float64, queue_size=10) # desired orientation

		self.has_reset = False
		self.reset_subscriber = rospy.Subscriber('/reset', Bool, self.reset_callback)

	def yaw_callback(self, msg):
		self.yaw_current = msg.data
	def reset_callback(self, msg):
		self.has_reset = msg.data

	def execute(self, userdata):
		print("yaw_state", self.yaw_current, "yaw_setpoint", self.yaw_current + self.yaw_change)
		if self.has_reset:
			self.resetValues()
			return 'reset'

		self.timer += 1
		if not self.yaw_published:
			self.yaw_target.data = self.yaw_current + self.yaw_change
			self.yaw_publisher.publish(self.yaw_target)
			self.yaw_published = True
			return 'notdone'
		elif abs(self.yaw_current - self.yaw_target.data) < self.yaw_variance:
			return 'done'
		else:
			return 'notdone'

	def resetValues(self):
		self.yaw_current = 0
		self.has_reset = False
		self.yaw_published = False
		self.timer = 0


class MoveForwardState(smach.State):
	def __init__(self, duration, isForward):
		smach.State.__init__(self, outcomes=['done', 'notdone', 'reset'])

		self.thrust_timer = 0
		self.timer = 0
		self.duration = duration
		self.isForward = isForward

		self.forward_thrust_publisher = rospy.Publisher('/yaw_pwm', Int16, queue_size=10)
		self.forward_thrust = 0

		self.has_reset = False
		self.reset_subscriber = rospy.Subscriber('/reset', Bool, self.reset_callback)

	def reset_callback(self, msg):
		self.has_reset = msg.data

	def execute(self, userdata):
		print("timer (%)", self.timer/self.duration, "forward?", self.isForward)
		if self.has_reset:
			self.resetValues()
			return 'reset'

		self.thrust_timer = self.thrust_timer + 1
		self.timer = self.timer + 1

		if self.timer >= self.duration:
			self.resetValues()
			return 'done'
		else:
			if self.isForward:
				self.change_forward_thrust(FORWARD_THRUST_INCREASE)
			else:
				self.change_forward_thrust(-FORWARD_THRUST_INCREASE)
			return 'notdone'

	def change_forward_thrust(self, amount):
		# only increase/decrease thrust every 200 ticks
		if self.thrust_timer % 200 != 0:
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

	def resetValues(self):
		self.thrust_timer = 0
		self.timer = 0
		self.forward_thrust = 0
		self.has_reset = False


class CompletedState(smach.State):
	def __init__(self, topic_name):
		smach.State.__init__(self, outcomes=['done'])
		self.completed_task_publisher = rospy.Publisher(topic_name, Bool, queue_size=10)

	def execute(self, userdata):
		completed = Bool()
		completed.data = True
		self.completed_task_publisher.publish(completed)
		return 'done'


class ResetState(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['done', 'notdone'])

	def execute(self, userdata):
		# currently this state does nothing
		return 'done'


def main():
	rospy.init_node('buoy_task_state_machine')
	sm = smach.StateMachine(outcomes=['buoy_task_complete'])
	sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
	sis.start()

	buoy_flat_topic = {
		'x': '/buoy_flat_x',
		'y': '/buoy_flat_y',
		'area': '/buoy_flat_area'
	}

	buoy_triangle_topic = {
		'x': '/buoy_triangle_x',
		'y': '/buoy_triangle_y',
		'area': '/buoy_triangle_area'
	}

	TOUCH_FLAT_TIMER 	= 10000 # time required (in ticks) to touch the flat buoy
	MOVE_BACK_1_TIMER 	= 6000  # time required (in ticks) to move back, away from flat buoy
	MOVE_FORWARD_TIMER 	= 10000 # time required (in ticks) to move past the flat buoy
	TOUCH_TRIANGLE_TIMER = 10000 # time required (in ticks) to touch the triangle buoy
	MOVE_BACK_2_TIMER 	= 6000 # time required (in ticks) to move back, away from triangle buoy

	BUOY_ABOVE_DEPTH 	= 2*12  # Adjusts sub to move 2 ft up
	BUOY_BELOW_DEPTH 	= 3.5*12   # Adjusts sub to move 3.5 ft down
	TORPEDO_BOARD_DEPTH = 3*12   # Adjusts sub to move 3 ft down
	DEPTH_VARIANCE 		= 2    # 2 inch

	YAW_BUOY_BACK 		= 3.14 # the yaw (in radians) to make sub turn 180 degrees to face back buoys
	YAW_TORPEDO_TASK 	= 0.5  # the yaw (in radians) to face the torpedo task
	YAW_VARIANCE 		= 0.1  # in radians

	with sm:
		smach.StateMachine.add('IDLE', StartState(), 
			transitions={'ready':'TOUCH_FLAT', 'notready':'IDLE'})
			#transitions={'ready':'TRACK_FLAT', 'notready':'IDLE'})

		#smach.StateMachine.add('TRACK_FLAT', sd.TrackObject(buoy_flat_topic), 
		#	transitions={'done':'TOUCH_FLAT', 'notdone':'TRACK_FLAT', 'reset':'RESET'})

		smach.StateMachine.add('TOUCH_FLAT', sd.MoveForwardTimed(TOUCH_FLAT_TIMER, True), 
			transitions={'done':'MOVE_BACK_1', 'notdone':'TOUCH_FLAT', 'reset':'RESET'})

		smach.StateMachine.add('MOVE_BACK_1', sd.MoveForwardTimed(MOVE_BACK_1_TIMER, False), 
			transitions={'done':'MOVE_UP', 'notdone':'MOVE_BACK_1', 'reset':'RESET'})

		smach.StateMachine.add('MOVE_UP', sd.ChangeDepthToTarget(BUOY_ABOVE_DEPTH), 
			transitions={'done':'MOVE_FORWARD', 'notdone':'MOVE_UP', 'reset':'RESET'})

		smach.StateMachine.add('MOVE_FORWARD', sd.MoveForwardTimed(MOVE_FORWARD_TIMER, True), 
			transitions={'done':'MOVE_DOWN', 'notdone':'MOVE_FORWARD', 'reset':'RESET'})

		smach.StateMachine.add('MOVE_DOWN', sd.ChangeDepthToTarget(BUOY_BELOW_DEPTH), 
			transitions={'done':'TURN_AROUND', 'notdone':'MOVE_DOWN', 'reset':'RESET'})

		smach.StateMachine.add('TURN_AROUND', sd.Rotate180(YAW_BUOY_BACK), 
			transitions={'done':'TOUCH_TRIANGLE', 'notdone':'TURN_AROUND', 'reset':'RESET'})
		#	transitions={'done':'TRACK_TRIANGLE', 'notdone':'TURN_AROUND', 'reset':'RESET'})

		#smach.StateMachine.add('TRACK_TRIANGLE', TrackObjectState(buoy_triangle_topic, 0), 
		#	transitions={'done':'TOUCH_TRIANGLE', 'notdone':'TRACK_TRIANGLE', 'reset':'RESET'})

		smach.StateMachine.add('TOUCH_TRIANGLE', sd.MoveForwardTimed(TOUCH_TRIANGLE_TIMER, True), 
			transitions={'done':'MOVE_BACK_2', 'notdone':'TOUCH_TRIANGLE', 'reset':'RESET'})

		smach.StateMachine.add('MOVE_BACK_2', sd.MoveForwardTimed(MOVE_BACK_2_TIMER, False), 
			transitions={'done':'FACE_TORPEDO_TASK', 'notdone':'MOVE_BACK_2', 'reset':'RESET'})

		smach.StateMachine.add('FACE_TORPEDO_TASK', RotateYawState(YAW_TORPEDO_TASK, YAW_VARIANCE), 
			transitions={'done':'MOVE_TORPEDO_DEPTH', 'notdone':'FACE_TORPEDO_TASK', 'reset':'RESET'})

		smach.StateMachine.add('MOVE_TORPEDO_DEPTH', sd.ChangeDepthToTarget(TORPEDO_BOARD_DEPTH), 
			transitions={'done':'COMPLETED', 'notdone':'MOVE_TORPEDO_DEPTH', 'reset':'RESET'})

		smach.StateMachine.add('COMPLETED', sd.PublishTopic('/buoy_task_complete', True),
			transitions={'done':'IDLE'})

		smach.StateMachine.add('RESET', sd.Reset(), 
			transitions={'done':'IDLE'})

	outcome = sm.execute()
	rospy.spin()
	sis.stop()

if __name__ == '__main__':
	main()
