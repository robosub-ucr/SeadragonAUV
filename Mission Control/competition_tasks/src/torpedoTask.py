#!/usr/bin/env python

import numpy as np
import rospy
import smach
import smach_ros
from std_msgs.msg import Bool, Float64, Int16

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

class TrackBoardState(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['completed', 'notcompleted'])

		self.object_x_subscriber = rospy.Subscriber('/torpedo_board_x', Float64, self.object_x_callback)
		self.object_y_subscriber = rospy.Subscriber('/torpedo_board_y', Float64, self.object_y_callback)
		self.object_area_subscriber = rospy.Subscriber('/torpedo_board_area', Float64, self.object_area_callback)
		self.object_x = 0
		self.object_y = 0
		self.object_area = 0

		#self.forward_thrust_subscriber	= rospy.Subscriber('/yaw_pwm', Int16, self.forward_thrust_callback)
		self.forward_thrust_publisher	= rospy.Publisher('/yaw_pwm', Int16, queue_size=10)
		self.forward_thrust = 0

		self.yaw_current_subscriber = rospy.Subscriber('/yaw_control/state', Float64, self.yaw_callback) # current orientation
		self.yaw_setpoint_publisher = rospy.Publisher('yaw_control/setpoint', Float64, queue_size=10) # desired orientation
		self.yaw_current = 0 # in degrees

		self.depth_subscriber = rospy.Subscriber('/depth_control/state', Float64, self.depth_callback)
		self.depth_publisher = rospy.Publisher('/depth_control/setpoint', Float64, queue_size=10)
		self.depth_current = 0 # in inches

	def object_x_callback(self, msg):
		self.object_x = msg.data

	def object_y_callback(self, msg):
		self.object_y = msg.data

	def object_area_callback(self, msg):
		self.object_area = msg.data

	def forward_thrust_callback(self, msg):
		self.forward_thrust = msg.data

	def yaw_callback(self, msg):
		self.yaw_current = msg.data

	def depth_callback(self, msg):
		self.depth_current = msg.data

	def execute(self, userdata):
		
		camera_width = 400
		camera_height = 300
		padding_x = 5
		padding_y = 5
		yaw_change = 2
		area_threshold_low = 0.85
		area_threshold_high = 0.90
		depth_change = 1
		foward_thrust_change = 1

		is_object_x_centered = False
		is_object_y_centered = False
		is_object_area_in_threshold = False
		
		new_yaw = Float64()
		new_depth = Float64()
		new_forward_thrust = Int16()

		# rotate yaw until x is within center
		if self.object_x > camera_width/2 + padding_x:
			is_object_x_centered = False
			new_yaw.data = self.yaw_current - yaw_change
			self.yaw_setpoint_publisher.publish(new_yaw)
		elif self.object_x < camera_width/2 - padding_x:
			is_object_x_centered = False
			new_yaw.data = self.yaw_current + yaw_change
			self.yaw_setpoint_publisher.publish(new_yaw)
		else:
			is_object_x_centered = True

		# change depth until y is within center
		if self.object_y > camera_height/2 + padding_y:
			is_object_y_centered = False
			new_depth.data = self.depth_current + depth_change
			self.depth_publisher.publish(new_depth)
		elif self.object_y < camera_height/2 - padding_y:
			is_object_y_centered = False
			new_depth.data = self.depth_current - depth_change
			self.depth_publisher.publish(new_depth)
		else:
			is_object_y_centered = True

		# move forward/backward until object area is 90%
		if self.object_area < area_threshold_low:
			is_object_area_in_threshold = False
			new_forward_thrust.data = self.forward_thrust + forward_thrust_change
			self.forward_thrust_publisher.publish(new_forward_thrust)
		elif self.object_area > area_threshold_low:
			is_object_area_in_threshold = False
			new_forward_thrust.data = self.forward_thrust - forward_thrust_change
			self.forward_thrust_publisher.publish(new_forward_thrust)
		else:
			is_object_area_in_threshold = True

		# go to next state if all object is at center of camera and within certain distace
		if is_object_x_centered and is_object_y_centered and is_object_area_in_threshold:
			return 'completed'
		else:
			return 'notcompleted'

class TrackHeartState(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['completed', 'notcompleted'])

		self.object_x_subscriber = rospy.Subscriber('/torpedo_heart_x', Float64, self.object_x_callback)
		self.object_y_subscriber = rospy.Subscriber('/torpedo_heart_y', Float64, self.object_y_callback)
		self.object_area_subscriber = rospy.Subscriber('/torpedo_heart_area', Float64, self.object_area_callback)
		self.object_x = 0
		self.object_y = 0
		self.object_area = 0

		#self.forward_thrust_subscriber	= rospy.Subscriber('/yaw_pwm', Int16, self.forward_thrust_callback)
		self.forward_thrust_publisher	= rospy.Publisher('/yaw_pwm', Int16, queue_size=10)
		self.forward_thrust = 0

		self.yaw_current_subscriber = rospy.Subscriber('/yaw_control/state', Float64, self.yaw_callback) # current orientation
		self.yaw_setpoint_publisher = rospy.Publisher('yaw_control/setpoint', Float64, queue_size=10) # desired orientation
		self.yaw_current = 0 # in degrees

		self.depth_subscriber = rospy.Subscriber('/depth_control/state', Float64, self.depth_callback)
		self.depth_publisher = rospy.Publisher('/depth_control/setpoint', Float64, queue_size=10)
		self.depth_current = 0 # in inches

	def execute(self, userdata):
		
		camera_width = 400
		camera_height = 300
		padding_x = 3
		padding_y = 3
		yaw_change = 2
		area_threshold_low = 0.85
		area_threshold_high = 0.90
		depth_change = 1
		foward_thrust_change = 1

		is_object_x_centered = False
		is_object_y_centered = False
		is_object_area_in_threshold = False
		
		new_yaw = Float64()
		new_depth = Float64()
		new_forward_thrust = Int16()

		# rotate yaw until x is within center
		if self.object_x > camera_width/2 + padding_x:
			is_object_x_centered = False
			new_yaw.data = self.yaw_current - yaw_change
			self.yaw_setpoint_publisher.publish(new_yaw)
		elif self.object_x < camera_width/2 - padding_x:
			is_object_x_centered = False
			new_yaw.data = self.yaw_current + yaw_change
			self.yaw_setpoint_publisher.publish(new_yaw)
		else:
			is_object_x_centered = True

		# change depth until y is within center
		if self.object_y > camera_height/2 + padding_y:
			is_object_y_centered = False
			new_depth.data = self.depth_current + depth_change
			self.depth_publisher.publish(new_depth)
		elif self.object_y < camera_height/2 - padding_y:
			is_object_y_centered = False
			new_depth.data = self.depth_current - depth_change
			self.depth_publisher.publish(new_depth)
		else:
			is_object_y_centered = True

		# move forward/backward until object area is 90%
		if self.object_area < area_threshold_low:
			is_object_area_in_threshold = False
			new_forward_thrust.data = self.forward_thrust + forward_thrust_change
			self.forward_thrust_publisher.publish(new_forward_thrust)
		elif self.object_area > area_threshold_low:
			is_object_area_in_threshold = False
			new_forward_thrust.data = self.forward_thrust - forward_thrust_change
			self.forward_thrust_publisher.publish(new_forward_thrust)
		else:
			is_object_area_in_threshold = True

		# go to next state if all object is at center of camera and within certain distace
		if is_object_x_centered and is_object_y_centered and is_object_area_in_threshold:
			self.resetValues()
			return 'completed'
		else:
			return 'notcompleted'

	def resetValues(self):
		self.object_x = 0
		self.object_y = 0
		self.object_area = 0
		self.forward_thrust = 0
		self.yaw_current = 0
		self.depth_current = 0
		

class ShootTorpedoState(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['completed', 'notcompleted'])

		self.shoot_publisher = rospy.Publisher('/torpedo_shoot', Bool, queue_size=10)

	def execute(self, userdata):
		shoot = Bool()
		shoot.data = True
		self.shoot_publisher.publish(shoot)
		return 'notcompleted'


class DepthChangeState(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['completed', 'notcompleted'])

	def execute(self, userdata):
		pass


class EndState(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['completed', 'notcompleted'])

	def execute(self, userdata):
		pass

class ResetState(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['completed', 'notcompleted'])

	def execute(self, userdata):
		pass


def main:
	rospy.init_node('torpedo_task_state_machine')
	sm = smach.StateMachine(outcomes=['torpedo_task_complete'])
	sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
	sis.start()

	with sm:
		smach.StateMachine.add('StartState', StartState(), transitions={'ready':'TrackBoardState', 'notready':'StartState'})
		smach.StateMachine.add('TrackBoardState', TrackBoardState(), transitions={'completed':'TrackHeartState', 'notcompleted':'TrackBoardState', 'reset':'ResetState'})
		smach.StateMachine.add('TrackHeartState', TrackHeartState(), transitions={'completed':'ShootTorpedoState', 'notcompleted':'TrackHeartState', 'reset':'ResetState'})
		smach.StateMachine.add('ShootTorpedoState', ShootTorpedoState(), transitions={'completed':'DepthChangeState', 'notcompleted':'ShootTorpedoState', 'reset':'ResetState'})
		smach.StateMachine.add('DepthChangeState', DepthChangeState(), transitions={'completed':'EndState', 'notcompleted':'DepthChangeState', 'reset':'ResetState'})
		smach.StateMachine.add('EndState', EndState(), transitions={'completed':'torpedo_task_complete', 'notcompleted':'EndState'})
		smach.StateMachine.add('ResetState', EndState(), transitions={'reset':'StartState'})

	outcome = sm.execute()
	rospy.spin()
	sis.stop()

if __name__ == '__main__':
	main()