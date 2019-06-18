#!/usr/bin/env python

import numpy as np
import rospy
import smach
import smach_ros
from std_msgs.msg import Bool, Float64, Int16

global ABC
ABC = 20

class StartState(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['ready', 'notready'])

		self.torpedo_task_subscriber = rospy.Subscriber('/torpedo_enable', Bool, self.torpedo_task_callback)
		self.torpedoEnabled = False

		self.XXXX_subscriber = rospy.Subscriber('/yaw_control/state', Float64, self.XXXX_callback) # get current orientation
		self.XXXX_publisher = rospy.Publisher('/yaw_control/setpoint', Float64, queue_size=10)
		self.XXXX = Float64()
		self.XXXX.data = 0.0

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

		self.object_x_subscriber = rospy.Subscriber('torpedo_x', Float64, self.object_x_callback)
		self.object_y_subscriber = rospy.Subscriber('torpedo_y', Float64, self.object_x_callback)
		self.object_area_subscriber = rospy.Subscriber('torpedo_area', Float64, self.object_area_callback)
		self.object_x = 0
		self.object_y = 0
		self.object_area = 0

		self.forward_thrust_publisher	= rospy.Subscriber('/yaw_pwm', Int16, self.forward_thrust_callback)
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
		
		new_yaw = Float64()
		new_depth = Float64()
		new_forward_thrust = Int16()

		# rotate yaw until x is within center
		if self.object_x > camera_width/2 + padding_x:
			new_yaw.data = self.yaw_current - yaw_change
			self.yaw_setpoint_publisher.publish(new_yaw)

		elif self.object_x < camera_width/2 - padding_x:
			new_yaw.data = self.yaw_current + yaw_change
			self.yaw_setpoint_publisher.publish(new_yaw)

		# change depth until y is within center
		if self.object_y > camera_height/2 + padding_y:
			new_depth.data = self.depth_current + depth_change
			self.depth_publisher.publish(new_depth)

		elif self.object_y < camera_height/2 - padding_y:
			new_depth.data = self.depth_current - depth_change
			self.depth_publisher.publish(new_depth)

		# move forward/backward until object area is 90%
		if self.object_area < area_threshold_low:
			new_forward_thrust.data = self.forward_thrust + forward_thrust_change
			self.forward_thrust_publisher.publish(new_forward_thrust)

		elif self.object_area > area_threshold_low:
			new_forward_thrust.data = self.forward_thrust - forward_thrust_change
			self.forward_thrust_publisher.publish(new_forward_thrust)

class TrackHeartState(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['completed', 'notcompleted'])

	def execute(self, userdata):
		pass


class ShootTorpedoState(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['completed', 'notcompleted'])

	def execute(self, userdata):
		pass


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