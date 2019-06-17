#!/usr/bin/env python

import numpy as np
import rospy
import smach
import smach_ros
from std_msgs.msg import Bool, Float64, Int16

class StartState(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['ready', 'notready'])

		self.XXXX_subscriber = rospy.Subscriber('/yaw_control/state', Float64, self.XXXX_callback) # get current orientation
		self.XXXX_publisher = rospy.Publisher('/yaw_control/setpoint', Float64, queue_size=10)
		self.XXXX = Float64()
		self.XXXX.data = 0.0

	def XXXX_callback(self, msg):
		self.XXXX.data = msg.data

	def execute(self, userdata):
		if True:
			return 'ready'
		else:
			return 'notready'

class TrackBoardState(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['completed', 'notcompleted'])

	def execute(self, userdata):
		pass


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
		smach.StateMachine.add('TrackBoardState', TrackBoardState(), transitions={'completed':'TrackHeartState', 'notcompleted':'TrackBoardState'})
		smach.StateMachine.add('TrackHeartState', TrackHeartState(), transitions={'completed':'ShootTorpedoState', 'notcompleted':'TrackHeartState'})
		smach.StateMachine.add('ShootTorpedoState', ShootTorpedoState(), transitions={'completed':'DepthChangeState', 'notcompleted':'ShootTorpedoState'})
		smach.StateMachine.add('DepthChangeState', DepthChangeState(), transitions={'completed':'EndState', 'notcompleted':'DepthChangeState'})
		smach.StateMachine.add('EndState', EndState(), transitions={'completed':'????', 'notcompleted':'EndState'})

	outcome = sm.execute()
	rospy.spin()
	sis.stop()

if __name__ == '__main__':
	main()