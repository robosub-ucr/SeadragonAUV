#!/usr/bin/env python
# ^ Execute with a Python interpreter, using the program search path to find it

#import numpy as np
import rospy
import smach
import smach_ros

# define state State1
class State1(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['OUT_State1_complete'])

	def execute(self, userdata):
		rospy.loginfo('Executing state State1')
		print('State1 executing')
		return 'OUT_State1_complete'

# define state 
class State2(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['OUT_State2_complete'])

	def execute(self, userdata):
		rospy.loginfo('Executing state State2')
		print('State2 executing')
		return 'OUT_State2_complete'

def main():
	# Initialize node with desired node name - ideally task name
	rospy.init_node('SM_poolTest')

	# Create a SMACH state machine
	sm = smach.StateMachine(outcomes=['OUT_poolTest_complete'])
	
	# Create and start introspection server - fancy way of saying view gui feature
	sis = smach_ros.IntrospectionServer('introspection_server', sm, '/SM_POOL_TEST')
	sis.start()

	# Open the container
	with sm:
		# Add states to the container
		smach.StateMachine.add('State1', State1(),
			transitions={'OUT_State1_complete':'State2'})
		smach.StateMachine.add('State2', State2(),
			transitions={'OUT_State2_complete':'OUT_poolTest_complete'})

	# Execute SMACH plan
	outcome = sm.execute()

	# spin?
	rospy.spin()
	sis.stop()

# main guard
if __name__ == '__main__':
	main()
