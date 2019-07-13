#!/usr/bin/env python

import numpy as np
import rospy
import smach
import smach_ros
from std_msgs.msg import Bool, Float64, Int16

import seadragon_states as state



WAIT_TIME = 10000
TORPEDO_Y_OFFSET = 10


def main():
	rospy.init_node('torpedo_task_state_machine')
	sm = smach.StateMachine(outcomes=['torpedo_task_complete'])
	sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_TORPEDO_TASK')
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
		smach.StateMachine.add('IDLE', state.WaitForTopic('/torpedo_enable'), 
			transitions={'done':'TRACK_BOARD', 'notdone':'IDLE'})
		smach.StateMachine.add('TRACK_BOARD', state.TrackObject(board_topic, 0,0), 
			transitions={'done':'TRACK_HEART', 'notdone':'TRACK_BOARD', 'reset':'RESET'})
		smach.StateMachine.add('TRACK_HEART', state.TrackObject(heart_topic, 0, TORPEDO_Y_OFFSET), 
			transitions={'done':'SHOOT', 'notdone':'TRACK_HEART', 'reset':'RESET'})
		smach.StateMachine.add('SHOOT', state.PublishTopic('/torpedo_shoot', True), 
			transitions={'done':'SHOOT2'})
		smach.StateMachine.add('SHOOT2', state.PublishTopic('/torpedo_shoot', False), 
			transitions={'done':'COMPLETE'})
		smach.StateMachine.add('COMPLETE', state.PublishTopic('/task_complete', True), 
			transitions={'done':'IDLE'})
		smach.StateMachine.add('RESET', state.Reset(), 
			transitions={'done':'IDLE'})

	outcome = sm.execute()
	rospy.spin()
	sis.stop()

if __name__ == '__main__':
	main()
