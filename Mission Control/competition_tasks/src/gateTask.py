'''

		To Run this code: 

		0. Save this file to any directory/Folder on your desktop

		1. Go to directory where you have this file in terminal

		2. From terminal run $ sudo chmod u+x [yourfilename.py] 

		3. Open new terminal and run $ roscore

		4. From terminal where you have your sm file run $ python filename.py

		5. In a new terminal, run rosrun smach_viewer smach_viewer.py 

		6. A gui should pop up showing you your state machine



		7. At any time during state A or state B type the following command

		8. In a new terminal run $ rostopic pub /reset std_msgs/Bool "data: True"

					    command ... topic ... var type ... data ...

		

		Be careful with your indentations! Indentations indicate a new

		block of code in python.  You've got no semicolon but you've 

		got indentations. 

'''

#!/usr/bin/env python

import rospy
import smach
import smach_ros
from std_msgs.msg import Bool, Float64, Int16

import seadragon_states as state

DEPTH_START = 18 #depth is in inches
YAW_GATE = 1.57 # will change acording to direction of gate
turn = 0.26 # degrees to turn after track state
buoy_depth = 36 # depth (in inches) for the buoys
gate_timer = 10000 #time to pass gate
BOUY_TASK_DEPTH = 5*12 # in inches

def main():
	# Initialize node with desired node name - ideally task name
	rospy.init_node('gateTask')

	# Create a SMACH state machine
	sm = smach.StateMachine(outcomes=['task_complete'])

	# Create and start introspection server - fancy way of saying view gui feature
	sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_GATE_TASK')
	sis.start()

	gate_topic = {
		'x':'/gate_x',
		'y':'/gate_y',
		'area':'/gate_area'
	}

	# Open SMACH container
	with sm:
		smach.StateMachine.add('IDLE', state.WaitForTopic('/gate_enable'), 
			transitions={'done':'DEPTH_PID_ENABLE', 'notdone':'IDLE'})
		smach.StateMachine.add('DEPTH_PID_ENABLE', state.PublishTopic('/depth_control/pid_enable', True), 
			transitions={'done':'DIVE_GATE_DEPTH'})
		smach.StateMachine.add('DIVE_GATE_DEPTH', state.ChangeDepthToTarget(18), 
			transitions={'done':'YAW_PID_ENABLE', 'notdone':'DIVE_GATE_DEPTH', 'reset':'RESET'})
		smach.StateMachine.add('YAW_PID_ENABLE', state.PublishTopic('/yaw_control/pid_enable', True), 
			transitions={'done':'ROTATE_TO_GATE'})
		smach.StateMachine.add('ROTATE_TO_GATE', state.RotateYawToAbsoluteTarget(1.57), 
			transitions={'done':'TRACK_GATE', 'notdone':'ROTATE_TO_GATE', 'reset':'RESET'})	
		smach.StateMachine.add('TRACK_GATE', state.TrackObject(gate_topic), 
			transitions ={'done':'SET_YAW_GATE_OFFSET', 'notdone':'TRACK_GATE', 'reset':'RESET'})


		smach.StateMachine.add('SET_YAW_GATE_OFFSET', state.PublishTopicRelative('/yaw_control/state', '/yaw_control/setpoint', Float64, -0.017*5),
			transitions = {'done':'ROTATE_GATE_LEFT', 'notdone':'SET_YAW_GATE_OFFSET', 'reset':'RESET'})

		smach.StateMachine.add('ROTATE_GATE_LEFT', state.WaitForConvergence('/yaw_control/state', Float64, -0.017*5, 0.017, 5000), 
			transitions ={'done':'MOVE_FORWARD_GATE', 'notdone':'ROTATE_GATE_LEFT', 'reset':'RESET'})
		

		smach.StateMachine.add('MOVE_FORWARD_GATE', state.MoveForwardTimed(10000, True), 
			transitions ={'done':'BUOY_DEPTH','notdone':'MOVE_FORWARD_GATE', 'reset':'RESET'})
		smach.StateMachine.add('BUOY_DEPTH', state.ChangeDepthToTarget(5*12), 
			transitions ={'done':'ROTATE_TO_BUOY','notdone':'BUOY_DEPTH', 'reset':'RESET'})
		smach.StateMachine.add('ROTATE_TO_BUOY', state.RotateYawToAbsoluteTarget(0.017 * 45), 
			transitions ={'done':'COMPLETED', 'notdone':'ROTATE_TO_BUOY', 'reset':'RESET'})
		smach.StateMachine.add('COMPLETED', state.PublishTopic('/task_complete', True), 
			transitions ={'done':'IDLE'})
		smach.StateMachine.add('RESET', state.Reset(), 
			transitions ={'done':'IDLE'})

	# Execute State Machine
	outcome = sm.execute()

	# Spin node - fancy way of saying run code in a loop
	rospy.spin()
	sis.stop()

if __name__ == '__main__':
	main()