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



##------------------------------- IMPORTS ---------------------------------------##

import rospy
import smach
import smach_ros
from std_msgs.msg import Bool, Float64, Int16

import seadragon_states as state

##----------------------------- END IMPORTS -------------------------------------##

DEPTH_START = 18 #depth is in inches
YAW_GATE = 1.57 # will change acording to direction of gate
turn = 0.26 # degrees to turn after track state
buoy_depth = 36 # depth (in inches) for the buoys
gate_timer = 10000 #time to pass gate
BOUY_TASK_DEPTH = 5*12 # in inches

##------------------------- STATE DEFINITIONS -----------------------------------##

class IDLE(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['ready','notready'])

		self.gateEnable_subscriber = rospy.Subscriber('/gate_enable', Bool, self.task_callback)	
		self.gateEnabled = False		
		self.depthPoint_publisher = rospy.Publisher('/depth_control/setpoint', Int16, queue_size=10)
		self.depthPoint = Int16()
		self.depthPidEnable_publisher = rospy.Publisher('/depth_control/pid_enable', Bool, queue_size=10)
		self.depthEnable = Bool()	

	def task_callback(self, msg):
		self.gateEnabled = msg.data	

	def execute(self, userdata):
		# Push down about two inches to start state machine
		if self.gateEnabled:
			# Set depth setpoint FIXME 48 - 4ft, 120 - 10ft
			self.depthPoint.data = DEPTH_START
			self.depthPoint_publisher.publish(self.depthPoint)
			# Enable depth pid
			self.depthEnable.data = True
			self.depthPidEnable_publisher.publish(self.depthEnable)
			self.gateEnabled = False
			return 'ready'
		else:
			return 'notready'


class DIVE(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['ready','notready', 'reset'])
	
		self.currDepth_subscriber	= rospy.Subscriber('/depth_control/state', Int16, self.depth_callback)
		self.currDepth = 0
		self.depthPoint = DEPTH_START

		self.yawPoint_publisher = rospy.Publisher('/yaw_control/setpoint', Float64, queue_size=10)
		self.yawPoint = Float64()
	
		self.yawPidEnable_publisher = rospy.Publisher('/yaw_control/pid_enable', Bool, queue_size=10)
		self.yawEnable = Bool()

		self.reset_subscriber = rospy.Subscriber('/reset', Bool, self.reset_callback)
		self.reset = False 
	
	def depth_callback(self, msg):
		self.currDepth = msg.data
	def reset_callback(self, msg):
		self.reset = msg.data

	def execute(self, userdata):
		#rospy.loginfo('Executing state dive')
		print("current depth: ", self.currDepth, "depth setpoint:", self.depthPoint)

		if self.reset == True:
			self.reset = False
			return 'reset'

		if abs(self.depthPoint - self.currDepth) > 2:
			return 'notready'
		else:
			self.yawPoint.data = YAW_GATE
			self.yawPoint_publisher.publish(self.yawPoint)
			# Enable depth pid
			self.yawEnable.data = True
			self.yawPidEnable_publisher.publish(self.yawEnable)
			self.reset = False
			return 'ready'			


class ORIENTATION(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['continue','wait', 'reset'])
		# Publishers, Subscribers

		self.reset_subscriber 	= rospy.Subscriber('/reset', Bool, self.reset_callback)
		self.curryaw_subscriber	= rospy.Subscriber('/yaw_control/state', Float64, self.yaw_callback)
		
		# Local Variables
		self.yawPoint = YAW_GATE
		self.orientation = 0
		self.reset = False

	def reset_callback(self,msg):
		self.reset = msg.data
	def yaw_callback(self,msg):
		self.orientation = msg.data
	
	def execute(self, userdata):
		if self.reset:
			self.reset = False
			return 'reset'

		if abs(self.yawPoint - self.orientation) <= 0.017 :
			self.reset = False
			return 'continue'
		else:
			return 'wait'


class TRACK(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['area>90','track', 'reset'])

		self.timer = 0
		#Subscribers and publishers
		self.reset_subscriber = rospy.Subscriber('/reset', Bool, self.reset_callback)
		self.curryaw_subscriber = rospy.Subscriber('/yaw_control/state', Float64, self.yaw_callback) 
		self.yawPoint_publisher = rospy.Publisher('/yaw_control/setpoint', Float64, queue_size=10) 

		self.gate_x_subscriber = rospy.Subscriber('/gate_x', Float64, self.gate_x_callback)
		self.gate_area_subscriber = rospy.Subscriber('/gate_area', Float64, self.gate_area_callback)
		self.gate_x = 0
		self.gate_area = 0.0

		self.forward_thrust_publisher	= rospy.Publisher('/yaw_pwm', Int16, queue_size=10)
		self.forward_thrust = 0
		
		# Local Variables																
		self.reset = False
		self.curryaw = 0 
		self.yawPoint = Float64()

	def reset_callback(self, msg):
		self.reset = msg.data
	def gate_x_callback(self, msg):
		self.gate_x = msg.data
	def gate_area_callback(self, msg):
		self.gate_area = msg.data
	def yaw_callback(self, msg):
		self.currYaw = msg.data

	def execute(self, userdata):
		if self.reset:
			self.timer = 0
			self.reset = False
			return 'reset'

		camera_width = 400
		camera_height = 300
		padding_x = 5
		yaw_change = .1
		area_threshold_low = 0.65
		area_threshold_high = 0.70
		new_yaw = Float64()
		
		#center the object for x-axis
		if self.gate_x > camera_width/2 + padding_x:
			new_yaw.data = self.currYaw - yaw_change
			self.yawPoint_publisher.publish(new_yaw)
			self.forward_thrust = 0
			self.forward_thrust_publisher.publish(self.forward_thrust)
		elif self.gate_x < camera_width/2 - padding_x:
			new_yaw.data = self.currYaw + yaw_change
			self.yawPoint_publisher.publish(new_yaw)
			self.forward_thrust = 0
			self.forward_thrust_publisher.publish(self.forward_thrust)
		else:
			self.timer +=1
			if self.timer % 200 == 0:
				if self.forward_thrust < 280:
					self.forward_thrust += 1
					self.forward_thrust_publisher.publish(self.forward_thrust)

		if self.gate_area/(camera_width*camera_height) > area_threshold_low and self.gate_area/(camera_width*camera_height) < area_threshold_high:
			self.reset = False
			self.forward_thrust = 0
			self.forward_thrust_publisher.publish(self.forward_thrust)
			self.yawPoint.data = self.currYaw - turn
			self.yawPoint_publisher.publish(self.yawPoint)
			self.timer = 0
			return 'area>90'
		else: 
			return 'track'


class TURN(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['pass','wait', 'reset'])

		self.reset = False
		self.yaw_current = 0
		self.yaw_setpoint = -1
		self.yaw_assigned = False
		rospy.Subscriber('/reset', Bool, self.reset_callback)
		rospy.Subscriber('/yaw_control/state', Float64, self.yaw_callback)

		self.yawPoint_publisher = rospy.Publisher('/yaw_control/setpoint', Float64, queue_size=10)

	def reset_callback(self,msg):
		self.reset = msg.data
	def yaw_callback(self,msg):
		self.yaw_current = msg.data
		if not self.yaw_assigned:
			self.yaw_setpoint = self.yaw_current - 0.017*5 # 5 radians
			self.yaw_assigned = True

	def execute(self, userdata):
		if self.reset:
			self.resetValues()
			return 'reset'
		print("yaw_current", self.yaw_current, "yaw_setpoint", self.yaw_setpoint)

		if self.yaw_assigned and abs(self.yaw_setpoint - self.yaw_current) < 0.017:
			self.resetValues()
			return 'pass'
		else: 
			return 'wait'

	def resetValues(self):
		self.yaw_current = 0
		self.yaw_setpoint = -1
		self.yaw_assigned = False
		self.reset = False


class PASS(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['timer','wait', 'reset'])

		self.reset_subscriber = rospy.Subscriber('/reset', Bool, self.reset_callback)
		self.fwdThrust_publisher = rospy.Publisher('/yaw_pwm', Int16, queue_size=10)
		self.depthPoint_publisher= rospy.Publisher('/depth_control/setpoint', Int16, queue_size=10)

		# Local Variables
		self.fwdThrust = Int16()
		self.fwdThrust.data = 0
		self.timer = 0
		self.reset = False
		self.depthPoint = Int16()

	def reset_callback(self, msg):
		self.reset = msg.data

	def execute(self, userdata):
		self.timer +=1
		if self.timer % 200 == 0:
			if self.fwdThrust.data < 280:
				self.fwdThrust.data += 1
				self.fwdThrust_publisher.publish(self.fwdThrust)

		if self.reset:
			self.timer = 0
			self.reset = False
			return 'reset'
		
		if self.timer > gate_timer:
			self.fwdThrust.data = 0
			self.fwdThrust_publisher.publish(self.fwdThrust)
			self.depthPoint.data = buoy_depth
			self.depthPoint_publisher.publish(self.depthPoint)
			self.reset = False
			self.timer = 0
			return 'timer'
		else: 
			return 'wait'


class SET_DEPTH(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['depth','wait', 'reset'])

		rospy.Subscriber('/depth_control/state', Int16, self.depth_callback) 
		rospy.Subscriber('/reset', Bool, self.reset_callback)

		#self.curryaw_subscriber = rospy.Subscriber('/yaw_control/state', Float64, self.yaw_callback) 
		#self.yawPoint_publisher = rospy.Publisher('/yaw_control/setpoint', Float64, queue_size=10) 
		
		self.depth = 0
		#self.curryaw = 0
		#self.yawPoint = Float64()
		self.reset = False

	def reset_callback(self,msg):
		self.reset = msg.data
	def depth_callback(self,msg):
		self.depth = msg.data

	def execute(self, userdata):
		if self.reset:
			self.reset = False
			return 'reset'

		if abs(self.depth - BOUY_TASK_DEPTH) <= 1:
			self.yawPoint.data = self.curryaw + turn
			self.yawPoint_publisher.publish(self.yawPoint)
			self.reset = False
			return 'depth'
		else:
			return 'wait'


class REORIENT(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['complete','wait', 'reset'])

		#Subscribers and Publishers
		self.reset_subscriber = rospy.Subscriber('/reset', Bool, self.reset_callback)
		self.curryaw_subscriber = rospy.Subscriber('/yaw_control/state', Float64, self.yaw_callback)
		self.yawPoint_subscriber= rospy.Subscriber('/yaw_control/setpoint', Float64, self.yawPoint_callback)
		
		# Local Variables
		self.currYaw = 0
		self.yawPoint = 0
		self.reset = False

	def reset_callback(self,msg):
		self.reset = msg.data
	def yaw_callback(self,msg):
		self.currYaw = msg.data
	def yawPoint_callback(self,msg):
		self.yawPoint = msg.data

	def execute(self, userdata):
		print(self.currYaw)
		
		if self.reset:
			self.reset = False
			return 'reset'

		if abs(self.yawPoint - self.currYaw) < 0.017:
			return 'complete'
		else: 
# ############################### 
# how does this state rotate yaw?
# ###############################
			return 'wait'	


class COMPLETED(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['done'])

		self.taskComplete_publisher = rospy.Publisher('/task_complete', Bool, queue_size=10)
	
	def execute(self, userdata):
		taskComplete = True
		self.taskComplete_publisher.publish(taskComplete)
		return 'done'


class RESET(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['restart'])

	def execute(self, userdata):
		return 'restart'


##-------------------------- END STATE DEFINITIONS ------------------------------------##

def main():
	# Initialize node with desired node name - ideally task name
	rospy.init_node('gateTask')

	# Create a SMACH state machine
	sm = smach.StateMachine(outcomes=['task_complete'])

	# Create and start introspection server - fancy way of saying view gui feature
	sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_GATE_TASK')
	sis.start()

	gate_topic = {
		'x':'gate_x',
		'y':'gate_y',
		'area':'gate_area'
	}

	# Open SMACH container
	with sm:
		smach.StateMachine.add('IDLE', state.WaitForTopic('/gate_enable'), 
			transitions={'done':'START', 'notdone':'IDLE'})
		smach.StateMachine.add('START', state.PublishTopic('/depth_control/pid_enable', True), 
			transitions={'done':'DIVE'})
		smach.StateMachine.add('DIVE', state.ChangeDepthToTarget(18), 
			transitions={'done':'ORIENTATION', 'notdone':'DIVE', 'reset':'RESET'})
		smach.StateMachine.add('ORIENTATION', state.RotateYawToAbsoluteTarget(1.57), 
			transitions={'done':'TRACK', 'notdone':'ORIENTATION', 'reset':'RESET'})	
		smach.StateMachine.add('TRACK', state.TrackObject(gate_topic, 0, 0), 
			transitions ={'done':'TURN', 'notdone':'TRACK', 'reset':'RESET'})
		smach.StateMachine.add('TURN', state.RotateYawToRelativeTarget(-0.017*5), 
			transitions ={'done':'PASS', 'notdone':'TURN', 'reset':'RESET'})
		smach.StateMachine.add('PASS', state.MoveForwardTimed(10000, True), 
			transitions ={'done':'SET_DEPTH','notdone':'PASS', 'reset':'RESET'})
		smach.StateMachine.add('SET_DEPTH', state.ChangeDepthToTarget(5*12), 
			transitions ={'done':'REORIENT','notdone':'SET_DEPTH', 'reset':'RESET'})
		smach.StateMachine.add('REORIENT', state.RotateYawToAbsoluteTarget(0.017 * 45), 
			transitions ={'done':'COMPLETED', 'notdone':'REORIENT', 'reset':'RESET'})
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