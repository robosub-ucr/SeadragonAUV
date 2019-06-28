

#!/usr/bin/env python

##------------------------------- IMPORTS ---------------------------------------##

from decimal import *

import matplotlib.pyplot as plt

import math
import numpy as np
import rospy
import smach
import smach_ros
from std_msgs.msg import Bool, Float64, Int16

##----------------------------- END IMPORTS -------------------------------------##


##------------------------- STATE DEFINITIONS -----------------------------------##

GATE_TASK = 0
BUOY_TASK = 1
TORPEDO_TASK =2
DUMMY_TASK = 3


class idle(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['go','!go'])
		
		# Subscribers
		self.depth_subscriber = rospy.Subscriber('/depth', Int16, self.depth_callback)		
		
		# Publishers
		self.yawPidEnable_publisher	= rospy.Publisher('/yaw_control/pid_enable', Bool, queue_size=10)
		self.depthPidEnable_publisher	= rospy.Publisher('/depth_control/pid_enable', Bool, queue_size=10)
		self.fwdThrust_publisher	= rospy.Publisher('/yaw_pwm', Int16, queue_size=10) 
		self.taskReset_publisher	= rospy.Publisher('/reset', Bool, queue_size=10)
		self.light_publisher		= rospy.Publisher('light_state', Int16, queue_size=10)
			
		# Publisher Data Containers
		self.taskReset			= Bool()
		self.taskReset.data		= True
		self.Disable			= Bool()
		self.Disable.data 		= False
		self.fwdThrust			= Int16()
		self.fwdThrust.data 		= 0
		self.light			= Int16()
		self.light.data			= 0

		# Local variables
		self.depthStart     = 0
		self.systemDisabled = False

	def depth_callback(self,msg):
		self.depthStart = msg.data

	def execute(self, userdata):
		
		self.light_publisher.publish(self.light)
		# Disable Motor control System & Task statemachines
		if self.systemDisabled == False:
			self.fwdThrust_publisher.publish(self.fwdThrust)
			self.taskReset.data = True
			self.taskReset_publisher.publish(self.taskReset)
			self.yawPidEnable_publisher.publish(self.Disable)
			self.depthPidEnable_publisher.publish(self.Disable)
			self.systemDisabled = True
		
		# Check if sub has been pushed down 'x' inches to begin run
		if self.depthStart > 12:
			self.systemDisabled = False
			self.taskReset.data = False
			self.taskReset_publisher.publish(self.taskReset)
			return 'go'

		else:
			return '!go'


class transition(smach.State):

	def __init__(self):
		smach.State.__init__(self, outcomes=['timeup','!timeup','reset'])

		# Subscribers
		self.depth_subscriber		= rospy.Subscriber('/depth', Int16, self.depth_callback) 
		
		# Publishers
		self.fwdThrust_publisher	= rospy.Publisher('/yaw_pwm', Int16, queue_size=10)
		self.light_publisher		= rospy.Publisher('light_state', Int16, queue_size=10)
		
		# Publisher Data Containers
		self.fwdThrust			= Int16()
		self.fwdThrust.data		= 0
		self.light			= Int16()
		self.light.data			= 1

		# Local Variables
		self.timer = 0
		self.resetDepth = 0
		
	def depth_callback(self,msg):
		self.resetDepth = msg.data

	def execute(self, userdata):
		self.light_publisher.publish(self.light)
		self.timer += 1

		# Begin Accelerating until cruising speed is reached
		if self.timer % 200 == 0:
			if self.fwdThrust.data < 280:
				self.fwdThrust.data += 1
				self.fwdThrust_publisher.publish(self.fwdThrust)

		# State Transitions
		if self.resetDepth < 6:
			# Reset Local Variables
			self.fwdThrust.data = 0			
			self.timer = 0
			return 'reset'

		elif self.timer > 10000:
			self.timer = 0
			# Stop Sub in preparation for search
			self.fwdThrust.data = 0
			self.fwdThrust_publisher.publish(self.fwdThrust)
			return 'timeup'

		else:
			return '!timeup'			


class search(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['taskfound','!taskfound','reset'])

		# Publishers, Subscribers
		self.depth_subscriber = rospy.Subscriber('/depth', Int16, self.depth_callback) 
		self.yaw_subscriber = rospy.Subscriber('/yaw_control/state', Float64, self.yaw_callback)
		self.task_subscriber = rospy.Subscriber('/task_detected', Int16, self.task_callback)
		
		self.task_publisher = rospy.Publisher('/task_detected', Int16, queue_size=10)
		self.yawOrientation_publisher = rospy.Publisher('/yaw_control/setpoint', Float64, queue_size=10)
		self.visionEnable_publisher	= rospy.Publisher('/vision_enable', Bool, queue_size=10)
		self.light_publisher = rospy.Publisher('light_state', Int16, queue_size=10)
		
		# Local Variables
		self.light = Int16()
		self.light.data = 2
		self.timer = 0
		self.resetDepth   = 0
		self.currYaw 	  = 0
		self.centerYaw    = 0
		self.taskDetected = 0
		self.visionEnable = Bool()
		self.visionEnable.data = False
		self.yawSetpoint  = Float64()
		self.turnRange	  = .79  # 45 degrees
		self.t 		  = 0
		
		self.rvs 	  = 1

	def depth_callback(self,msg):
		self.resetDepth = msg.data
	def yaw_callback(self,msg):
		self.currYaw = msg.data
	def task_callback(self,msg):
		self.taskDetected = msg.data	
	
	def map(self, x, in_min, in_max, out_min, out_max):
   		 return (x-in_min) * (out_max-out_min) / (in_max-in_min) + out_min

	def scan(self, T, thetaStart, thetaEnd):
		#T = T
		a0 = 0
		a1 = 0
		a2 = Decimal(3)  / Decimal(T**2)
		a3 = Decimal(-2) / Decimal(T**3)
	
		s      = a0 + (a1 * self.t) + (a2 * self.t**2) + (a3 * self.t**3) 	
		theta  = s * Decimal(abs(thetaEnd-thetaStart)) 
		self.yawSetpoint.data  = self.map(float(theta), 0, abs(thetaEnd-thetaStart), thetaStart, thetaEnd)
		self.yawOrientation_publisher.publish(self.yawSetpoint)
		
		self.t += 1

		if self.t > T:
			self.t = 0
			self.rvs = self.rvs * -1

	def execute(self, userdata):
		self.light_publisher.publish(self.light)
		# Check for reset condition
		if self.resetDepth < 6:
			# Reset Local Variables
			self.resetDepth = 0
			self.centerYaw  = 0
			self.currYaw    = 0
			self.taskDetected = 0
			self.visionEnable.data = False	
			self.t = 0
			self.timer = 0
			return 'reset'
		
		# Perform search procedure		
		self.timer += 1
		
		# Give some time to aquire center orientation
		if self.timer < 5:
			self.centerYaw = self.currYaw
		else: 
			if self.rvs == 1:
				self.scan(2000, self.centerYaw + self.turnRange, self.centerYaw - self.turnRange)
			else:
				self.scan(2000, self.centerYaw - self.turnRange, self.centerYaw + self.turnRange)
		
		# Turn Computer vision on -- Allow time to reach first setpoint 
		if self.timer >= 50 and self.timer <= 60:
			self.visionEnable.data = True
			self.visionEnable_publisher.publish(self.visionEnable)

		# Check for object detected to go to execute state
		if self.taskDetected > 0:
			# Reset Local Variables
			self.resetDepth = 0
			self.centerYaw  = 0
			self.currYaw    = 0
			self.taskDetected = 0
			self.visionEnable.data = False	
			self.t = 0
			self.timer = 0
			return 'taskfound'
		else:
			return'!taskfound'


class execute(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['taskcomplete','!taskcomplete','reset'])

		# Subscribers
		self.task_subscriber     = rospy.Subscriber('/task_detected', Int16, self.task_callback)
		self.complete_subscriber = rospy.Subscriber('/task_complete', Bool, self.complete_callback)
		self.depth_subscriber    = rospy.Subscriber('/depth', Int16, self.reset_callback)
	
		# Publishers
		self.enable 		 = Bool()
		self.enable.data 	 = False
		self.gate_publisher	 = rospy.Publisher('/gate_enable', Bool, queue_size=1)
		self.buoy_publisher	 = rospy.Publisher('/buoy_enable', Bool, queue_size=1)
		self.torpedo_publisher	 = rospy.Publisher('/torpedo_enable', Bool, queue_size=1)
		self.light_publisher	 = rospy.Publisher('light_state', Int16, queue_size=10)

		# Local variables	
		self.task	  	  = GATE_TASK
		self.taskEnabled  = False
		self.taskComplete = False	
		self.resetDepth   = 0
		self.light		  = Int16()
		self.light.data	  = 3

	def reset_callback(self,msg):
		self.resetDepth = msg.data
	def task_callback(self,msg):
		self.task = msg.data	
	def complete_callback(self,msg):
		self.taskComplete = msg.data	
			
	def execute(self, userdata):
		self.light_publisher.publish(self.light)
		print("execute :: task:", self.task, "taskEnabled:", self.taskEnabled)
		if self.task == GATE_TASK and not self.taskEnabled:
			self.enable.data = True
			self.gate_publisher.publish(self.enable)
			self.taskEnabled = True
		elif self.task == BUOY_TASK and not self.taskEnabled:
			self.enable.data = True
			self.buoy_publisher.publish(self.enable)
			self.taskEnabled = True
		elif self.task == TORPEDO_TASK and not self.taskEnabled:
			self.enable.data = True
			self.torpedo_publisher.publish(self.enable)
			self.taskEnabled = True
		
		# Transitions
		if self.resetDepth <= 6:
			# Reset State Variables
			self.taskEnabled  = False
			self.taskComplete = False
			self.resetDepth	  = 0
			self.task	  = GATE_TASK
			return 'reset'

		elif self.taskComplete == True:
			# Reset Task Enable Variables for Next Task
			self.taskEnabled  = False
			self.enable.data  = False
			self.taskComplete = False
			self.task 	 = GATE_TASK	 # Dummy number to prevent previous task from starting again
			return 'taskcomplete'
		else: 
			return '!taskcomplete'


##-------------------------- END STATE DEFINITIONS ------------------------------------##

def main():
	# Initialize node with desired node name - ideally task name
	rospy.init_node('my_task_statemachine')

	# Create a SMACH state machine
	sm = smach.StateMachine(outcomes=['competition_complete'])

	# Create and start introspection server - fancy way of saying view gui feature
	sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
	sis.start()

	# Open SMACH container
	with sm:
		# Add states to the container
		smach.StateMachine.add('IDLE', idle(), 
					transitions = {'go':'EXECUTE','!go':'IDLE'} )
		smach.StateMachine.add('TRANSITION', transition(),
					transitions = {'timeup':'SEARCH','!timeup':'TRANSITION','reset':'IDLE'} )
		smach.StateMachine.add('SEARCH', search(),
					transitions = {'taskfound':'EXECUTE','!taskfound':'SEARCH','reset':'IDLE'} )
		smach.StateMachine.add('EXECUTE', execute(),
					transitions = {'taskcomplete':'TRANSITION','!taskcomplete':'EXECUTE','reset':'IDLE'} )

	outcome = sm.execute()
	rospy.spin()	
	sis.stop()

if __name__ == '__main__':
	main()