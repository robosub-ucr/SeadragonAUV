

#!/usr/bin/env python

##------------------------------- IMPORTS ---------------------------------------##

import numpy as np
import rospy
import smach
import smach_ros
from std_msgs.msg import Bool, Float64, Int16

##----------------------------- END IMPORTS -------------------------------------##


##------------------------- STATE DEFINITIONS -----------------------------------##

# Define State idle

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
			
		# Publisher Data Containers
		self.taskReset			= Bool()
		self.taskReset.data		= True
		self.Disable			= Bool()
		self.Disable.data 		= False
		self.fwdThrust			= Int16()
		self.fwdThrust.data 		= 0

		# Local variables
		self.depthStart     = 0
		self.systemDisabled = False

	def depth_callback(self,msg):
		self.depthStart = msg.data

	def execute(self, userdata):
		
		# Disable Motor control System & Task statemachines
		if self.systemDisabled == False:
			self.fwdThrust_publisher.publish(self.fwdThrust)
			self.taskReset_publisher.publish(self.taskReset)
			self.yawPidEnable_publisher.publish(self.Disable)
			self.depthPidEnable_publisher.publish(self.Disable)
			self.systemDisabled = True
		
		# Check if sub has been pushed down 'x' inches to begin run
		if self.depthStart > 12:
			self.systemDisabled = False
			return 'go'

		else:
			return '!go'



# Define State Transition

class transition(smach.State):

	def __init__(self):

		smach.State.__init__(self, outcomes=['timeup','!timeup','reset'])

		# Subscribers
		self.depth_subscriber		= rospy.Subscriber('/depth', Int16, self.depth_callback) 
		
		# Publishers
		self.fwdThrust_publisher	= rospy.Publisher('/yaw_pwm', Int16, queue_size=10)
		
		# Publisher Data Containers
		self.fwdThrust			= Int16()
		self.fwdThrust.data		= 0

		# Local Variables
		self.timer = 0
		self.resetDepth = 0


	def depth_callback(self,msg):

		self.resetDepth = msg.data



	def execute(self, userdata):

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

		elif self.timer > 80000:
			self.timer = 0
			# Stop Sub in preparation for search
			self.fwdThrust.data = 0
			self.fwdThrust_publisher.publish(self.fwdThrust)
			return 'timeup'

		else:
			return '!timeup'			



# Define State Search

class search(smach.State):

        def __init__(self):

                smach.State.__init__(self, outcomes=['taskfound','!taskfound','reset'])

                # Publishers, Subscribers
                self.reset_subscriber = rospy.Subscriber('/reset', Bool, self.reset_callback)

                # Local Variables
                self.timer = 0
                self.reset = False


        def reset_callback(self,msg):

                self.reset = msg.data


        def execute(self, userdata):

                self.timer += 1

                if self.reset == True:

                        return 'reset'

                elif self.timer > 20000:

                        self.timer = 0

                        return 'taskfound'

                else:

                        return '!taskfound'



# Define State Execute

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

		# Local variables	
		self.task	  = 0
		self.taskEnabled  = False
		self.taskComplete = False	
		self.resetDepth   = 0

	def reset_callback(self,msg):

                self.resetDepth   = msg.data

	def task_callback(self,msg):

		self.task	  = msg.data	
	
	def complete_callback(self,msg):
		
		self.taskComplete = msg.data	
			
	def execute(self, userdata):

		# Actions
		if self.task == 0 and self.taskEnabled == False:

			# Enable Gate Task
			self.enable.data = True
			self.gate_publisher.publish(self.enable)
			self.taskEnabled = True

		elif self.task == 1 and self.taskEnabled == False:

			# Enable Buoy Task
			self.enable.data = True
			self.buoy_publisher.publish(self.enable)
			self.taskEnabled = True

		elif self.task == 2 and self.taskEnabled == False:

			# Enable Torpedo Task
			self.enable.data = True
			self.torpedo_publisher.publish(self.enable)
			self.taskEnabled = True
		
		# Transitions
		if self.resetDepth <= 6:
			# Reset State Variables
			self.taskEnabled  = False
			self.taskComplete = False
			self.resetDepth	  = 0
			self.task	  = 0
			return 'reset'

		elif self.taskComplete == True:
			# Reset Task Enable Variables for Next Task
			self.taskEnabled  = False
			self.enable.data  = False
			self.taskComplete = False
			self.task 	 = 3	 # Dummy number to prevent previous task from starting again
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
