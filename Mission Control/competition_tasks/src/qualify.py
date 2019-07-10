#!/usr/bin/env python

#import roslib; roslib.load_manifest('smach_tutorials')
import rospy
import smach
import smach_ros
from std_msgs.msg import Bool,  Float64, Int16 

from decimal import *
import math
import numpy as np

QUALIFY_DEPTH  = 72	# 6ft
PRACTICE_DEPTH = 36	# 3ft
START_DEPTH    = 12	# 1ft
RESET_DEPTH    = 6	# .5ft

# define state init
class init(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['ready','notready'])
	
	self.yawPoint_publisher		= rospy.Publisher('/yaw_control/setpoint', Float64, queue_size=10)
	self.yawPoint 			= Float64()
	self.yawPoint.data 		= 0.0
	
	self.depthPoint_publisher     	= rospy.Publisher('/depth_control/setpoint', Float64, queue_size=10)
	self.depthPoint 		= Float64()
	self.depthPoint.data 		= PRACTICE_DEPTH

	self.yawPidEnable_publisher	= rospy.Publisher('/yaw_control/pid_enable', Bool, queue_size =10)
	self.depthPidEnable_publisher 	= rospy.Publisher('/depth_control/pid_enable', Bool, queue_size=10)

	self.Enable 			= Bool()
	self.Enable.data 		= True

	self.currDepth_subscriber     	= rospy.Subscriber('/depth', Int16, self.depth_callback)
	self.depthStart = 0
    
    def depth_callback(self, msg):
	self.depthStart = msg.data	
    
    def yaw_callback(self, msg):
	self.yawPoint.data = msg.data

    def enable_controllers(self):
	# Set yaw setpoint
	self.yawPoint_publisher.publish(self.yawPoint)

	# Enable yaw pid
	self.yawPidEnable_publisher.publish(self.Enable)
	
	# Set depth setpoint  
	self.depthPoint_publisher.publish(self.depthPoint)

	# Enable depth pid
	self.depthPidEnable_publisher.publish(self.Enable)

    def reset_variables(self):
	self.depthStart    = 0
	self.yawPoint.data = 0
	
    def execute(self, userdata):

	# Push sub down about two inches to start state machine
	if self.depthStart >= START_DEPTH:
		self.enable_controllers()
		self.reset_variables()
		return 'ready'
	else:
		return 'notready'


# define state dive
class dive(smach.State):
    def __init__(self):
	smach.State.__init__(self, outcomes=['complete','notcomplete','reset'])	
	
	self.currDepth_subscriber	= rospy.Subscriber('/depth', Int16, self.depth_callback)
	self.resetDepth 		= 0	
	self.error      		= 0
	
    def depth_callback(self, msg):
	self.error      =  PRACTICE_DEPTH - msg.data
	self.resetDepth =  msg.data
	
    def reset_variables(self):
	self.error 	= 0
	self.resetDepth = 0

    def execute(self, userdata):
	
	# Check if kill Switch has been triggered to restart state machine
	if self.resetDepth < RESET_DEPTH:
		self.reset_variables()
		return 'reset'
		
	# Check if sub has reached its desired depth point
	if self.error <=2:
		self.reset_variables()
		return 'complete'
	else:
		return 'notcomplete'


# define state transition
class transition(smach.State):
    def __init__(self, qualify):
	smach.State.__init__(self, outcomes=['complete','notcomplete','reset'])

	self.currDepth_subscriber	= rospy.Subscriber('/depth', Int16, self.depth_callback)
	self.resetDepth 		= 0

	self.Disable 			= Bool()
	self.Disable.data 		= False

        self.yawPidEnable_publisher     = rospy.Publisher('/yaw_control/pid_enable', Bool, queue_size =10)
        self.depthPidEnable_publisher 	= rospy.Publisher('/depth_control/pid_enable', Bool, queue_size=10)

	self.fwdThrust_publisher	= rospy.Publisher('/yaw_pwm', Int16, queue_size=10)
	self.fwdThrust 			= Int16()
	self.fwdThrust.data 		= 0

	self.qualify = qualify
	self.counter = 0

    def depth_callback(self, msg):
	self.resetDepth =  msg.data

    def reset_variables(self):
	self.counter 	= 0
	self.resetDepth = 0	

    def reset_controllers(self):
	# Disable yaw pid
	self.yawPidEnable_publisher.publish(self.Disable)

	# Disable depth pid
	self.depthPidEnable_publisher.publish(self.Disable)

	# Disable fwd thrust
	self.fwdThrust.data = 0
	self.fwdThrust_publisher.publish(self.fwdThrust)
	
    def execute(self, userdata):

	# Check if kill Switch has been triggered to restart state machine
	if self.resetDepth < RESET_DEPTH:
		self.reset_variables()
		return 'reset'
	
	# Accelerate until max cruising velocity
	if self.counter < 50000:
		self.counter += 1
		if self.counter % 200 == 0:	
			if self.fwdThrust.data < 280:
				self.fwdThrust.data += 1
		self.fwdThrust_publisher.publish(self.fwdThrust)
		return 'notcomplete'
	else:
		# If qualify is True then the sub is on the return path. 
		# So once complete, reset the controllers to end run.
		if self.qualify == True:
			self.reset_controllers()
		self.reset_variables()	
		return 'complete'
	

# define state Turn
class turn(smach.State):
    def __init__(self):
	smach.State.__init__(self, outcomes=['done','notdone','reset'])
	self.counter = 0

	# Subscribers
	self.depth_subscriber 		= rospy.Subscriber('/depth', Int16, self.depth_callback) 
	self.yaw_subscriber 		= rospy.Subscriber('/yaw_control/state', Float64, self.yaw_callback)

	self.yawOrientation_publisher 	= rospy.Publisher('/yaw_control/setpoint', Float64, queue_size=10)	
	
	self.yawSetpoint  		= Float64()
	self.turnRange	  		= 3.145  # 360 degrees
	self.resetDepth   		= 0
	self.currYaw 	  		= 0
	self.yawBegin			= 0
	self.yawRecorded		= False
	self.t 				= 0
	self.turnState			= 0

    def depth_callback(self,msg):
		self.resetDepth = msg.data
    def yaw_callback(self,msg):
		self.currYaw = msg.data

    def map(self, x, in_min, in_max, out_min, out_max):
   		return (x-in_min) * (out_max-out_min) / (in_max-in_min) + out_min

    def scan(self, T, thetaStart, thetaEnd):
		
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
			self.turnState += 1
			self.t = 0

    def reset_variables(self):
		self.resetDepth   = 0
		self.currYaw  	  = 0
		self.yawBegin 	  = 0
		self.t 		  = 0
		self.yawRecorded  = False
		self.turnState    = 0
	 
    def execute(self, userdata):

	# Check if kill Switch has been triggered to restart state machine
	if self.resetDepth < RESET_DEPTH:
		self.reset_variables()
		return 'reset'
	
	# Check what the sub's current orientation is
	if self.yawRecorded == False:
		self.yawBegin = self.currYaw
		self.yawRecorded = True

	# Once sub know's what it's current orientation is, begin turning by 360 degrees
	if self.yawRecorded == True:

		# Handle's Turn's at discontinuities [Added constraint for only right turns]
		# Uses 3rd order polynomial for smooth trajectory
		if self.yawBegin > -1:
			if self.turnState == 0:
				self.scan(2000, self.yawBegin, 3.14)
			else:
				self.scan(2000, -3.14, (self.yawBegin - 3.12) )
		elif self.yawBegin < 0:
			if self.turnState == 0:
				self.scan(2000, self.yawBegin, 0)
			else:
				self.scan(2000, 0, (self.yawBegin + 3.12) )
		
	# Once 'scan' sets turnComplete flag 'True' proceed to transition
	if self.turnState == 2:
		self.reset_variables()
		return 'done'
	else:
		return 'notdone'

# Define state Reset
class reset(smach.State):
    def __init__(self):
	smach.State.__init__(self, outcomes=['complete','notcomplete'])
	self.resetDone = False
		
	self.Disable 	= Bool()
	self.Disable.data = False

	self.yawPidEnable_publisher     = rospy.Publisher('/yaw_control/pid_enable', Bool, queue_size =10)
        self.depthPidEnable_publisher 	= rospy.Publisher('/depth_control/pid_enable', Bool, queue_size=10)
	
	self.fwdThrust_publisher	= rospy.Publisher('/yaw_pwm', Int16, queue_size=10)
	self.fwdThrust = Int16()
	self.fwdThrust.data = 0

    def reset_controllers(self):
	# Disable yaw pid
	self.yawPidEnable_publisher.publish(self.Disable)

	# Disable depth pid
	self.depthPidEnable_publisher.publish(self.Disable)

	# Disable fwd thrust
	self.fwdThrust_publisher.publish(self.fwdThrust)
	
	# Mark Reset as complete
	self.resetDone = True
	 
    def execute(self, userdata):

	# Sets 'resetDone' as True when executed
	self.reset_controllers()

	# If reset procedure is complete return to Init state
	if self.resetDone == True:
		self.resetDone = False
		return 'complete'
	else:
		return 'notcomplete'	

	
def main():
    rospy.init_node('qualify_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['qualify'])

    # Create and start introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_PREQ')
    sis.start()
    
    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('INIT', init(), 
                               transitions={'ready':'DIVE','notready':'INIT'})

        smach.StateMachine.add('DIVE', dive(), 
                               transitions={'complete':'FWD','notcomplete':'DIVE','reset':'RESET'})

	smach.StateMachine.add('FWD', transition(False), 
		 	       transitions={'complete':'TURN','notcomplete':'FWD','reset':'RESET'})

	smach.StateMachine.add('TURN', turn(),
			       transitions={'done':'BACK','notdone':'TURN','reset':'RESET'})

	smach.StateMachine.add('BACK', transition(True), 
		 	       transitions={'complete':'qualify','notcomplete':'BACK','reset':'RESET'})

	smach.StateMachine.add('RESET', reset(), 
		 	       transitions={'complete':'INIT','notcomplete':'RESET'})	
	
    # Execute SMACH plan
    outcome = sm.execute()
 
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
