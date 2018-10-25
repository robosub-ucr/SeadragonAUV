#!/usr/bin/env python

#import roslib; roslib.load_manifest('smach_tutorials')
import rospy
import smach
import smach_ros
from std_msgs.msg import Bool,  Float64, Int16 

# define state init
class init(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['ready','notready'])
	
	self.currYaw_subscriber       = rospy.Subscriber('/yaw_control/state', Float64, self.yaw_callback)
	self.yawPoint_publisher       = rospy.Publisher('/yaw_control/setpoint', Float64, queue_size=10)
	self.yawPoint = Float64()
	self.yawPoint.data = 0.0
			
	self.yawPidEnable_publisher   = rospy.Publisher('/yaw_control/pid_enable', Bool, queue_size =10)
	self.yawEnable = Bool()
	
	self.depthPoint_publisher     = rospy.Publisher('/depth_control/setpoint', Float64, queue_size=10)
	self.depthPoint = Float64()
	
	self.depthPidEnable_publisher = rospy.Publisher('/depth_control/pid_enable', Bool, queue_size=10)
	self.depthEnable = Bool()	

	self.currDepth_subscriber     = rospy.Subscriber('/depth', Int16, self.depth_callback)
	self.depthStart = 0
    
    def depth_callback(self, msg):
	self.depthStart = msg.data	
    
    def yaw_callback(self, msg):
	self.yawPoint.data = msg.data
	
    def execute(self, userdata):
	
	# Push down about two inches to start state machine
	if self.depthStart >= 5:
		# Set yaw setpoint
		self.yawPoint_publisher.publish(self.yawPoint)
		# Enable yaw pid
		self.yawEnable.data = True
		self.yawPidEnable_publisher.publish(self.yawEnable)
	
		# Set depth setpoint FIXME 48 - 4ft   120 - 10ft
		self.depthPoint.data = 18
		self.depthPoint_publisher.publish(self.depthPoint)
		# Enable depth pid
		self.depthEnable.data = True
		self.depthPidEnable_publisher.publish(self.depthEnable)
		
		return 'ready'
	else:
		"""
		# Disable Yaw pid
		self.yawEnable.data   = False
		self.yawPidEnable_publisher.publish(self.yawEnable)
		
		# Disable Depth pid
		self.depthEnable.data = False
		self.depthPidEnable_publisher.publish(self.depthEnable) 
		"""	
	
		return 'notready'

# define state dive
class dive(smach.State):
    def __init__(self):
	smach.State.__init__(self, outcomes=['complete','notcomplete'])	
	self.currDepth_subscriber	= rospy.Subscriber('/depth', Int16, self.depth_callback)
	self.error      = 0
	self.depthPoint = 18
	
    def depth_callback(self, msg):
	self.error      =  self.depthPoint - msg.data
	
    def execute(self, userdata):
	rospy.loginfo('Executing state dive')
	
	print self.error
	
	if self.error > 2:
		return 'notcomplete'
	else:
		return 'complete'
	


# define state transition
class transition(smach.State):
    def __init__(self):
	smach.State.__init__(self, outcomes=['complete','notcomplete'])

        self.yawPidEnable_publisher     = rospy.Publisher('/yaw_control/pid_enable', Bool, queue_size =10)
        self.yawEnable = Bool()	

        self.depthPidEnable_publisher = rospy.Publisher('/depth_control/pid_enable', Bool, queue_size=10)
        self.depthEnable = Bool()
	
	self.fwdThrust_publisher	= rospy.Publisher('/yaw_pwm', Int16, queue_size=10)
	self.fwdThrust = Int16()
	self.fwdThrust.data = 0
	
	self.counter = 0
	
    def execute(self, userdata):
	rospy.loginfo('Executing state transition')
		
	
	if self.counter < 50000:
		self.counter += 1
		if self.counter % 200 == 0:	
			if self.fwdThrust.data < 200:
				self.fwdThrust.data += 1
		self.fwdThrust_publisher.publish(self.fwdThrust)
		return 'notcomplete'
	else:
		#FIXME
		# Disable yaw pid
		self.yawEnable.data = False
		self.yawPidEnable_publisher.publish(self.yawEnable)
		# Disable depth pid
		self.depthEnable.data = False
		self.depthPidEnable_publisher.publish(self.depthEnable)
		# Disable fwd thrust
		self.fwdThrust.data = 0
		self.fwdThrust_publisher.publish(self.fwdThrust)
		return 'complete'
	

	

# define state search
class search(smach.State):
    def __init__(self):
	smach.State.__init__(self, outcomes=['targetlocked','notarget'])
	self.counter = 0
	 
    def execute(self, userdata):
	rospy.loginfo('Executing state search')

	if self.counter < 5000:
		self.counter +=1
		return 'notarget'
	else:
		return 'targetlocked'
	
	# FIXME
	# Enable computer vision
	# Search for orange pole


#define state execute
class execute(smach.State):
    def __init__(self):
	smach.State.__init__(self, outcomes=['complete','notcomplete'])	
	self.counter = 0

    def execute(self, userdata):
	rospy.loginfo('Executing state execute')
	
	if self.counter < 5000:
		self.counter += 1
		return 'notcomplete'
	else:
		return 'complete'
		
	# FIXME
	# make 180 around pole


	
	
def main():
    rospy.init_node('qualify_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['qualify'])

    # Create and start introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()
    
    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('INIT', init(), 
                               transitions={'ready':'DIVE','notready':'INIT'})
        smach.StateMachine.add('DIVE', dive(), 
                               transitions={'complete':'TRANSITION','notcomplete':'DIVE'})
	smach.StateMachine.add('TRANSITION', transition(), 
		 	       transitions={'complete':'qualify','notcomplete':'TRANSITION'})
	smach.StateMachine.add('SEARCH', search(),
			       transitions={'targetlocked':'EXECUTE','notarget':'SEARCH'})
	smach.StateMachine.add('EXECUTE', execute(),
			       transitions={'complete':'TRANSITION','notcomplete':'EXECUTE'})
	
    # Execute SMACH plan
    outcome = sm.execute()
 
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
