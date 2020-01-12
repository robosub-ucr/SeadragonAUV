#!/usr/bin/env python
# ^ Execute with a Python interpreter, using the program search path to find it

#import numpy as np
import rospy
import smach
import smach_ros
import sys
from std_msgs.msg import Int16
import seasdragon_states as state

# define state S1_start
class start(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=["done"])

	def execute(self, userdata):
		rospy.loginfo("Executing state start")
		print("start executing")
		#initialize any variables?

		return "done"

# define state S2_check_depth
class check_depth(smach.State):
	def __init__(self, target_depth):
		smach.State.__init__(self, outcomes=["done", "loop"])
		self.target_depth = target_depth
		self.depth = 0
		rospy.Subscriber("depth_control/state", Int16, self.depth_callback)

	def execute(self, userdata):
		rospy.loginfo("Executing state check_depth")
		print("depth: ", self.depth)
		if(self.depth < self.target_depth):
			#self.depth += 1 #simulate Robosub diving down, decreasing depth
			return "loop"
		return "done"

	def depth_callback(self, msg):
		self.depth = msg.data

class maintain_depth(smach.State):
	def __init__(self, state, target_depth):
		smach.State.__init__(self, outcomes=["done", "loop"])
		state.ChangeDepthToTarget(target_depth)
    
    
class WaitForTopicEqual(smach.State):
    def __init__(self, topic_name, topic_datatype, value):
      smach.State.__init__(self, outcomes=['done', 'notdone'])
      self.reset = False
      self.data = None
      
      rospy.Subscriber("/reset", Bool, self.reset_callback)
      rospy.Subscriber(topic_name, topic_datatype, self.topic_callback)
      
    def reset_callback(self, msg):
			self.reset = msg.data
      
    def execute(self, userdata):
      if self.reset:
        return "reset"
      
      if self.data == value:
        return "done"
      else:
        return "notdone"
      
    def topic_callback(self, msg):
      self.data = msg.data

def main():
	# initial arguments
	arg_dict = {
		"target_depth" : 12, # desired depth is 12 inches = 1 foot below the surface of the water
		"target_length" : , #
		"yaw_PWM" : 0, # signal strength given to yaw thrusters to change planar angle
	}
	if(len(sys.argv) <= 1):
		print("Error: No arguments!")
		exit()
	elif(len(sys.argv) > 1):
		for arg in sys.argv[1:]:
			key, value = arg.split("=")
			arg_dict[key] = float(value)

	#print("target_depth: ", arg_dict["target_depth"])

	# Initialize node with desired node name - ideally task name
	rospy.init_node("SM_poolTest")

	# Create a SMACH state machine
	sm = smach.StateMachine(outcomes=["done"])
	
	# Create and start introspection server - fancy way of saying view gui feature
	sis = smach_ros.IntrospectionServer("introspection_server", sm, "/SM_POOL_TEST")
	sis.start()
  
  START_DEPTH = 12 # in inches, the depth at which the state machine will start

	# Open the container
	with sm:
    # Check that the sub's depth is at 12 inches below surface, then move to the next state
    smach.StateMachine.add("SM_CheckDepth", WaitForTopicEqual("/depth_control/state", Int16, START_DEPTH),
                           transitions={"done":"SM_MaintainDepth",
                                       "notdone": "SM_CheckDepth",
                                       "reset": "SM_Reset"})
		# Maintain its current depth by setting the depth_setpoint to the current depth
    # Turn on the depth_pid
		smach.StateMachine.add("SM_MaintainDepth", STATE_CLASS(),
                           transitions={"done":"__NEXT_STATE__",
                                       "notdone": "SM_MaintainDepth",
                                       "reset": "__RESET_STATE__"})
    
    smach.StateMachine.add("SM_RotateLeft90", state.RotateYawToRelativeTarget(1.57), 
                          transitions={"done":"??????",
                                      "notdone":"SM_RotateLeft90",
                                      "reset":"SM_Reset"})
    
    smach.StateMachine.add("SM_Reset", state.Reset(), 
                          transitions={"done": "SM_CheckDepth"})
    
		# Add states to the container
		# smach.StateMachine.add("start", start(),
		# 	transitions={"done":"check_depth"})
		# smach.StateMachine.add("check_depth", check_depth(arg_dict["target_depth"]),
		# 	transitions={"done":"done", "loop":"check_depth"})
		# smach.StateMachine.add("maintain_depth", state.)

	# Execute SMACH plan
	outcome = sm.execute()

	# spin?
	rospy.spin()
	sis.stop()

# main guard
if __name__ == "__main__":
	main()
