import numpy as np
import rospy
import smach
import smach_ros
from std_msgs.msg import Bool, Float64, Int16

FORWARD_THRUST_MAX = 280
FORWARD_THRUST_CHANGE = 1
DEPTH_CHANGE = 1 # in inches
DEPTH_VARIANCE = 1
YAW_VARIANCE = 0.017 # in radians (0.017 = 1 radian)
YAW_CHANGE = 0.017

class Idle(smach.State):
	def __init__(self, topic):
		smach.State.__init__(self, outcomes=['done', 'notdone'])

		self.enabled = False
		rospy.Subscriber(topic, Bool, self.enabled_callback)

	def enabled_callback(self, msg):
		self.enabled = msg.data

	def execute(self, userdata):
		if self.enabled:
			self.reset_values()
			return 'done'
		else:
			return 'notdone'

	def reset_values(self):
		self.enabled = False


class Finish(smach.State):
	def __init__(self, topic):
		smach.State.__init__(self, outcomes=['done'])

		self.topic_publisher = rospy.Publisher(topic, Bool, queue_size=10)

	def execute(self, userdata):
		finish = Bool()
		finish.data = True
		self.topic_publisher.publish(finish)
		return 'done'


class TimedWait(smach.State):
	# This state causes the robot to do nothing for a set amount of ticks
	# Arguments:
	#   duration (int): amount of ticks this state lasts
	def __init__(self, duration):
		smach.State.__init__(self, outcomes=['done', 'notdone', 'reset'])

		self.duration = duration

		self.ticks = 0
		self.reset = False
		rospy.Subscriber('/reset', Bool, self.reset_callback)

	def reset_callback(self, msg):
		self.reset = msg.data

	def execute(self, userdata):
		if self.reset:
			self.reset_values()
			return 'reset'

		self.ticks += 1
		if self.ticks >= self.duration:
			self.reset_values()
			return 'done'
		else:
			return 'notdone'

	def reset_values(self):
		self.ticks = 0
		self.reset = False

class ChangeDepthToTarget(smach.State):
	# This state causes the robot to change its depth until it is within certain distance of the target depth
	# 
	# Arguments:
	#   depth_target (int): the target depth the robot will move to
	def __init__(self, depth_target):
		smach.State.__init__(self, outcomes=['done', 'notdone', 'reset'])

		self.depth_target = depth_target

		self.depth_received = False
		self.depth = 0
		self.reset = False
		rospy.Subscriber('/depth_control/state', Int16, self.depth_callback)
		rospy.Subscriber('/reset', Bool, self.reset_callback)

		self.depth_publisher = rospy.Publisher('/depth_control/setpoint', Int16, queue_size=10)

	def depth_callback(self, msg):
		self.depth = msg.data
		self.depth_received = True

	def reset_callback(self, msg):
		self.reset = msg.data

	def execute(self, userdata):
		if self.reset:
			self.reset_values()
			return 'reset'

		if not self.depth_received:
			return 'notdone'

		if abs(self.depth - self.depth_target) < DEPTH_VARIANCE: # Note: depth == depth_target is included in this condition
			self.reset_values()
			return 'done'
		
		new_depth = Int16()
		if self.depth < self.depth_target:
			new_depth.data = self.depth + DEPTH_CHANGE
		else: # depth > depth_target
			new_depth.data = self.depth - DEPTH_CHANGE
		self.depth_publisher.publish(new_depth)
		return 'notdone'

	def reset_values(self):
		self.depth = 0
		self.reset = False
		self.depth_received = False


class ChangeDepthTimed(smach.State):
	# This state causes the robot to change its depth for a set amount of ticks
	# 
	# Arguments:
	#   duration (int): amount of ticks this state lasts
	#   is_downward (bool): determines if depth increases or decreases
	def __init__(self, duration, is_downward=True):
		smach.State.__init__(self, outcomes=['done', 'notdone', 'reset'])

		self.ticks = 0
		self.duration = duration
		self.is_downward = is_downward

		self.depth = 0
		self.depth_received = False
		self.reset = False
		rospy.Subscriber('/depth_control/state', Int16, self.depth_callback)
		rospy.Subscriber('/reset', Bool, self.reset_callback)

		self.depth_publisher = rospy.Publisher('/depth_control/setpoint', Int16, queue_size=10)

	def depth_callback(self, msg):
		self.depth = msg.data
		self.depth_received = True

	def reset_callback(self, msg):
		self.reset = msg.data

	def execute(self, userdata):
		if self.reset:
			self.reset_values()
			return 'reset'

		if not self.depth_received:
			return 'notdone'

		self.ticks += 1
		if self.ticks >= self.duration:
			self.reset_values()
			return 'done'
		else:
			new_depth = Int16()
			if self.is_downward:
				new_depth.data = self.depth + DEPTH_CHANGE
			else:
				new_depth.data = self.depth - DEPTH_CHANGE
			self.depth_publisher.publish(new_depth)
			return 'notdone'

	def reset_values(self):
		self.depth = 0
		self.depth_received = False
		self.reset = False
		self.ticks = 0


class RotateYawTimed(smach.State):
	# This state causes the robot to rotate about the y-axis for a set amount of time
	# Arguments:
	#   ticks (int)
	#   is_clockwise (bool): determines if the robot rotates clockswise or counterclockwise
	def __init__(self, duration, is_clockwise=True):
		smach.State.__init__(self, outcomes=['done', 'notdone', 'reset'])

		self.duration = duration
		self.is_clockwise = is_clockwise

		self.ticks = 0
		self.yaw = 0
		self.yaw_received = False
		self.reset = False
		rospy.Subscriber('/yaw_control/state', Float64, self.yaw_callback)
		rospy.Subscriber('/reset', Bool, self.reset_callback)

		self.yaw_publisher = rospy.Publisher('yaw_control/setpoint', Float64, queue_size=10)

	def reset_callback(self, msg):
		self.reset = msg.data

	def yaw_callback(self, msg):
		self.yaw = msg.data
		self.yaw_received = True

	def execute(self, userdata):
		if self.reset:
			self.reset_values()
			return 'reset'

		if not self.yaw_received:
			return 'notdone'

		self.ticks += 1
		if self.ticks >= self.duration:
			self.reset_values()
			return 'done'
		
		new_yaw = Float64()
		if self.is_clockwise:
			new_yaw.data = self.yaw + YAW_CHANGE
		else:
			new_yaw.data = self.yaw - YAW_CHANGE
		self.yaw_publisher.publish(new_yaw)
		return 'notdone'

	def reset_values(self):
		self.ticks = 0
		self.yaw = 0
		self.yaw_received = False
		self.reset = False


class RotateYawToRelativeTarget(smach.State):
	def __init__(self, yaw_change):
		smach.State.__init__(self, outcomes=['done', 'notdone', 'reset'])

		self.yaw_change = yaw_change

		self.yaw = 0
		self.yaw_target = 0
		self.yaw_received = False
		self.reset = False
		rospy.Subscriber('/yaw_control/state', Float64, self.yaw_callback)
		rospy.Subscriber('/reset', Float64, self.reset_callback)

		self.yaw_publisher = rospy.Publisher('/yaw_control/setpoint', Float64, queue_size=10)

	def reset_callback(self, msg):
		self.reset = msg.data

	def yaw_callback(self, msg):
		self.yaw = msg.data
		self.yaw_received = True
		self.yaw_target = self.yaw + self.yaw_change

	def execute(self, userdata):
		if self.reset:
			self.reset_values()
			return 'reset'

		if not self.yaw_received:
			return 'notdone'

		if abs(self.yaw - self.yaw_target) < YAW_VARIANCE:
			self.reset_values()
			return 'done'

		new_yaw = Int16()
		if self.yaw < self.yaw_target:
			new_yaw.data = self.yaw + YAW_CHANGE
		else: # yaw > yaw_target
			new_yaw.data = self.yaw - YAW_CHANGE
		self.yaw_publisher.publish(new_yaw)
		return 'notdone'

	def reset_values(self):
		self.yaw = 0
		self.yaw_target = 0
		self.yaw_received = False
		self.reset = False


class RotateYawToAbsoluteTarget(smach.State):
	# This state causes the robot to rotate towards a target
	def __init__(self, yaw_target):
		smach.State.__init__(self, outcomes=['done', 'notdone', 'reset'])

		self.yaw_target = yaw_target

		self.yaw = 0
		self.yaw_received = False
		self.reset = False
		rospy.Subscriber('/yaw_control/state', Float64, self.yaw_callback)
		rospy.Subscriber('/reset', Float64, self.reset_callback)

		self.yaw_publisher = rospy.Publisher('/yaw_control/setpoint', Float64, queue_size=10)

	def reset_callback(self, msg):
		self.reset = msg.data

	def yaw_callback(self, msg):
		self.yaw = msg.data
		self.yaw_received = True

	def execute(self, userdata):
		if self.reset:
			self.reset_values()
			return 'reset'

		if not self.yaw_received:
			return 'notdone'

		if abs(self.yaw - self.yaw_target) < YAW_VARIANCE:
			self.reset_values()
			return 'done'

		new_yaw = Int16()
		if self.yaw < self.yaw_target:
			new_yaw.data = self.yaw + YAW_CHANGE
		else: # yaw > yaw_target
			new_yaw.data = self.yaw - YAW_CHANGE
		self.yaw_publisher.publish(new_yaw)
		return 'notdone'

	def reset_values(self):
		self.yaw = 0
		self.reset = False
		self.yaw_received = False

class MoveForwardTimed(smach.State):
	# This state causes the robot to set its forward thrust to zero
	# and slowly changes the forward thrust to be higher or lower (up to a maximum)
	# After some duration, the robot goes to the next state

	# Arguments:
	#   duration (int):     amount of ticks this state lasts
	#   forward_step (int): how much thrust is increased/decreased per tick
	#   is_forward (bool):  determines if thrust increases or decreases
	def __init__(self, duration, is_forward=True):
		smach.State.__init__(self, outcomes=['done', 'notdone', 'reset'])

		self.ticks = 0
		self.duration = duration
		self.forward_change = forward_change
		self.is_forward = is_forward

		self.forward_thrust_publisher = rospy.Publisher('/yaw_pwn', Int16, queue_size=10)
		self.forward_thrust = Int16() # Keeps track of forward thrust. Required because there's no topic to subcribe to for forward thrust

		self.reset = False
		rospy.Subscriber('/reset', Bool, self.reset_callback)

	def reset_callback(self, msg):
		self.reset = msg.data

	def execute(self, userdata):
		# If '/reset' is true, go to the reset state
		if self.reset:
			self.reset_values()
			return 'reset'

		# When ticks reaches duration, go to the next state
		self.ticks += 1
		if self.ticks >= self.duration:
			self.reset_values()
			return 'done'

		# Else, increase or decrease forward thrust every 200 ticks
		elif self.ticks % 200 != 0:
			if self.is_forward:
				self.change_forward_thrust(FORWARD_THRUST_CHANGE)
			else:
				self.change_forward_thrust(-FORWARD_THRUST_CHANGE)
			# Publish the new forward thrust
			self.forward_thrust_publisher.publish(self.forward_thrust)

		# If not reset or done, stay on this state
		return 'notdone'

	def change_forward_thrust(self, amount):
		# Change forward thrust by amount (can be positive or negative)
		# Dont allow it to increase/decrease beyond a max amount
		# This is to prevent thrusters from going too fast
		self.forward_thrust.data = self.forward_thrust.data + amount
		if self.forward_thrust.data > FORWARD_THRUST_MAX:
			self.forward_thrust.data = FORWARD_THRUST_MAX
		elif self.forward_thrust.data < -1 * FORWARD_THRUST_MAX:
			self.forward_thrust.data = -1 * FORWARD_THRUST_MAX

	def reset_values(self):
		# This function must be called in execute() immediately before return.
		# Except when returning 'notdone' or any value that keeps you in this state
		self.ticks = 0
		self.forward_thrust.data = 0
		self.reset = False

