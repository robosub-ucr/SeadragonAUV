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

CAMERA_WIDTH = 400
CAMERA_HEIGHT = 300
CENTER_PADDING_X = 5
CENTER_PADDING_Y = 5

AREA_THRESHOLD_LOW = 0.12
AREA_THRESHOLD_HIGH = 0.15

# Reset 					currently does nothing
# WaitForTopic 				waits until Bool topic is True
# PublishTopic 				publishes topic
# WaitTimed 				wait for x ticks
# ChangeDepthToTarget 		increase/decrease depth 
# ChangeDepthTimed 			increase/decrease depth 
# RotateYawTimed 			rotate yaw for x ticks
# RotateYawToRelativeTarget rotate yaw until it reaches x rotation relative to initial (at start of this state) rotation
# RotateYawToAbsoluteTarget rotate yaw until it reaches x rotation
# MoveForwardTimed 			increase/decrease thrust for x ticks
# TrackObject 				change thrust, rotate yaw, and change depth until the object's (x,y) is within center of camera and until object's area is within threshold

class Reset(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['done'])

	def execute(self, userdata):
		# Currently, this state does nothing
		return 'done'


class WaitForTopic(smach.State):
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


class PublishTopic(smach.State):
	# This state publishes a value to a topic and then it immediately moves to the next state
	# 
	# Arguments:
	#   topic (string)
	#   value (bool)
	def __init__(self, topic, value):
		smach.State.__init__(self, outcomes=['done'])

		self.value = value
		self.topic_publisher = rospy.Publisher(topic, Bool, queue_size=10)

	def execute(self, userdata):
		flag = Bool()
		flag.data = self.value
		self.topic_publisher.publish(flag)
		return 'done'


class WaitTimed(smach.State):
	# This state causes the robot to do nothing for a certain duration
	# 
	# Arguments:
	#   duration (int): amount of ticks that this state lasts
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


class WaitUntilTopicDataReachesTarget(smach.State):
	def __init__(self, topic, datatype, target, variance, minimum_time):
		smach.State.__init__(self, outcomes=['done', 'notdone', 'reset'])

		self.topic = topic
		self.datatype = datatype
		self.target = target
		self.variance = variance
		self.minimum_time = minimum_time

		self.current = 0
		self.timer = 0
		
		rospy.Subscriber(self.topic, self.datatype, self.state_callback)

	def state_callback(self, msg):
		self.current = msg.data
		self.data_received = True

	def execute(self, userdata):
		if self.data_received and abs(self.current - self.target) <= self.variance:
			self.timer += 1

		if self.timer >= self.minimum_time:
			return 'done'
		else:
			return 'notdone'

	def reset_values(self):
		self.current = 0
		self.timer = 0


class ChangeDepthToTarget(smach.State):
	# This state causes the robot to change its depth until it is within certain distance of the target depth
	# 
	# Arguments:
	#   depth_target (int): the target depth the robot will move to
	def __init__(self, depth_target):
		smach.State.__init__(self, outcomes=['done', 'notdone', 'reset'])

		self.depth_target = Int16()
		self.depth_target.data = depth_target

		self.depth_received = False
		self.depth_published = False
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

		if not self.depth_published:
			self.depth_publisher.publish(self.depth_target)
			self.depth_published = True
			return 'notdone'

		if abs(self.depth - self.depth_target.data) < DEPTH_VARIANCE: # Note: depth == depth_target is included in this condition
			self.reset_values()
			return 'done'
		else:
			return 'notdone'

	def reset_values(self):
		self.depth = 0
		self.reset = False
		self.depth_received = False
		self.depth_published = False


# class ChangeDepthTimed(smach.State):
# 	# This state causes the robot to change its depth for a set amount of ticks
# 	# 
# 	# Arguments:
# 	#   duration (int): amount of ticks this state lasts
# 	#   is_downward (bool): determines if depth increases or decreases
# 	def __init__(self, duration, is_downward=True):
# 		smach.State.__init__(self, outcomes=['done', 'notdone', 'reset'])

# 		self.ticks = 0
# 		self.duration = duration
# 		self.is_downward = is_downward

# 		self.depth = 0
# 		self.depth_received = False
# 		self.reset = False
# 		rospy.Subscriber('/depth_control/state', Int16, self.depth_callback)
# 		rospy.Subscriber('/reset', Bool, self.reset_callback)

# 		self.depth_publisher = rospy.Publisher('/depth_control/setpoint', Int16, queue_size=10)

# 	def depth_callback(self, msg):
# 		self.depth = msg.data
# 		self.depth_received = True

# 	def reset_callback(self, msg):
# 		self.reset = msg.data

# 	def execute(self, userdata):
# 		if self.reset:
# 			self.reset_values()
# 			return 'reset'

# 		if not self.depth_received:
# 			return 'notdone'

# 		self.ticks += 1
# 		if self.ticks >= self.duration:
# 			self.reset_values()
# 			return 'done'
# 		else:
# 			new_depth = Int16()
# 			if self.is_downward:
# 				new_depth.data = self.depth + DEPTH_CHANGE
# 			else:
# 				new_depth.data = self.depth - DEPTH_CHANGE
# 			self.depth_publisher.publish(new_depth)
# 			return 'notdone'

# 	def reset_values(self):
# 		self.depth = 0
# 		self.depth_received = False
# 		self.reset = False
# 		self.ticks = 0


# class RotateYawTimed(smach.State):
# 	# This state causes the robot to rotate about the y-axis for a set amount of time
# 	# 
# 	# Arguments:
# 	#   ticks (int)
# 	#   is_clockwise (bool): determines if the robot rotates clockswise or counterclockwise
# 	def __init__(self, duration, is_clockwise=True):
# 		smach.State.__init__(self, outcomes=['done', 'notdone', 'reset'])

# 		self.duration = duration
# 		self.is_clockwise = is_clockwise

# 		self.ticks = 0
# 		self.yaw = 0
# 		self.yaw_received = False
# 		self.reset = False
# 		rospy.Subscriber('/yaw_control/state', Float64, self.yaw_callback)
# 		rospy.Subscriber('/reset', Bool, self.reset_callback)

# 		self.yaw_publisher = rospy.Publisher('yaw_control/setpoint', Float64, queue_size=10)

# 	def reset_callback(self, msg):
# 		self.reset = msg.data

# 	def yaw_callback(self, msg):
# 		self.yaw = msg.data
# 		self.yaw_received = True

# 	def execute(self, userdata):
# 		if self.reset:
# 			self.reset_values()
# 			return 'reset'

# 		if not self.yaw_received:
# 			return 'notdone'

# 		self.ticks += 1
# 		if self.ticks >= self.duration:
# 			self.reset_values()
# 			return 'done'
		
# 		new_yaw = Float64()
# 		if self.is_clockwise:
# 			new_yaw.data = self.yaw + YAW_CHANGE
# 		else:
# 			new_yaw.data = self.yaw - YAW_CHANGE
# 		self.yaw_publisher.publish(new_yaw)
# 		return 'notdone'

# 	def reset_values(self):
# 		self.ticks = 0
# 		self.yaw = 0
# 		self.yaw_received = False
# 		self.reset = False

class RotateYawToRelativeTarget(smach.State):
	# This state causes the robot to rotate to a yaw (relative from its current rotation)
	# 
	# Arguments:
	#   yaw_change (float)
	def __init__(self, yaw_change):
		smach.State.__init__(self, outcomes=['done', 'notdone', 'reset'])

		self.yaw_change = yaw_change

		self.yaw = 0
		self.yaw_target = Float64()
		self.yaw_target.data = 0
		self.yaw_received = False
		self.yaw_published = False
		self.reset = False
		rospy.Subscriber('/yaw_control/state', Float64, self.yaw_callback)
		rospy.Subscriber('/reset', Float64, self.reset_callback)

		self.yaw_publisher = rospy.Publisher('/yaw_control/setpoint', Float64, queue_size=10)

	def reset_callback(self, msg):
		self.reset = msg.data

	def yaw_callback(self, msg):
		self.yaw = msg.data
		
		if not self.yaw_received:
			self.yaw_target.data = self.yaw + self.yaw_change
			self.yaw_received = True

	def execute(self, userdata):
		if self.reset:
			self.reset_values()
			return 'reset'

		if not self.yaw_received:
			return 'notdone'

		if not self.yaw_published:
			self.yaw_publisher.publish(self.yaw_target)
			self.yaw_published = True
			return 'notdone'

		if abs(self.yaw - self.yaw_target.data) < YAW_VARIANCE:
			self.reset_values()
			return 'done'
		else:
			return 'notdone'

	def reset_values(self):
		self.yaw = 0
		self.yaw_target.data = 0
		self.yaw_received = False
		self.yaw_published = False
		self.reset = False


class RotateYawToAbsoluteTarget(smach.State):
	# This state causes the robot to rotate towards a an absolute rotation (on the Earth)
	# 
	# Arguments:
	#   yaw_target (float)
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

		new_yaw = Float64()
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


class TrackObject(smach.State):
	# This state causes the robot to move towards an object
	# This state is subscribed to the object's X, Y, and area. And uses them to determine what action to take:
	#   increase/decrease forward thrust 			until area is within threshold
	#   increase/decrease depth 					until object's Y is within camera's center (height/2)
	#   change rotation about its y-axis (yaw) 		until object's X is within camera's center (width/2)
	# Note that forward thrust is not used until the X and Y are centered
	# 
	# Arguments:
	#   obj_topic (object)	contains strings for the topics to subscribe to. Must be in this format: {'x': string, 'y': string, 'area': string}
	#   xoffset, yoffset (int) the offsets from the center of the screen. Use this if you want to move towards 

	def __init__(self, obj_topic, xoffset=0, yoffset=0):
		smach.State.__init__(self, outcomes=['done', 'notdone', 'reset'])

		self.yoffset = yoffset
		self.timer = 0

		self.object_x = 0 # in pixels
		self.object_y = 0 # in pixels
		self.object_area = 0 # object width * height
		rospy.Subscriber(obj_topic['x'], Float64, self.object_x_callback)
		rospy.Subscriber(obj_topic['y'], Float64, self.object_y_callback)
		rospy.Subscriber(obj_topic['area'], Float64, self.object_area_callback)
		
		self.yaw_current = 0 # in degrees
		rospy.Subscriber('/yaw_control/state', Float64, self.yaw_callback) # current orientation
		self.yaw_publisher = rospy.Publisher('yaw_control/setpoint', Float64, queue_size=10) # desired orientation

		self.depth_current = 0 # in inches
		rospy.Subscriber('/depth_control/state', Int16, self.depth_callback)
		self.depth_publisher = rospy.Publisher('/depth_control/setpoint', Float64, queue_size=10)

		self.forward_thrust_publisher = rospy.Publisher('/yaw_pwm', Int16, queue_size=10)
		self.forward_thrust = 0

		self.has_reset = False
		self.reset_subscriber = rospy.Subscriber('/reset', Bool, self.reset_callback)

	def object_x_callback(self, msg):
		self.object_x = msg.data
	def object_y_callback(self, msg):
		self.object_y = msg.data
	def object_area_callback(self, msg):
		self.object_area = msg.data
	def yaw_callback(self, msg):
		self.yaw_current = msg.data
	def depth_callback(self, msg):
		self.depth_current = msg.data
	def reset_callback(self, msg):
		self.has_reset = msg.data

	def execute(self, userdata):
		self.timer = self.timer + 1
		if self.has_reset:
			self.resetValues()
			return 'reset'

		is_object_x_centered = self.adjust_yaw() 
		is_object_y_centered = self.adjust_depth()
		is_object_area_in_threshold = False

		if is_object_x_centered and is_object_y_centered:
			is_object_area_in_threshold = self.adjust_position() 

		# go to next state if the object is at the center of the camera frame and within certain distace of the submarine
		if is_object_x_centered and is_object_y_centered and is_object_area_in_threshold:
			self.resetValues()
			return 'done'
		else:
			return 'notdone'

	def resetValues(self):
		self.object_x = 0 # in pixels
		self.object_y = 0
		self.object_area = 0 # object width * height
		self.yaw_current = 0 # in degrees
		self.depth_current = 0 # in inches
		self.forward_thrust = 0
		self.has_reset = False
		self.timer = 0

	def adjust_yaw(self):
		# rotate yaw until x is within center +/- padding
		new_yaw = Float64() # 0 to 180 degrees (counterclockwise) or -180 degrees (clockwise)
		if self.object_x > CAMERA_WIDTH/2 + CENTER_PADDING_X:
			new_yaw.data = self.yaw_current - YAW_CHANGE
			self.yaw_publisher.publish(new_yaw)
			return False
		elif self.object_x < CAMERA_WIDTH/2 - CENTER_PADDING_X:
			new_yaw.data = self.yaw_current + YAW_CHANGE
			self.yaw_publisher.publish(new_yaw)
			return False
		else:
			return True

	def adjust_depth(self):
		# change depth until y is within center +/- padding
		new_depth = Float64() # 0 to 60 inches
		if self.object_y > CAMERA_HEIGHT/2 + self.yoffset + CENTER_PADDING_Y:
			new_depth.data = self.depth_current + DEPTH_CHANGE
			self.depth_publisher.publish(new_depth)
			return False
		elif self.object_y < CAMERA_HEIGHT/2 + self.yoffset - CENTER_PADDING_Y:
			new_depth.data = self.depth_current - DEPTH_CHANGE
			self.depth_publisher.publish(new_depth)
			return False
		else:
			return True

	def adjust_position(self):
		# move forward/backward until object area is within threshold
		if self.object_area/(CAMERA_WIDTH*CAMERA_HEIGHT) < AREA_THRESHOLD_LOW:
			self.change_forward_thrust(FORWARD_THRUST_CHANGE)
			return False
		elif self.object_area/(CAMERA_WIDTH*CAMERA_HEIGHT) > AREA_THRESHOLD_HIGH:
			self.change_forward_thrust(-FORWARD_THRUST_CHANGE)
			return False
		else:
			return True

	def change_forward_thrust(self, amount):
		# only increase/decrease thrust every 200 ticks
		if self.timer % 200 != 0:
			return

		# ensure thrust cannot exceed 280 or -280
		self.forward_thrust = self.forward_thrust + amount
		if self.forward_thrust > FORWARD_THRUST_MAX:
			self.forward_thrust = FORWARD_THRUST_MAX
		elif self.forward_thrust < -FORWARD_THRUST_MAX:
			self.forward_thrust = -FORWARD_THRUST_MAX

		# Publish the new forward thrust
		new_forward_thrust = Int16()
		new_forward_thrust.data = self.forward_thrust
		self.forward_thrust_publisher.publish(new_forward_thrust)
