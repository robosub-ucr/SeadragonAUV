
FORWARD_THRUST_MAX = 280
FORWARD_THRUST_CHANGE = 1

class MoveForwardState(smach.State):
	# This state causes the robot to set its forward thrust to zero
	# and slowly changes the forward thrust to be higher or lower (up to a maximum)
	# After some amount of ticks, the robot goes to the next state

	# Arguments:
	#   duration (int):     amount of ticks this state will last
	#   forward_step (int): how much thrust is increased/decreased per tick
	#   is_forward (bool):  determines if thrust increases or decreases
	def __init__(self, duration, is_forward=True):
		smach.State.__init__(self, outcomes=['done', 'notdone', 'reset'])

		self.timer = 0
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

		# When timer is up, go to the next state
		self.timer += 1
		if self.timer >= self.duration:
			self.reset_values()
			return 'done'

		# Else, increase or decrease forward thrust every 200 ticks
		elif self.timer % 200 != 0:
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
		self.timer = 0
		self.forward_thrust.data = 0
		self.reset = False

