
FORWARD_THRUST_MAX = 280

class MoveForwardState(smach.State):
	def __init__(self, duration, forward_step, is_forward):
		smach.State.__init__(self, outcomes=['done', 'notdone', 'reset'])

		self.timer = 0
		self.duration = duration
		self.forward_step = forward_step
		self.is_forward = is_forward

		self.forward_thrust_publisher = rospy.Publisher('/yaw_pwn', Int16, queue_size=10)
		self.forward_thrust = 0

		self.reset = False
		rospy.Subscriber('/reset', Bool, self.reset_callback)

	def reset_callback(self, msg):
		self.reset = msg.data

	def execute(self, userdata):
		if self.reset:
			self.reset_values()
			return 'reset'

		self.timer += 1
		if self.timer >= self.duration:
			self.reset_values()
			return 'done'
		else:
			if self.is_forward:
				self.change_forward_thrust(self.forward_step)
			else:
				self.change_forward_thrust(-self.forward_step)
			return 'notdone'

	def change_forward_thrust(self, amount):
		if self.timer % 200 != 0:
			return

		self.forward_thrust = self.forward_thrust + amount
		if self.forward_thrust > FORWARD_THRUST_MAX:
			self.forward_thrust = FORWARD_THRUST_MAX
		elif self.forward_thrust < -1 * FORWARD_THRUST_MAX:
			self.forward_thrust = -1 * FORWARD_THRUST_MAX

	def reset_values(self):
		self.timer = 0
		self.forward_thrust = 0
		self.reset = False

