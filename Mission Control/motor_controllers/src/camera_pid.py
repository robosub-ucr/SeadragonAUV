## --- camera pid class --- ##

class cam_pid():
	def __init__(self):
		self.TARGET_SETPOINT = (640/2,480/2)	# Center of image frame is the target ------ The target setpoint can also be passed in as a parameter to the constructor 
		self.DEPTHPWM_MAX    = 150
		self.YAWPWM_MAX	     = 75
		self.prevXerror	     = 0
		self.prevYerror	     = 0
		
	def update(self,centroid):
		# Calculate error from center of frame
                error_x = self.TARGET_SETPOINT[0] - centroid[0]
                error_y = self.TARGET_SETPOINT[1] - centroid[1]

		# Update Proportional term for x and y axis
                pX      =  error_x * .02
                pY      = -error_y * .6

		# Update Derivative term for x and y axis
                dX	=  (error_x - self.prevXerror) * .4	
		dY	=  (error_y - self.prevYerror) * .1
		
		# Output control to thrusters
		pwmOutyaw   =  pX + dX
		pwmOutdepth =  pY + dY

		# Check if pwm for yaw is past its max limit
		if pwmOutyaw > self.YAWPWM_MAX:
			pwmOutyaw = self.YAWPWM_MAX
		elif pwmOutyaw < -self.YAWPWM_MAX:
			pwmOutyaw = -self.YAWPWM_MAX

		
		# Check if pwm for depth is past its max limit
		if pwmOutdepth > self.DEPTHPWM_MAX:
			pwmOutdepth = self.DEPTHPWM_MAX
		elif pwmOutdepth < -self.DEPTHPWM_MAX:
			pwmOutdepth = -self.DEPTHPWM_MAX			

		print ('Yaw Pwm: {}, Depth Pwm: {}' ).format( pwmOutyaw, pwmOutdepth)

		# FIXME Return yaw pwm and depth pwm
