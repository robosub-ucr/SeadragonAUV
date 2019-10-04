import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32, Float64, Bool, Int16
import math
import numpy as np

import time
import sys
import os
import stat

class JoyLED:
# LED States are as follows:

# 	# 0: All LEDs off
# 	# 1: Flash all LEDs 3 times, then go back to the previous display.
# 	# 2: Flash the top-left LED 3 times, then leave it on.
# 	# 3: Flash the top-right LED 3 times, then leave it on.
# 	# 4: Flash the bottom-left LED 3 times, then leave it on.
# 	# 5: Flash the bottom-right LED 3 times, then leave it on.
# 	# 6: Turn on the top-left LED.
# 	# 7: Turn on the top-right LED.
# 	# 8: Turn on the bottom-left LED.
# 	# 9: Turn on the bottom-right LED.
# 	# 10: Turn on each LED, one after another, clockwise.
# 	# 11: Flash the currently-active LED (on for about half a second, off for about half a second) 15 times, then leave it on.
# 	# 12: Flash the currently-active LED (on for about half a second, off for about 3.5 seconds), forever.
# 	# 13: Flash a chequer-board pattern (top-left and bottom-right, followed by top-right and bottom left; on for about half a second, off for about half a second) for about 10 seconds, then go back to the previous display.
	FILEPATH =  "/sys/class/leds/xpad0/brightness"

	def __init__(self):
		self.currentTime = time.time()
		self.lastTime = self.currentTime

		self.timeInterval = 0
		self.ledState = 0

		self.depth = None

		#os.chmod(self.FILEPATH, 666)

		rospy.Subscriber('/joy_led/state', Int32, self.led_callback)
		rospy.Subscriber('/depth', Int32, self.depth_callback)

	def depth_callback(self, msg):
		print("depth_callback(): ", msg.data)
		self.depth = msg.data
		print("self.depth = ", self.depth, type(self.depth))

	def led_callback(self, msg):
		print("led_callback()")
		self.ledState = msg.data
		self.time_update()

	def time_update(self):
		self.currentTime = time.time()
		self.timeInterval = self.currentTime - self.lastTime
		self.lastTime = self.currentTime

	def execute(self):
		#print(type(self.depth))
		if not self.depth is None:
			print("if self.depth")
			if self.depth == 0:

				self.checker_pattern()
			else:
				print("else")
				pass
		self.display()

	def checker_pattern(self):
		print("checker_pattern")
		self.ledState = 13
		pass

	def display(self):
		delay = 0.010
		with open(self.FILEPATH, "w") as f:
			print("writing ", self.ledState)
			f.write(str(self.ledState))
		time.sleep(delay)
		# with open(self.FILEPATH, "w") as f:
		# 	f.write(str(self.ledState + 1))
		# time.sleep(delay)
		# with open(self.FILEPATH, "w") as f:
		# 	f.write(str(self.ledState + 2))
		# time.sleep(delay)
		# with open(self.FILEPATH, "w") as f:
		# 	f.write(str(self.ledState + 3))
		# time.sleep(delay)
		# as (f, err):
		# 	if err:
		# 		print("IOError: (custom): ", err)
		# 	else:
		# 		f.write(str(self.ledState))

		#try:
			# f = open(self.FILEPATH, "w")
			# f.write(str(self.ledState))
			# f.close()

			# f = open(self.FILEPATH, "w")
			# f.write(str(self.ledState + 1))
			# f.close()

		# except: #OSError as e:
		# 	print("led.py did not find the joystick file")
		# 	sys.exit()

	def led_pattern_rotate_left(self):
		pass

def main():
	rospy.init_node('JoyLED') # create a ROS node
	rate = rospy.Rate(20) # set to 20 Hz (this program loops 20 times per second)
	joyLED = JoyLED()

	print("Python Project 'JoyLED' Running...")
	while not rospy.is_shutdown(): # main loop
		joyLED.execute()
		rate.sleep()
	
if __name__ == '__main__':
	main()