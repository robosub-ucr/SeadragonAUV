import sys
import math
from enum import Enum

from PyQt5.QtCore import Qt
from PyQt5.QtGui import QColor, QFont, QPainter, QBrush, QPen, QIcon
from PyQt5.QtWidgets import QApplication, QMainWindow, QAction, qApp

import rospy
from std_msgs.msg import Int16, Float64, Bool

class Topic(Enum):
	YAW_PWM = '/yaw_pwm'
	YAW_PWM_FEEDBACK = '/yaw_pwm_feedback'
	YAW_STATE = '/yaw_control/state'
	YAW_SETPOINT = '/yaw_control/setpoint'
	YAW_PID = '/yaw_control/pid_enable'

class Window(QMainWindow):
	def __init__(self):
		#        super().__init__(self) # Python3
		super(Window, self).__init__() # Python2

		
		self.title = "PyQt5 Drawing Tutorial"
		self.top = 150
		self.left = 150
		self.width = 640
		self.height = 480

		self.cx = self.width * 0.5
		self.cy = self.height * 0.5
		self.radius = self.height * 0.4

		self.yaw_state_str = "---"

		self.yaw_state = None

		# Subscribers
		rospy.Subscriber('/yaw_control/state', Float64, self.yaw_state_callback)

		# Publishers
		self.topics = {
			Topic.YAW_SETPOINT: {'publisher':rospy.Publisher('/yaw_control/setpoint', Float64, queue_size=10), 'msg':Float64()},
		}

		self.mouse_left_x = None
		self.mouse_left_y = None
		self.angle = None

		

		self.InitWindow()

	def mousePressEvent(self, QMouseEvent):
		if QMouseEvent.button() == Qt.LeftButton:
			self.mouse_left_x = QMouseEvent.x()
			self.mouse_left_y = QMouseEvent.y()
			self.angle = math.atan2(self.cy - self.mouse_left_y, self.cx - self.mouse_left_x) - math.pi/2
			if self.angle > 3.14:
				self.angle -= 3.14 * 2
			elif self.angle < -3.14:
				self.angle += 3.14 * 2
			self.update()
			self.publish(Topic.YAW_SETPOINT, -1 * self.angle)
			print("left mouse pressed at ({}, {}) with angle {}".format(self.mouse_left_x,self.mouse_left_y, self.angle))
		

	def yaw_state_callback(self, msg):
		self.yaw_state = msg.data
		self.update()

	def InitWindow(self):
		self.setWindowTitle(self.title)
		self.setGeometry(self.top, self.left, self.width, self.height)
		self.show()

	def publish(self, topic, value):
		msg = self.topics[topic]['msg']
		msg.data = value
		self.topics[topic]['publisher'].publish(msg)

	def paintEvent(self, event):
		
		

		# draw circle
		painter = QPainter(self)
		painter.setPen(QPen(Qt.green, 8, Qt.DashLine))
		painter.drawEllipse(self.cx-self.radius, self.cy-self.radius, self.radius*2, self.radius*2)
		painter.end()

		if self.yaw_state is not None:
		    self.drawCompassLine(self.cx, self.cy, self.radius, self.yaw_state)

	def drawCompassLine(self, cx, cy, radius, angle):
		painter = QPainter(self)

		painter.setFont(QFont("Helvetica", 12))

		text = str(angle) if angle is not None else "None!"
		#painter.drawText(cx, cy, text)

		#if value is not None:
		x = self.cx + math.cos(-angle - math.pi/2)*radius
		y = self.cy + math.sin(-angle - math.pi/2)*radius
		line = QPainter(self)
		line.setPen(Qt.red)
		line.drawLine(cx,cy, x, y)

		if self.mouse_left_x is not None:
			rx = self.cx + math.cos(self.angle - math.pi/2) * self.radius
			ry = self.cy + math.sin(self.angle - math.pi/2) * self.radius
			line.drawLine(self.cx, self.cy, rx, ry)
		else:
			print("self.x is None")
		#line.end()

		painter.setPen(QColor("Black"))
		painter.drawText(x, y, text)

def main():
	rospy.init_node('sensor_gui')

	App = QApplication(sys.argv)
	window = Window()

	sys.exit(App.exec_())

if __name__ == '__main__':
	main()
