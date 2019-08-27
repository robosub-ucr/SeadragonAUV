from PyQt5.QtGui import QColor, QFont, QPainter, QBrush, QPen
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtCore import Qt
import sys
import math

import rospy
from std_msgs.msg import Int16, Float64, Bool

App = None
window = None
yaw_state = None

class Window(QMainWindow):
    def __init__(self):
#        super().__init__(self) # Python3
        super(Window, self).__init__() # Python2
        self.title = "PyQt5 Drawing Tutorial"
        self.top = 150
        self.left = 150
        self.width = 640
        self.height = 480
        
        self.yaw_state_str = "---"
        
        self.yaw_pub = rospy.Publisher('/yaw_control/setpoint', Float64, queue_size=10)
        
        self.InitWindow()

    def InitWindow(self):
        self.setWindowTitle(self.title)
        self.setGeometry(self.top, self.left, self.width, self.height)
        self.show()

    def paintEvent(self, event):
        cx = self.width * 0.5
        cy = self.height * 0.5
        radius = self.height * 0.4
        
        # draw circle
        painter = QPainter(self)
        painter.setPen(QPen(Qt.green, 8, Qt.DashLine))
        painter.drawEllipse(cx-radius, cy-radius, radius*2, radius*2)
        painter.end()

        # get text
        global yaw_state
        if yaw_state is not None:
            self.yaw_state_str = str(yaw_state)
        else:
            self.yaw_state_str = "None"
        
        # draw center text
        textPainter = QPainter(self)
        textPainter.setPen(QColor("Black"))
        textPainter.setFont(QFont('Helvetica', 48))
        textPainter.drawText(cx, cy, self.yaw_state_str)
        
        # draw line for yaw_state
        line = QPainter(self)
        line.setPen(Qt.red)
        
        if yaw_state is not None:
            x = cx + math.cos(-yaw_state - math.pi/2)*radius
            y = cy + math.sin(-yaw_state - math.pi/2)*radius
            line.drawLine(cx,cy, x, y)
        

def yaw_state_callback(msg):
    global window, yaw_state
    yaw_state = msg.data
    window.update()

def main():
    rospy.init_node('sensor_gui')
    #rospy.Subscriber('/yaw_control/state', Float64, yaw_state_callback)
    
    global App, window
    App = QApplication(sys.argv)
    window = Window()
    
    rospy.Subscriber('/yaw_control/state', Float64, yaw_state_callback)
    
    sys.exit(App.exec_())

if __name__ == '__main__':
    main()
