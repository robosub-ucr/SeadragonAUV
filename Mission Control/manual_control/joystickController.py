import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32, Float64, Bool, Int16

buttonA = 0 
forwardPublisher = 0

DEGREE_45 = 0.785398
DEGREE_90 = 1.5708
DEGREE_1 =  0.0174

class JoyNode:
    "Xbox controller for AUV"

    def __init__(self):
        "JoyNode Constructor"
        rospy.Subscriber('joy', Joy, self.joyCallBack)

        self.yaw_state = None
        self.depth_state = None
        self.yaw_setpoint = None
        self.depth_setpoint = None
        rospy.Subscriber('/yaw_control/state', Float64, self.yaw_state_callback)
        rospy.Subscriber('/depth_control/state', Int16, self.depth_state_callback)
        rospy.Subscriber('/yaw_control/setpoint', Float64, self.yaw_setpoint_callback)
        rospy.Subscriber('/depth_control/setpoint', Int16, self.depth_setpoint_callback)

        self.depthPublisher = rospy.Publisher('/depth_pwm', Int16, queue_size=10)
        self.forwardPublisher = rospy.Publisher('/yaw_pwm', Int16, queue_size=10)
        self.yawSetpointPublisher = rospy.Publisher('/yaw_control/setpoint', Float64, queue_size=10)
        self.depthSetpointPublisher = rospy.Publisher('/depth_control/setpoint', Int16, queue_size=10)
        
        self.depthPidPublisher = rospy.Publisher('/depth_control/pid_enable', Bool, queue_size=10)
        self.yawPidPublisher = rospy.Publisher('yaw_control/pid_enable', Bool, queue_size=10)

        # List of buttons and axes for the Microsoft XBox Wired Controller
        # http://wiki.ros.org/joy
        self.buttons =  [False for i in range(11)] 
        self.axes = [0 for i in range(8)]

    def yaw_state_callback(self, msg):
        self.yaw_state = msg.data

    def depth_state_callback(self, msg):
        self.depth_state = msg.data

    def yaw_setpoint_callback(self, msg):
        self.yaw_setpoint = msg.data

    def depth_setpoint_callback(self, msg):
        self.depth_setpoint = msg.data

    def fix_yaw(self, yaw):
        if yaw >= 3.14:
            yaw -= 2 * 3.14
        elif yaw <= -3.14:
            yaw += 2 * 3.14
        return yaw

    def execute(self):
        buttonIncreaseDepth = self.buttons[0] # Button A
        buttonDecreaseDepth = self.buttons[3] # Button Y
        buttonRotateCounterClockwise = self.buttons[2] # Button X
        buttonRotateClockwise = self.buttons[1] # Button B

        axisStrafe = self.axes[6] # Axis Cross Key Left/Right
        axisForward = self.axes[7] # Axis Cross Key Up/Down

        if buttonIncreaseDepth: # Button A -- Increase depth setpoint
            print("Button A pressed")
            if self.depth_state != None:
                new_depth = self.depth_state + 1
                depthObj = Int16()
                depthObj.data = new_depth
                print("Button A -- new depth: {}".format(new_depth))
                self.depthSetpointPublisher.publish(depthObj)
            else:
                print("Button A -- depth state was not published")

        if buttonRotateClockwise: # Button B -- Rotate clockwise
            print("Button B pressed")
            # Increase setpoint clockwise
            if self.yaw_setpoint != None:
                new_yaw = self.yaw_setpoint + DEGREE_1 / 50.0
                new_yaw = self.fix_yaw(new_yaw)
                yawObj = Float64()
                yawObj.data = new_yaw
                self.yawSetpointPublisher.publish(yawObj)
            else:
                print("Button B -- yaw setpoint was not published. Press the START button")
        # else:
        #     # Stop rotating by setting the setpoint to current rotation
        #     if self.yaw_state != None:
        #         yawObj = Float64()
        #         yawObj.data = self.yaw_state
        #         self.yawSetpointPublisher.publish(yawObj)


        if buttonRotateCounterClockwise: # Button X -- Rotate counter-clockwise
            print("Button X pressed")
            # Increase setpoint counter-clockwise
            if self.yaw_setpoint != None:
                new_yaw = self.yaw_setpoint - DEGREE_1 / 50.0
                new_yaw = self.fix_yaw(new_yaw)
                yawObj = Float64()
                yawObj.data = new_yaw
                print("Button X -- {}".format(new_yaw))
                self.yawSetpointPublisher.publish(yawObj)
            else:
                print("Button X -- yaw setpoint was not published. Press the START button")

        # else:
        #     # Stop rotating by setting the setpoint to current rotation
        #     if self.yaw_state != None:
        #         yawObj = Float64()
        #         yawObj.data = self.yaw_state
        #         self.yawSetpointPublisher.publish(yawObj)


        if buttonDecreaseDepth: # Button Y -- Decrease depth setpoint
            print("Button Y pressed")
            if self.depth_state != None:
                new_depth = self.depth_state - 1
                depthObj = Int16()
                depthObj.data = new_depth
                print("Button Y -- new depth: {}".format(new_depth))
                self.depthSetpointPublisher.publish(depthObj)
            else:
                print("Button Y -- depth state was not published")


        if self.buttons[4]:
            # drop weight LB
            pass

        if self.buttons[5]:
            # torpedo launcher RB
            pass

        if self.buttons[6]: # Button BACK -- Disable PIDs
            print("Button BACK -- PIDs are disabled")
            off = Bool()
            off.data = False
            zeroInt = Int16()
            zeroInt.data = 0
            self.depthPidPublisher.publish(off)
            self.yawPidPublisher.publish(off)
            self.forwardPublisher.publish(zeroInt)
            self.depthPublisher.publish(zeroInt)

        if self.buttons[7]: # Button START -- Enable PIDs
            print("Button START -- PIDs are enabled")
            on = Bool()
            on.data = True
            self.depthPidPublisher.publish(on)
            self.yawPidPublisher.publish(on)
            if self.yaw_state != None:
                yaw = Float64()
                yaw.data = self.yaw_state
                self.yawSetpointPublisher.publish(yaw)
            else:
                print("Button START -- /yaw_control/state has not been published. Check that AHRS is running?")

        if self.buttons[8]:
            # nothing mapped Power
            pass

        if self.buttons[9]:
            # nothing mapped Button Stick Left
            pass

        if self.buttons[10]:
            # nothing mapped Button Stick Right
            pass

        # AXES

        if self.axes[0]:
            # nothing mapped L/R Axis Stick Left
            pass

        if self.axes[1]:
            # camera rotation U/D Axis Stick Left
            pass

        if self.axes[2]:
            # nothing mapped LT
            pass

        if self.axes[3]:
            # rotation about y-axis L/R Axis Stick Right
            pass

        if self.axes[4]:
            # rotation about y-axis U/D Axis Stick Right
            pass

        if self.axes[5]:
            # nothing mapped RT
            pass

        if self.axes[6]:
            # nothing mapped Cross Key L/R
            pass

    def joyCallBack(self, joy):
        "invoked every time a joystick message arrives"
        global DEGREE_1, DEGREE_45, DEGREE_90
        
        for i in range(len(self.buttons)):
            self.buttons[i] = joy.buttons[i]
        for i in range(len(self.axes)):
            self.axes[i] = joy.axes[i]

        # 

        # Joystick Input: Cross Key Up/Down
        # sub moving fwd/bckwrd 

        # forwardInt16 = Int16()
        # if joy.axes[7] >= 0.5:
        #     forwardInt16.data = 100
        # elif joy.axes[7] <= -0.5:
        #     forwardInt16.data = -100
        # else:
        #     forwardInt16.data = 0
        # self.forwardPublisher.publish(forwardInt16)

import atexit

def kill_motors():
    print("killed motors")

    off = Bool()
    off.data = False
    zeroInt = Int16()
    zeroInt.data = 0
    zeroFloat = Float64()
    zeroFloat.data = 0.0
    yaw_pub = rospy.Publisher('/yaw_pwm', Float64, queue_size=10)
    depth_pub = rospy.Publisher('/depth_pwm', Int16, queue_size=10)
    yaw_pid_pub = rospy.Publisher('yaw_control/pid', Bool, queue_size=10)
    depth_pid_pub = rospy.Publisher('/depth_control/pid', Bool, queue_size=10)
    yaw_pub.publish(zeroFloat)
    depth_pub.publish(zeroInt)
    yaw_pid_pub.publish(off)
    depth_pid_pub.publish(off)

atexit.register(kill_motors)


def main():
    rospy.init_node('joystickController');
    joyObject = JoyNode()

    print("Python Project Running....")
    while not rospy.is_shutdown():
       joyObject.execute()

if __name__ == '__main__':
    main()
