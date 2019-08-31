import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32, Float64, Bool, Int16

buttonA = 0 
forwardPublisher = 0

DEGREE_45 = 0.785398
DEGREE_90 = 1.5708

class JoyNode:
    "Xbox controller for AUV"

    def __init__(self):
        "JoyNode Constructor"
        rospy.Subscriber('joy', Joy, self.joyCallBack)


        self.yaw_state = None
        self.depth_state = None
        rospy.Subscriber('/yaw_control/state', Float64, self.yaw_state_callback)
        rospy.Subscriber('/depth_control/state', Int16, self.depth_state_callback)

        self.forwardPublisher = rospy.Publisher('/yaw_pwm', Int16, queue_size= 10 )
        self.yawSetpointPublisher = rospy.Publisher('/yaw_control/setpoint', Float64, queue_size= 10 )
        self.depthSetpointPublisher = rospy.Publisher('/depth_control/setpoint', Int16, queue_size= 10 )


        self.buttons =  [0 for i in range(12)]
        self.axes = [0 for i in range(6)]

    def yaw_state_callback(self, msg):
        self.yaw_state = msg.data

    def depth_state_callback(self, msg):
        self.depth_state = msg.data

    def fix_yaw(self, yaw):
        if yaw >= 3.14:
            yaw -= 2 * 3.14
        elif yaw <= -3.14:
            yaw += 2 * 3.14
        return yaw

    def joyCallBack(self, joy):
        "invoked every time a joystick message arrives"
        global DEGREE_45, DEGREE_90
        print(joy)
        print(len(joy.buttons))
        print(len(joy.axes))
        
        
        if joy.buttons[0]:
            # depth go down A
            if self.depth_state != None:
                new_depth = self.depth_state + 1
                depthObj = Int16()
                depthObj.data = new_depth
                self.depthSetpointPublisher.publish(depthObj)
        else:
            if self.depth_state != None:
                new_depth = self.depth_state - 1
                if new_depth < 0:
                    new_depth = 0
                depthObj = Int16()
                depthObj.data = new_depth
                self.depthSetpointPublisher.publish(depthObj)

        if joy.buttons[1] == 1: # B
            # Not mapped B
            # Rotate clockwise
            if self.yaw_state != None:
                new_yaw = self.yaw_state + DEGREE_45
                new_yaw = self.fix_yaw(new_yaw)
                yawObj = Float64()
                yawObj.data = new_yaw
                self.yawSetpointPublisher.publish(yawObj)
            pass
        else:
            if self.yaw_state != None:
                yawObj = Float64()
                yawObj.data = self.yaw_state
                self.yawSetpointPublisher.publish(yawObj)

        if  joy.buttons[2]:
            # Not mapped X
            # Rotate counter-clockwise
            if self.yaw_state != None:
                new_yaw = self.yaw_state - DEGREE_45
                new_yaw = self.fix_yaw(new_yaw)
                yawObj = Float64()
                yawObj.data = new_yaw
                self.yawSetpointPublisher.publish(yawObj)
            pass
        else:
            if self.yaw_state != None:
                yawObj = Float64()
                yawObj.data = self.yaw_state
                self.yawSetpointPublisher.publish(yawObj)

        if  joy.buttons[3]:
            # depth up Y
            pass

        if joy.buttons[4]:
            # drop weight LB
            pass

        if  joy.buttons[5]:
            # torpedo launcher RB
            pass

        if  joy.buttons[6]:
            # nothing mapped Back
            pass

        if  joy.buttons[7]:
            # nothing mapped Start
            pass

        if  joy.buttons[8]:
            # nothing mapped Power
            pass

        if  joy.buttons[9]:
            # nothing mapped Button Stick Left
            pass

        if joy.buttons[10]:
            # nothing mapped Button Stick Right
            pass


        if joy.axes[0]:
            # nothing mapped L/R Axis Stick Left
            pass

        if  joy.axes[1]:
            # camera rotation U/D Axis Stick Left
            pass

        if  joy.axes[2]:
            # nothing mapped LT
            pass

        if  joy.axes[3]:
            # rotation about y-axis L/R Axis Stick Right
            pass

        if joy.axes[4]:
            # rotation about y-axis U/D Axis Stick Right
            pass

        if joy.axes[5]:
            # nothing mapped RT
            pass

        if joy.axes[6]:
            # nothing mapped Cross Key L/R
            pass

        # Joystick Input: Cross Key Up/Down
        # sub moving fwd/bckwrd 
        forwardInt16 = Int16()
        if joy.axes[7] >= 0.5:
            forwardInt16.data = 100
        else if joy.axes[7] <= -0.5:
            forwardInt16.data = -100
        else:
            forwardInt16.data = 0
        self.forwardPublisher.publish(forwardInt16)




def main():
    rospy.init_node('joystickController');
    joyObject = JoyNode()
    print("Python Project Running....")
 #   rospy.Subscriber('joy', Joy, buttonACallback)
    rospy.Subscriber('joy', Joy, joyObject.joyCallBack)

    #forwardPublisher = rospy.Publisher('/yaw_pwm', Int16, queue_size= 10 )    
    forwardObj = Int16()

    while not rospy.is_shutdown():
        
        if buttonA == 0:
            forwardObj.data = 0
        else:
            forwardObj.data = -10
        yaw_pwm_publisher = rospy.Publisher('/yaw_pwm', Int16, queue_size=10)
        publisher.publish(forwardObj)
        pass   
if __name__ == '__main__':
    main()
