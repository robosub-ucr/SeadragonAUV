import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32, Float64, Bool, Int16

buttonA = 0 
forwardPublisher = 0

class JoyNode:
    "Xbox controller for AUV"

    def __init__(self):
        "JoyNode Constructor"
        rospy.Subscriber('joy', Joy, self.joyCallBack)
        self.forwardPublisher = rospy.Publisher('/yaw_pwm', Int16, queue_size= 10 )
        self.buttons =  [0 for i in range(12)]
        self.axes = [0 for i in range(6)]

    def joyCallBack(self, joy):
        "invoked every time a joystick message arrives"
        print(joy)
        print(len(joy.buttons))
        print(len(joy.axes))
        
        
        if joy.buttons[0]:
            # depth go down A
            pass

        if joy.buttons[1]:
            # Not mapped B
            pass

        if  joy.buttons[2]:
            # nothing mapped X
            pass

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

        if joy.axes[7]:
            # sub moving fwd/bckwrd Cross Key U/D
            pass




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
        publisher = rospy.Publisher('/yaw_pwm', Int16, queue_size=10)
        publisher.publish(forwardObj)
        pass   
if __name__ == '__main__':
    main()
