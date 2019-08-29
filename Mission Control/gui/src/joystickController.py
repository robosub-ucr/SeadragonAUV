import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16, Float64, Bool

buttonZero = 0 
forwardPublisher = 0

def button_zero_callback(msg): 
    global buttonZero, forwardPublisher
    buttonZero = msg.buttons[0]
    print(buttonZero)
    


def main():
    rospy.init_node('joystickController')

    rospy.Subscriber('joy', Joy, button_zero_callback)

    forwardPublisher = rospy.Publisher('/yaw_pwm', Int16, queue_size= 10 )    
    forwardObj = Int16()

    while not rospy.is_shutdown():
        
        if buttonZero == 0:
            forwardObj.data = 0
        else:
            forwardObj.data = -10
        publisher = rospy.Publisher('/yaw_pwm', Int16, queue_size=10)
        publisher.publish(forwardObj)
        pass   
if __name__ == '__main__':
    main()
