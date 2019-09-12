import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32, Float64, Bool, Int16
import math
import numpy as np

class JoyInput(IntEnum):
    A = 0
    B = 1
    X = 2
    Y = 3
    LB = 4
    RB = 5
    BACK = 6
    START = 7
    POWER = 8 # XBox button
    LS = 9    # left stick button
    RS = 10   # right stick button

    LS_X = 11    # left stick left/right
    LS_Y = 12    # left stick up/down
    LT = 13      # left trigger
    RS_X = 14    # right stick left/right
    RS_Y = 15    # right stick up/down
    RT = 16      # right trigger
    CROSS_X = 17 # cross key left/right
    CROSS_Y = 18 # cross key up/down

class Topic:
    YAW_PWM = '/yaw_pwm'
    YAW_PWM_FEEDBACK = '/yaw_pwm_feedback'
    YAW_PID = '/yaw_control/pid_enable'
    YAW_STATE = '/yaw_control/state'
    YAW_SETPOINT = '/yaw_control/setpoint'

    DEPTH_PWM = '/depth_pwm'
    DEPTH_PID = '/depth_control/pid_enable'
    DEPTH_STATE = '/depth_control/state'
    DEPTH_SETPOINT = '/depth_control/setpoint'

class Joystick:
    """This class contains all information related to the Microsoft XBox 360 Wired controller. 
    It subscribes to the /joy topic. And publishes to topics related to controlling the behavior of the AUV, such as /yaw_pwm and /depth_control/setpoint. 
    In essence, it takes joystic inputs and converts them into messages for other nodes.
    """

    MAX_DEPTH_PWM = 130
    MAX_YAW_PWM = 150
    MAX_YAW_PWM_FEEDBACK = 75
    MAX_DEPTH = 7 * 12 # 7 feet
    RS_ANGLE = 'right_stick_angle'

    def __init__(self):
        """Initializes the inputs and inputs as dictionaries, 
        and sets up Subscribers and Publishers for various topics.
        Learn more: http://wiki.ros.org/joy
        """

        # The Microsoft XBox 360 Wired controller has 11 buttons and 8 axes.
        # Buttons can be 0 (not pressed) or 1 (pressed)
        # Axes are floats and range between -1 and 1. Note that for LT and RT, their "not pressed" value is 1 and for the others it is 0. Cross keys only have values -1, 0, and 1. The others have be any value in between -1 and 1.
        num_buttons = 11
        num_axes = 8
        self.inputs = [0 for i in range(num_buttons + num_axes)]
        self.inputs[JoyInput.LT] = self.inputs[JoyInput.RT] = 1

        # Dictionary of saved inputs. If an input is not currently saved, you must set it to None.
        # For example, the LS_Y ("left stick Y") axis may be saved in self.saved[JoyInput.LS_Y]
        self.saved = {
            JoyInput.LS_Y: None,
            Joystick.RS_ANGLE: None,
        }

        # Field variables
        self.depth_state = None # stores the depth state
        self.depth_last_received = 0 # how long since the last depth state callback
        self.depth_pwm_input = 0 # tracks pwm given to depth thrusters

        # ROS Subscribers
        rospy.Subscriber(Topic.YAW_STATE, Float64, self.yaw_state_callback)
        rospy.Subscriber(Topic.DEPTH_STATE, Float64, self.depth_state_callback)
        rospy.Subscriber(Topic.YAW_SETPOINT, Float64, self.yaw_setpoint_callback)
        rospy.Subscriber(Topic.DEPTH_SETPOINT, Float64, self.depth_setpoint_callback)

        # ROS Publishers
        # self.topics is a dictionary of dictionaries.
        # 'publisher' contains the rospy.Publisher()
        # 'msg' contains the Int16(), Float64(), or Bool() related to the publisher
        # Use self.publish() rather than using self.topics directly.
        self.topics = {
            Topic.YAW_PWM: {'publisher':rospy.Publisher(Topic.YAW_PWM, Int16, queue_size=10), 'msg':Int16()},
            Topic.YAW_PWM_FEEDBACK: {'publisher':rospy.Publisher(Topic.YAW_PWM_FEEDBACK, Int16, queue_size=10), 'msg':Int16()},
            Topic.YAW_PID: {'publisher':rospy.Publisher(Topic.YAW_PID, Bool, queue_size=10), 'msg':Bool()},
            Topic.YAW_SETPOINT: {'publisher':rospy.Publisher(Topic.YAW_SETPOINT, Float64, queue_size=10), 'msg':Float64()},

            Topic.DEPTH_PWM: {'publisher':rospy.Publisher(Topic.DEPTH_PWM, Int16, queue_size=10), 'msg':Int16()},
            Topic.DEPTH_PID: {'publisher':rospy.Publisher(Topic.DEPTH_PID, Bool, queue_size=10), 'msg':Bool()},
            Topic.DEPTH_SETPOINT: {'publisher':rospy.Publisher(Topic.DEPTH_SETPOINT, Int16, queue_size=10), 'msg':Int16()},
        }

    def yaw_state_callback(self, msg):
        self.yaw_state = msg.data

    def depth_state_callback(self, msg):
        self.depth_state = msg.data

    def yaw_setpoint_callback(self, msg):
        self.yaw_setpoint = msg.data

    def depth_setpoint_callback(self, msg):
        self.depth_setpoint = msg.data
  
    def joy_callback(self, joy):
        """This function runs whenver the joystick node publishes data. It stores the values from the topic /joy. These values are mainly used in execute()."""
        self.inputs[JoyInput.A] = joy.buttons[0]
        self.inputs[JoyInput.B] = joy.buttons[1]
        self.inputs[JoyInput.X] = joy.buttons[2]
        self.inputs[JoyInput.Y] = joy.buttons[3]
        self.inputs[JoyInput.LB] = joy.buttons[4]
        self.inputs[JoyInput.RB] = joy.buttons[5]
        self.inputs[JoyInput.BACK] = joy.buttons[6]
        self.inputs[JoyInput.START] = joy.buttons[7]
        self.inputs[JoyInput.POWER] = joy.buttons[8]
        self.inputs[JoyInput.LS] = joy.buttons[9]
        self.inputs[JoyInput.RS] = joy.buttons[10]

        self.inputs[JoyInput.LS_X] = joy.axes[0]
        self.inputs[JoyInput.LS_Y] = joy.axes[1]
        self.inputs[JoyInput.LT] = joy.axes[2]
        self.inputs[JoyInput.RS_X] = joy.axes[3]
        self.inputs[JoyInput.RS_Y] = joy.axes[4]
        self.inputs[JoyInput.RT] = joy.axes[5]
        self.inputs[JoyInput.CROSS_X] = joy.axes[6]
        self.inputs[JoyInput.CROSS_Y] = joy.axes[7]

    def execute(self):
        """Called from main()'s infinite while loop. Calls other functions that check for joystick inputs to determine how the AUV will behave.
        """
        # Check buttons
        self.check_enable_pids(self.inputs[JoyInput.START])
        self.check_disable_pids(self.inputs[JoyInput.BACK])
        self.check_toggle(self.inputs[JoyInput.LS], self.inputs[JoyInput.LS_Y], self.saved[JoyInput.LS_Y])

        # Check axes
        self.input_forward(JoyInput.LS_Y)
        self.input_depth(JoyInput.CROSS_Y)
        self.input_rotate(JoyInput.LT, JoyInput.RT)

        # yaw setpoint
        magnitude, angle = self.input_yaw_setpoint(JoyInput.RS_X, JoyInput.RS_Y, Joystick.RS_ANGLE)
        self.check_yaw_setpoint_toggle(JoyInput.RS, Joystick.RS_ANGLE, magnitude, angle)

    def check_disable_pids(self, input_value):
        if input_value:
            self.publish(Topic.DEPTH_PWM, 0)
            self.publish(Topic.YAW_PWM, 0)
            self.publish(Topic.DEPTH_PID, True)
            self.publish(Topic.YAW_PID, True)

    def check_enable_pids(self, input_value):
        if input_value:
            self.publish(Topic.DEPTH_PID, False)
            self.publish(Topic.YAW_PID, False)

    def check_yaw_setpoint_toggle(self, joy_input, saved_key, magnitude, angle):
        """When the button is pressed: If the magnitude is close to 1, save the angle. Otherwise, erase the saved value.
        """
        if self.inputs[joy_input]:
            if magnitude >= 0.95:
                self.saved[saved_key] = angle
            else:
                self.saved[saved_key] = None
            pass

    def input_yaw_setpoint(self, joy_input_x, joy_input_y, saved_key):
        """Calculates the magnitude and angle of the input (x,y). If magnitude is close to 1, set the angle as setpoint. If not close to 1 and there's a saved angle, use that angle. Else, do nothing.
        """
        x = self.inputs[joy_input_x] * -1 # change the x-coord to the standard coordinate system
        y = self.inputs[joy_input_y]
        magnitude = math.sqrt(x**2 + y**2)
        angle = math.atan2(y, x) + math.pi/2 # rotate the angle so that 0 is north
        if magnitude >= 0.95:
            self.publish(Topic.YAW_SETPOINT, angle)
        elif self.saved[saved_key]:
            self.publish(Topic.YAW_SETPOINT, self.saved[saved_key])
        print(x,y)
        return magnitude, angle

    def input_rotate(self, joy_input_counterclockwise, joy_input_clockwise):
        """Calculates how much the two joystick triggers are being pressed to determine which direction to rotate. If the difference is positive, the AUV rotates counterclockwise. If negative, counterclockwise.
        """
        ccw = np.interp(self.inputs[joy_input_counterclockwise], [-1,1], [1, 0])
        cw = np.interp(self.inputs[joy_input_clockwise], [-1,1], [1, 0])
        yaw_pwm = ccw - cw
        yaw_pwm = int(yaw_pwm * Joystick.MAX_YAW_PWM_FEEDBACK)
        self.publish(Topic.YAW_PWM_FEEDBACK, yaw_pwm)

    def publish(self, topic, value):
        """Uses a Publisher to publish the value (wrapped in a msg object).
        The topic is a key to access a tuple (Publisher, msg).
        The msg object can be an Int16(), Bool(), etc.
        """
        msg = self.topics[topic]['msg']
        msg.data = value
        self.topics[topic]['publisher'].publish(msg)
        print("published \t{} \t{}".format(topic, value))

    def check_toggle(self, toggle_button, input_value, saved_value):
        """When the button is pressed and there's a saved value, set the saved value to None. Otherwise, set it to the axis' value.
        """
        if toggle_button and saved_value is None:
            saved_value = input_value
        elif toggle_button:
            saved_value = None
        return saved_value

    def input_forward(self, joy_input):
        """Publishes a value to the topic /yaw_pwm. . If a saved value exists, use it. Otherwise, use the joystick input.
        """
        if self.saved[joy_input]:
            value = self.saved[joy_input]
        else:
            value = self.inputs[joy_input]
        yaw_pwm = np.interp(value, [-1, 1], [0, Joystick.MAX_YAW_PWM])
        self.publish(Topic.YAW_PWM, yaw_pwm)

    def is_depth_valid(self):
        """Check that the depth was received recently and is above MAX_DEPTH.
        """
        if not self.depth_state:
            print('depth was never received')
            return False
        if self.depth_last_received > 100:
            print('depth not received for 100 ticks')
            return False
        if self.depth_state > Joystick.MAX_DEPTH:
            print('depth > MAX_DEPTH')
            return False
        return True

    def input_depth(self, joy_input):
        """When the input is -1, disable depth PID and slowly increase depth_pwm
        When the input becomes 0 (after being -1), enble depth PID, set depth_setpoint and set depth_pwm to 0
        When input is +1, disable PID and set depth_pwm to 0
        """
        # If input is +1
        if self.inputs[joy_input] > 0.5:
            self.depth_pwm_input = 0
            self.publish(Topic.DEPTH_PID, False)
            self.publish(Topic.DEPTH_PWM, 0)
        # If input is -1
        elif self.inputs[joy_input] < -0.5 and self.is_depth_valid():
            self.publish(Topic.DEPTH_PID, False)
            if self.depth_pwm_input < Joystick.MAX_DEPTH_PWM:
                self.depth_pwm_input += 1
            self.publish(Topic.DEPTH_PWM, self.depth_pwm_input)
        # If input is 0
        else:
            if self.depth_pwm_input != 0:
                self.depth_pwm_input = 0
                self.publish(Topic.DEPTH_PWM, 0)
                self.publish(Topic.DEPTH_PID, True)
                # TODO -- CHECK DEPTH IS VALID
                # TODO -- SETPOINT SHOULD NOT BE SET REPEATEDLY -- FIXED
                self.publish(Topic.DEPTH_SETPOINT, self.depth_state)

import atexit

def kill_motors():
    print("killed motors")
    joystick = JoystickController()
    joystick.check_disable_pids(1) # turns off PIDs and sets PWMs to 0
    # off = Bool()
    # off.data = False
    # zeroInt = Int16()
    # zeroInt.data = 0
    # zeroFloat = Float64()
    # zeroFloat.data = 0.0
    # yaw_pub = rospy.Publisher('/yaw_pwm', Float64, queue_size=10)
    # depth_pub = rospy.Publisher('/depth_pwm', Int16, queue_size=10)
    # yaw_pid_pub = rospy.Publisher('yaw_control/pid', Bool, queue_size=10)
    # depth_pid_pub = rospy.Publisher('/depth_control/pid', Bool, queue_size=10)
    # yaw_pub.publish(zeroFloat)
    # depth_pub.publish(zeroInt)
    # yaw_pid_pub.publish(off)
    # depth_pid_pub.publish(off)

atexit.register(kill_motors)

def main():
    rospy.init_node('joystickController') # create a ROS node
    rate = rospy.Rate(20) # set to 20 Hz (this program loops 20 times per second)

    joystick = JoystickController() # initializes this object. Sets up Publishers and Subscribers

    print("Python Project Running....")
    while not rospy.is_shutdown(): # main loop
        joystick.execute() # check the joystick inputs and use them to control the AUV
        rate.sleep()

if __name__ == '__main__':
    main()