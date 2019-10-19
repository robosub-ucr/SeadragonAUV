#include "mbed.h"
#include "Servo.h"
#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include "thruster.h"

class Node {
    private:
        std_msgs::Int16 depth_msg;
        int depth;
    public:
        Node() { // Constructor
            ros::NodeHandle nh; // interface for creating subscribers and publishers, library included within <ros.h>
            depth = 0;
            
            // Initialize subscribers
            ros::Subscriber<std_msgs::Int16> depth_pwm_subscriber("depth_pwm", depth_motor_callback);
            ros::Subscriber<std_msgs::Int16> yaw_pwm_subscriber("yaw_pwm", thrust_motor_callback);
            ros::Subscriber<std_msgs::Int16> fb_yaw_subscriber("yaw_pwm_feedback", yaw_feedback_callback);
            ros::Subscriber<std_msgs::Int16> fb_depth_subscriber("depth_pwm_feedback", depth_feedback_callback);
            ros::Subscriber<std_msgs::Int16> pan_servo_subscriber("pan_servo", pan_callback); // *** Currently no servo ***

            // Initialize publisher
            ros::Publisher depth_publisher("depth", &depth_msg);
        }
}