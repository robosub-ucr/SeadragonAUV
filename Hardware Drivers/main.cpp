#include "mbed.h"
#include "Servo.h"
#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include "thruster.h"
#include "node.h"

int main() {
    Node nd;
    
    nd.nh.initNode();
    nd.nh.advertise(depth_publisher);
    nd.nh.subscribe(depth_pwm_subscriber);
    nd.nh.subscribe(yaw_pwm_subscriber);
    nd.nh.subscribe(fb_yaw_subscriber);
    nd.nh.subscribe(fb_depth_subscriber);
    nd.nh.subscribe(pan_servo_subscriber); // *** Currently no servo ***

    nd.th.testLed = 0;
    
    while(1) {       
        // If data is recieved, blink led
        if (nd.th.device.readable()) {
            // Obtain depth 
            nd.depth = nd.th.device.getc(); // returns char from serial port
            // toggle led on
            nd.th.testLed = 1;
        }
        
        // Publish depth information
        nd.depth_msg.data = depth;
        nd.depth_publisher.publish(&depth_msg);
        
        // Depth thrust + feedback
        nd.th.depthtot = nd.th.m1pwm + nd.th.depthfeedback;
        
        // Sets motor pwm in microseconds "void set_depth_pwm(int depthtot)"  tag-depth
        nd.th.m1.pulsewidth_us(nd.th.depthtot);
        nd.th.m2.pulsewidth_us(nd.th.depthtot);
        nd.th.m3.pulsewidth_us(nd.th.depthtot);
        
        // back right thruster is T200, other depth thrusters are T100. So we need to adjust PWM for that thruster
        if (m4pwm > 1500) {
            nd.th.depthtot_4map = nd.th.map(nd.th.m4pwm, 1500, 1800, 1500, 1630); // [130(m4pwm - 1500)/300] + 1500 --> >1500
        }
        else if (m4pwm < 1500) {
            nd.th.depthtot_4map = nd.th.map(nd.th.m4pwm, 1500, 1200, 1500, 1370); // [130(m4pwm - 1500)/300] + 1500 --> <1500
        }
        else {
            nd.th.depthtot_4map = nd.th.PWMBASELINE;
        }
        m4.pulsewidth_us(nd.th.depthtot_4map);


        // Fwd thrust + feedback
        nd.th.lthrust_tot = nd.th.lthrustpwm + nd.th.lfeedback;
        nd.th.rthrust_tot = nd.th.rthrustpwm + nd.th.rfeedback;
        
        // Output pwm to fwd/rev thrusters in microseconds   
        nd.th.lthrust.pulsewidth_us(nd.th.lthrust_tot);
        nd.th.rthrust.pulsewidth_us(nd.th.rthrust_tot);
        
        // actuate pan tilt camera *** servo not used ***
        myservo = pan_servo;    // controls servo movement based on adjusting values from 0.0 to 1.0
        
        nd.nh.spinOnce();  // ROS only processes callbacks when you tell it to
        wait_ms(5);
        nd.th.testLed = 0;
    }
}