#include "mbed.h"
#include "Servo.h"
#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include "thruster.h"
#include "node.h"

int main() {    
    nh.initNode();
    nh.advertise(depth_publisher);
    nh.subscribe(depth_pwm_subscriber);
    nh.subscribe(yaw_pwm_subscriber);
    nh.subscribe(fb_yaw_subscriber);
    nh.subscribe(fb_depth_subscriber);
    nh.subscribe(pan_servo_subscriber); // *** Currently no servo ***

    testLed = 0;
    
    while(1) {       
        // If data is recieved, blink led
        if (device.readable()) {
            // Obtain depth 
            depth = device.getc(); // returns char from serial port
            // toggle led on
            testLed = 1;
        }
        
        // Publish depth information
        depth_msg.data = depth;
        depth_publisher.publish(&depth_msg);
        
        // Depth thrust + feedback
        depthtot = m1pwm + depthfeedback;
        
        // Sets motor pwm in microseconds "void set_depth_pwm(int depthtot)"  tag-depth
        m1.pulsewidth_us(depthtot);
        m2.pulsewidth_us(depthtot);
        m3.pulsewidth_us(depthtot);
        
        // back right thruster is T200, other depth thrusters are T100. So we need to adjust PWM for that thruster
        if (m4pwm > 1500) {
            depthtot_4map = map(m4pwm, 1500, 1800, 1500, 1630); // [130(m4pwm - 1500)/300] + 1500 --> >1500
        }
        else if (m4pwm < 1500) {
            depthtot_4map = map(m4pwm, 1500, 1200, 1500, 1370); // [130(m4pwm - 1500)/300] + 1500 --> <1500
        }
        else {
            depthtot_4map = PWMBASELINE;
        }
        m4.pulsewidth_us(depthtot_4map);


        // Fwd thrust + feedback
        lthrust_tot = lthrustpwm + lfeedback;
        rthrust_tot = rthrustpwm + rfeedback;
        
        // Output pwm to fwd/rev thrusters in microseconds   
        lthrust.pulsewidth_us(lthrust_tot);
        rthrust.pulsewidth_us(rthrust_tot);
        
        // actuate pan tilt camera *** servo not used ***
        myservo = pan_servo;    // controls servo movement based on adjusting values from 0.0 to 1.0
        
        nh.spinOnce();  // ROS only processes callbacks when you tell it to
        wait_ms(5);
        testLed = 0;
    }
}