#include "mbed.h"
#include "Servo.h"
#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include "node.h"

class Thruster {
    private:
        const int PWMBASELINE = 1500;
        int m1pwm, m2pwm, m3pwm, m4pwm, lthrustpwm, rthrustpwm;
        int lfeedback, rfeedback, lthrust_tot, rthrust_tot;
        int depthfeedback, depthtot;

    public:
        thruster() { // Constructor
            // Initialize PWM object to specfic pin (D8-12, 14); D13 used as predefined pin for LED1
            PwmOut      m1(D8);
            PwmOut      m2(D9);
            PwmOut      m3(D10);
            PwmOut      m4(D11);
            PwmOut      lthrust(D12);
            PwmOut      rthrust(D14);

            m1pwm = m2pwm = m3pwm = m4pwm = lthrustpwm = rthrustpwm = PWMBASELINE;
            lfeedback = rfeedback  = lthrust_tot = rthrust_tot = 0;
            depthfeedback = depthtot = 0;

            // Tx - (Pin PA_15) 11 from bottom, Rx - (Pin PB_7) 9 from bottom
            RawSerial device(PA_15, PB_7);  // Allows for UART communication between stm32 and arduino mega; PA_15(tx) PB_7(rx)
            DigitalOut testLed(LED1);  // LED1 predefined pin on mbed boards, 411RE - D13
            Servo myservo(D3);  // Controls servo using a PwmOut signal from 0.0 to 1.0 *** Currently no servo ***
            double pan_servo = 0;   // holds value that controls servo movement 0.0 to 1.0 *** Currently no servo ***


        }

        // simple map function to scale values for T100-T200 thrusters
        float map(float x, float in_min, float in_max, float out_min, float out_max) {
            return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
        }

        // Specific values (25, 29) are to help motors startup, for T100 and T200 respectively - further calibration may be required
        // May want to create a variable for these values
        // Updates depth thruster PWM value, thrusters oriented so +pwmbase: up/-pwmbase: down (check w/ team)
        void depth_motor_callback( const std_msgs::Int16& msg) {    //tag-depth  
            if (msg.data == 0) {
                m1pwm = m2pwm = m3pwm = m4pwm = PWMBASELINE;
            }   
            else if (msg.data < 0) {
                m1pwm = m2pwm = m3pwm = m4pwm = PWMBASELINE - 25 + msg.data;
            }
            else {  
                m1pwm = PWMBASELINE + 25 + msg.data;
                m2pwm = PWMBASELINE + 25 + msg.data;
                m3pwm = PWMBASELINE + 25 + msg.data;
                m4pwm = PWMBASELINE + 25 + msg.data;
            }

        // T100 (except back-right thruster which is T200)
        // Forward thrust callback returns values [0:150]
        // Updates thrust PWM values, +pwm: forward/-pwm: reverse
        void thrust_motor_callback( const std_msgs::Int16& msg) {   //tag-yaw
            // Updates pwm values for forward thrust motors
            if (msg.data == 0) {    // Stop motors
                lthrustpwm = rthrustpwm = PWMBASELINE;   
            }
            else if (msg.data < 0) {    // Go reverse
                lthrustpwm = PWMBASELINE - 25 + msg.data;
                rthrustpwm = PWMBASELINE - 25 + msg.data;
            }
            else {  // Go forward
                lthrustpwm = PWMBASELINE + 25 + msg.data;
                rthrustpwm = PWMBASELINE + 25 + msg.data;
            }
        }

        // T200
        // Receives values ranging [0:75]
        // Updates yaw PWM values
        void yaw_feedback_callback( const std_msgs::Int16& msg) {   //tag-yaw
            // Motors are off
            if (lthrustpwm == PWMBASELINE) {
                if (msg.data > 0.25) {
                    lfeedback = (msg.data + 29) * -1;
                    rfeedback = (msg.data + 29);
                }
                else if (msg.data < -0.25) {
                    lfeedback = (msg.data - 29) * -1;
                    rfeedback = (msg.data - 29);   
                }
                else {
                    lfeedback = rfeedback = 0;   
                }
            }
            // Motors are on
            else {   
                lfeedback = msg.data * -1;
                rfeedback = msg.data;       
            }
        }   

        // Receives depth feedback vallues from [0:50]
        void depth_feedback_callback( const std_msgs::Int16& msg) { //tag-depth
            // depth feedback control
            if (msg.data > 1) {
                depthfeedback = 25 + msg.data;
            }
            else if (msg.data < -1) {
                depthfeedback = -25 + msg.data;   
            }
            else {
                depthfeedback = 0;
            }   
        }

        // Converts input value to stay within range of pan_servo: 0.0 to 1.0
        void pan_callback( const std_msgs::Int16& msg) {    //tag-servo
            pan_servo = msg.data/100.0;
        }
}