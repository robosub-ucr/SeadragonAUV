#include "mbed.h"
#include "Servo.h"
#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>

//------------Declare Global Variables------------------//

//      (m1)         (m2)
//          |       |
//          |       |
// (Ltrhust)|       |(Rthrust)
//          |       |
//          |       |    
//      (m3)         (m4)   

// Initialize PWM objects
PwmOut      m1(D8);
PwmOut      m2(D9);
PwmOut      m3(D10);
PwmOut      m4(D11);
PwmOut      lthrust(D12);
PwmOut      rthrust(D14);

int PWMBASELINE = 1500;
// Initialize all pwm values to the baseline value
int m1pwm = PWMBASELINE;
int m2pwm = PWMBASELINE;
int m3pwm = PWMBASELINE;
int m4pwm = PWMBASELINE;

int lthrustpwm = PWMBASELINE;
int rthrustpwm = PWMBASELINE;

int lfeedback  = 0;
int rfeedback  = 0;
int lthrust_tot = 0;
int rthrust_tot = 0;

int depthfeedback = 0;
int depthtot = 0;
int depthtot_4map = 0;

//Initialize LED state and torpedo 
int light_state = 0;
bool torpedo_launch = 0; 
int data_send = 0x00; //variable to combine led and torpedo data

float map(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Tx - 11, Rx - 9
RawSerial device(PA_15, PB_7);//Pins on stm32 that arduino mega communication pins 18 and 19
DigitalOut testLed(LED1);
Servo myservo(D3);
double pan_servo = 0;

//--------------ROS Node Configuration---------------
//Node Handle is main access point to communication. This line initializes node
ros::NodeHandle nh;

std_msgs::Int16 depth_msg;
int depth = 0;
ros::Publisher depth_publisher("depth", &depth_msg);

void depth_motor_callback( const std_msgs::Int16& msg)
{  
    if (msg.data == 0)
    {
        m1pwm = m2pwm = m3pwm = m4pwm = PWMBASELINE;   
    }
    else if (msg.data < 0)   
    {
        m1pwm = m2pwm = m3pwm = m4pwm = PWMBASELINE - 25 + msg.data;
    }
    else{
        // Update motor pwm
        m1pwm = PWMBASELINE + 25 + msg.data;
        m2pwm = PWMBASELINE + 25 + msg.data;
        m3pwm = PWMBASELINE + 25 + msg.data;
        m4pwm = PWMBASELINE + 25 + msg.data;
    }
}

// T100 (except back-right thruster which is T200)
// Forward thrust callback returns valules [0:150]
void thrust_motor_callback( const std_msgs::Int16& msg)
{
    // Updates pwm values for forward thrust motors
    if (msg.data == 0)
    {
        // Stop motors
        lthrustpwm = rthrustpwm = PWMBASELINE;   
    }
    else if (msg.data < 0)
    {
        // Go reverse
        lthrustpwm = PWMBASELINE - 25 + msg.data;
        rthrustpwm = PWMBASELINE - 25 + msg.data;
    }
    else{
        // Go forward
        lthrustpwm = PWMBASELINE + 25 + msg.data;
        rthrustpwm = PWMBASELINE + 25 + msg.data;
    }
}

// T200
// Receives values ranging [0:75]
void yaw_feedback_callback( const std_msgs::Int16& msg)
{
    if (lthrustpwm == PWMBASELINE){
        if (msg.data > 0.25){
            lfeedback = (msg.data + 29) * -1;
            rfeedback = (msg.data + 29);
        }
        else if (msg.data < -0.25){
            lfeedback = (msg.data - 29) * -1;
            rfeedback = (msg.data - 29);   
        }
        else{
            lfeedback = rfeedback = 0;   
        }
    }
    else{   
        lfeedback = msg.data * -1;
        rfeedback = msg.data;       
    }
}   

// Receives depth feedback vallues from [0:50]
void depth_feedback_callback( const std_msgs::Int16& msg)
{
    // depth feedback control
    if (msg.data > 1)
    {
        depthfeedback = 25 + msg.data;
    }
    else if (msg.data < -1)
    {
        depthfeedback = -25 + msg.data;   
    }
    else{
        depthfeedback = 0;
    }   
}

void pan_callback( const std_msgs::Int16& msg)
{
    pan_servo = msg.data/100.0;
}

//Receives State that the LED strip should represent 
void led_state_callback(const std_msgs::Int16& msg){   
  light_state = msg.data;
}

//Receives when solenoid should be actuated
void torpedo_callback(const std_msgs::Bool& msg){
    torpedo_launch = msg.data;        
}

ros::Subscriber<std_msgs::Int16> depth_pwm_subscriber("depth_pwm", depth_motor_callback);
ros::Subscriber<std_msgs::Int16> yaw_pwm_subscriber("yaw_pwm", thrust_motor_callback);
ros::Subscriber<std_msgs::Int16> fb_yaw_subscriber("yaw_pwm_feedback", yaw_feedback_callback);
ros::Subscriber<std_msgs::Int16> fb_depth_subscriber("depth_pwm_feedback", depth_feedback_callback);
ros::Subscriber<std_msgs::Int16> pan_servo_subscriber("pan_servo", pan_callback);

ros::Subscriber<std_msgs::Int16> led_state_subscriber("light_state", led_state_callback);
//ros::Subscriber<std_msgs::Bool> torpedo_subscriber("torpedo_shoot", torpedo_callback);

int main() 
{    
    m1.pulsewidth_us(1500);
    m2.pulsewidth_us(1500);
    m3.pulsewidth_us(1500);
    m4.pulsewidth_us(1500);
    lthrust.pulsewidth_us(1500);
    rthrust.pulsewidth_us(1500);
    wait(7);
    
    nh.initNode();
    nh.advertise(depth_publisher);
    nh.subscribe(depth_pwm_subscriber);
    nh.subscribe(yaw_pwm_subscriber);
    nh.subscribe(fb_yaw_subscriber);
    nh.subscribe(fb_depth_subscriber);
    nh.subscribe(pan_servo_subscriber);
    
    nh.subscribe(led_state_subscriber);
    //nh.subscribe(torpedo_subscriber);

    testLed = 0;
    
    while(1) 
    {       
        // If data is recieved, blink led
        if (device.readable())
        {
            // Obtain depth 
            depth = device.getc();
            // toggle led on
            testLed = 1;
        }
        
        // Publish depth information
        depth_msg.data = depth;
        depth_publisher.publish(&depth_msg);
        
        // Depth thrust + feedback
        depthtot = m1pwm + depthfeedback;
        m4pwm    = m1pwm + depthfeedback;
        
        
        // Output motor pwm
        m1.pulsewidth_us(depthtot);
        m2.pulsewidth_us(depthtot);
        m3.pulsewidth_us(depthtot);
        
        // back right thruster is T200, other depth thrusters are T100. So we need to adjust PWM for that thruster
        if (m4pwm > 1500){
            depthtot_4map = map(m4pwm, 1500, 1800, 1500, 1630);
        }else if (m4pwm < 1500){
            depthtot_4map = map(m4pwm, 1500, 1200, 1500, 1370);
        }else{
            depthtot_4map = PWMBASELINE;
        }
        m4.pulsewidth_us(depthtot_4map);
        
        // Fwd thrust + feedback
        lthrust_tot = lthrustpwm + lfeedback;
        rthrust_tot = rthrustpwm + rfeedback;
        
        // Output pwm to fwd/rev thrusters    
        lthrust.pulsewidth_us(lthrust_tot);
        rthrust.pulsewidth_us(rthrust_tot);
        
        // actuate pan tilt camera
        myservo = pan_servo;
        
        nh.spinOnce();
       
        //light state upper 4 bits
        data_send  = light_state << 4;    
        //torpedo lower 4 bits
        data_send = data_send + (torpedo_launch ? 1:0);  
        //send concatenated data
        device.putc(data_send);   
        
        wait_ms(5);
        testLed = 0;
    }
}
/*
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
    //nd.nh.subscribe(pan_servo_subscriber); // *** Currently no servo ***

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
        nd.th.m4.pulsewidth_us(nd.th.depthtot_4map);


        // Fwd thrust + feedback
        nd.th.lthrust_tot = nd.th.lthrustpwm + nd.th.lfeedback;
        nd.th.rthrust_tot = nd.th.rthrustpwm + nd.th.rfeedback;
        
        // Output pwm to fwd/rev thrusters in microseconds   
        nd.th.lthrust.pulsewidth_us(nd.th.lthrust_tot);
        nd.th.rthrust.pulsewidth_us(nd.th.rthrust_tot);
        
        // actuate pan tilt camera *** servo not used ***
        //nd.th.myservo = nd.th.pan_servo;    // controls servo movement based on adjusting values from 0.0 to 1.0
        
        nd.nh.spinOnce();  // ROS only processes callbacks when you tell it to
        wait_ms(5); // nd.wait_ms(5);
        nd.th.testLed = 0;
    }
}
*/
