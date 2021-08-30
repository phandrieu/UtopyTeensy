//Include Instructions
#include <PID_v1.h>
#include <PWMServo.h>
#include "./VescUart.h"
#include <HardwareSerial.h>

//Speed Variables
double demandX = 0;
double demandZ = 0;
double demand1;
double demand2;

//Encoder variables


long EncVal1 = 0;
long EncVal2 = 0;
long prevEncVal1 = 0;
long prevEncVal2 =0;

int delta1 = 0;
int delta2 = 0;

VescUart vesc1;
VescUart vesc2;

//ROS Communication
#include "ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include <SoftwareSerial.h>

void velCallback(   const geometry_msgs::Twist& vel)
{
    demandX = vel.linear.x;
    demandZ = vel.angular.z;
}
std_msgs::String str_msg;
ros::NodeHandle nh;
ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel" , velCallback);
ros::Publisher chatter("chatter", &str_msg);
String fullmessage;
void setup(){

    //Encoders & Serial init
    Serial.begin(9600);
    Serial2.begin(115200);
    vesc1.setSerialPort(&Serial2);
    Serial3.begin(115200);
    vesc2.setSerialPort(&Serial3);
    Serial.print("OK");
    //ROS Init
    nh.initNode();
    nh.subscribe(sub);
    nh.advertise(chatter);
    char hw[13] = "hello world";
    str_msg.data = hw;
    chatter.publish( &str_msg );
    nh.spinOnce();
    int deadClock = 0;
}

void loop(){
    
    
    demand1 = (demandX*9900) + (demandZ*5000);
    demand2 = (demandX*9900) - (demandZ*5000);
    vesc1.setRPM(demand1);
    vesc2.setRPM(demand2);
    if(vesc1.getVescValues() && vesc2.getVescValues()){
      float rpm1 = (vesc1.data.rpm );
      float rpm2 = (vesc2.data.rpm);
      float amp = vesc1.data.avgMotorCurrent + vesc2.data.avgMotorCurrent ;
      float volt = vesc1.data.inpVoltage;
      fullmessage = String(rpm1) + "," + String(rpm2) + "," + String(amp) + "," + String(volt);
      char message[200];
      fullmessage.toCharArray(message, 200);
      str_msg.data = message;
    }
    chatter.publish(&str_msg);
    nh.spinOnce();
}
