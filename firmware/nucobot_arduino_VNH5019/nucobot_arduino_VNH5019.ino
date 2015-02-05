#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include "DualVNH5019MotorShield.h"

ros::NodeHandle nh;
DualVNH5019MotorShield md;

#define WHEEL_SEPARATION 0.17 // Wheel separation in meters
#define TRANSLATION_FACTOR 400 // The speed in m/s is multiplied by this factor to get the PWM 

void motor_cb(const geometry_msgs::Twist& cmd_msg){
  double right = 0;
  double left  = 0;
  
  // linear is in meters per second
  right += cmd_msg.linear.x;
  left  += cmd_msg.linear.x;
  
  // angular is in radians per second
  right -= cmd_msg.angular.z * WHEEL_SEPARATION / 2.0;
  left  += cmd_msg.angular.z * WHEEL_SEPARATION / 2.0;

  md.setSpeeds(int(right*TRANSLATION_FACTOR), int(left*TRANSLATION_FACTOR));
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", motor_cb);

void setup()
{
  md.init();
  nh.initNode();
  nh.subscribe(sub);  
}

void loop()
{
  nh.spinOnce();
  delay(1);
}
