#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include "DualVNH5019MotorShield.h"
/*
* 1ina 8
* 1inb 11
* 2ina 12
* 2inb 13
*/
ros::NodeHandle nh;
DualVNH5019MotorShield md;
geometry_msgs::Point wheel_odom_msg;
long int publish_timer, motor_1_timer, motor_2_timer;

#define WHEEL_SEPARATION 0.17 // Wheel separation in meters
#define TRANSLATION_FACTOR 400 // The speed in m/s is multiplied by this factor to get the PWM 

#define MOTOR1_INT 1
#define MOTOR1_REV_PIN 4
#define MOTOR2_INT 0
#define MOTOR2_REV_PIN 5

void motor_1_cb(){
  if (digitalRead(MOTOR1_REV_PIN) == HIGH) {
    motor_1_timer --;
  }
  else {
    motor_1_timer ++;
  }
}

void motor_2_cb(){
  if (digitalRead(MOTOR2_REV_PIN) == HIGH) {
    motor_2_timer --;
  }
  else {
    motor_2_timer ++;
  }
}

void motor_cb(const geometry_msgs::Twist& cmd_msg){
  double right = 0;
  double left  = 0;
  
  // linear is in meters per second
  right += cmd_msg.linear.x;
  left  += cmd_msg.linear.x;
  
  // angular is in radians per second
  right -= cmd_msg.angular.z * WHEEL_SEPARATION / 2.0;
  left  += cmd_msg.angular.z * WHEEL_SEPARATION / 2.0;

  md.setSpeeds(int(left*TRANSLATION_FACTOR), -int(right*TRANSLATION_FACTOR));
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", motor_cb);
ros::Publisher pub_wheel_odom("raw_wheel_odometry", &wheel_odom_msg);

void setup()
{
  pinMode(MOTOR1_REV_PIN, INPUT);
  pinMode(MOTOR2_REV_PIN, INPUT);
  md.init();
  nh.initNode();
  nh.advertise(pub_wheel_odom);
  nh.subscribe(sub);
  attachInterrupt(MOTOR1_INT, motor_1_cb, RISING);
  attachInterrupt(MOTOR2_INT, motor_2_cb, RISING);
}

void loop()
{
  if ( (millis()-publish_timer) > 50){
    wheel_odom_msg.x = motor_2_timer;
    wheel_odom_msg.y = -motor_1_timer;
    motor_1_timer = 0;
    motor_2_timer = 0;
    pub_wheel_odom.publish(&wheel_odom_msg);
    publish_timer = millis();
  }
  nh.spinOnce();
}
