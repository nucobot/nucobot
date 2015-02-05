#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <ros.h>
#include <std_msgs/Int16.h>

ros::NodeHandle nh;


/*
DRV8833 Dual Motor Driver Carrier connection

+-----+-----+-------------------------+
|xIN1 |xIN2 |         FUNCTION        |
+-----+-----+-------------------------+
| PWM |  0  | Forward PWM, fast decay |
+-----+-----+-------------------------+
|  1  | PWM | Forward PWM, slow decay |
+-----+-----+-------------------------+
|  0  | PWM | Reverse PWM, fast decay |
+-----+-----+-------------------------+
| PWM |  1  | Reverse PWM, slow decay |
+-----+-----+-------------------------+
*/

#define AIN1 6   // (PWM)
#define AIN2 9   // (PWM)
#define BIN2 10  // (PWM)
#define BIN1 11  // (PWM)

void motor_cb( const std_msgs::Int16& cmd_msg){
  if (cmd_msg.data > 0) {
    analogWrite(BIN1, 0);
    analogWrite(BIN2, cmd_msg.data);
  }
  if (cmd_msg.data <= 0) {
    analogWrite(BIN2, 0);
    analogWrite(BIN1, -cmd_msg.data);
  }
}


ros::Subscriber<std_msgs::Int16> sub("cmd_vel", motor_cb);

void setup(){
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  analogWrite(AIN1, 0);
  analogWrite(AIN2, 0);
  analogWrite(BIN1, 0); 
  analogWrite(BIN2, 0); 

  nh.initNode();
  nh.subscribe(sub);  
}

void loop(){
  nh.spinOnce();
  delay(1);
}

