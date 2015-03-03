#include <Arduino.h>
#include <SPI.h>
#include <math.h>

#include <IMU.h>
#include <DualVNH5019MotorShield.h>

#include <ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/Imu.h>


//DEFINES
#define WHEEL_SEPARATION 0.17   // Wheel separation in meters
#define TRANSLATION_FACTOR 400  // The speed in m/s is multiplied by this factor to get the PWM 
#define MOTOR1_INT 1
#define MOTOR1_REV_PIN 4
#define MOTOR2_INT 0
#define MOTOR2_REV_PIN 5
#define BARO_ENABLE_PIN 40
#define IMU_ENABLE_PIN 53

//FUNCTIONS
void motor_1_cb();
void motor_2_cb();
void motor_cb(const geometry_msgs::Twist& cmd_msg);


//VARS
long int publish_timer;
long int motor_1_timer, motor_2_timer;
//DEVICES
DualVNH5019MotorShield md;
IMU imu;
//ROS
ros::NodeHandle nh;
sensor_msgs::Imu imu_msg;
geometry_msgs::Point wheel_odom_msg;
ros::Publisher pub_imu("imu_data",&imu_msg);
ros::Publisher pub_wheel_odom("raw_wheel_odometry", &wheel_odom_msg);
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", motor_cb);



void setup() {
    //SPI
    SPI.begin();  
    SPI.setClockDivider(SPI_CLOCK_DIV16); 
    SPI.setBitOrder(MSBFIRST); 
    SPI.setDataMode(SPI_MODE0);
    delay(100);

    //DISABLE BARO
    pinMode(BARO_ENABLE_PIN, OUTPUT);
    digitalWrite(BARO_ENABLE_PIN, HIGH); //stop the barometer from holding the SPI bus

    //IMU    
    pinMode(IMU_ENABLE_PIN, OUTPUT);
    imu.Init(IMU_ENABLE_PIN);  // configure chip

    //MOTORS
    pinMode(MOTOR1_REV_PIN, INPUT);
    pinMode(MOTOR2_REV_PIN, INPUT);
    attachInterrupt(MOTOR1_INT, motor_1_cb, RISING);
    attachInterrupt(MOTOR2_INT, motor_2_cb, RISING);
    md.init();

    //ROS
    nh.initNode();
    nh.subscribe(sub);
    nh.advertise(pub_imu);
    nh.advertise(pub_wheel_odom);
}


void loop()
{
    if ( (millis()-publish_timer) > 50) {
        //IMU
        imu_msg.angular_velocity.x = (float)(imu.GyroX());
        imu_msg.angular_velocity.y = (float)(imu.GyroY());
        imu_msg.angular_velocity.z = (float)(imu.GyroZ());
        imu_msg.linear_acceleration.x = (float)(imu.AcceX());
        imu_msg.linear_acceleration.y = (float)(imu.AcceY());
        imu_msg.linear_acceleration.z = (float)(imu.AcceZ());
        //MOTORS
        wheel_odom_msg.x = motor_2_timer;
        wheel_odom_msg.y = -motor_1_timer;
        motor_1_timer = 0;
        motor_2_timer = 0;
        //ROS
        imu_msg.header.stamp = nh.now();
        pub_wheel_odom.publish(&wheel_odom_msg);
        pub_imu.publish(&imu_msg);
        publish_timer = millis();
    }
    nh.spinOnce();
}




void motor_1_cb()
{
    if (digitalRead(MOTOR1_REV_PIN) == HIGH) {
        motor_1_timer --;
    }
    else {
        motor_1_timer ++;
    }
}


void motor_2_cb()
{
    if (digitalRead(MOTOR2_REV_PIN) == HIGH) {
        motor_2_timer --;
    }
    else {
        motor_2_timer ++;
    }
}


void motor_cb(const geometry_msgs::Twist& cmd_msg)
{
    double right = 0;
    double left  = 0;
    
    // linear is in meters per second
    right += cmd_msg.linear.x;
    left  += cmd_msg.linear.x;
    
    // angular is in radians per second
    right -= cmd_msg.angular.z * WHEEL_SEPARATION / 2.0;
    left  += cmd_msg.angular.z * WHEEL_SEPARATION / 2.0;

    md.setSpeeds(int(right*TRANSLATION_FACTOR), -int(left*TRANSLATION_FACTOR));
}
