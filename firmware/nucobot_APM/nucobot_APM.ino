// APM CONSTRAINTS:
// TYPE        |  ARDUINO-PIN  | APM-PIN 
//-------------+---------------+--------
// OC3A        |       5       |  pwm-8
// INT0        |       2       |  pwm-7
// INT1        |       3       |  pwm-6
// OC4A        |       6       |  pwm-5
// OC4B        |       7       |  pwm-4
// OC4C        |       8       |  pwm-3
// OC1A/PCINT5 |       11      |  pwm-2
// OC1B/PCINT6 |       12      |  pwm-1
//-------------+---------------+--------
// LEDB        |       25      |    -
// LEDY        |       26      |    -
// LEDR        |       27      |    -


#include <Arduino.h>
#include <SPI.h>
#include <math.h>
#include <Wire.h>

#include <HMC5883L.h>
#include <IMU.h>
#include <DualVNH5019MotorShield.h>

#include <ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>


//DEFINES         
#define WHEEL_SEPARATION 0.17                                
#define TRANSLATION_FACTOR 100                                                                  
#define BARO_ENABLE_PIN 40
#define IMU_ENABLE_PIN 53
#define ToD(x) (x/131)
#define ToG(x) (x*9.80665/16384)
#define BAT1_PIN 4
#define BAT2_PIN 5

#define ENCODER0_INT 0       // Interrupt number
#define ENCODER0_INT_WIRE 2  // Interrupt input pin (according to datasheet)
#define ENCODER0_REF_WIRE 5  // Reference wire to check rotation direction
#define ENCODER1_INT 1       // Interrupt number
#define ENCODER1_INT_WIRE 3  // Interrupt input pin (according to datasheet)
#define ENCODER1_REF_WIRE 6  // Reference wire to check rotation direction


           
//FUNCTIONS
void setup();
void loop();
void motor_cb(const geometry_msgs::Twist& cmd_msg);
void encoder0_cb();
void encoder1_cb();

//VARS     
long int publish_timer;
long int encoder0_cnt, encoder1_cnt;

//DEVICES         
DualVNH5019MotorShield md(54, 55, 69, 68, 56, 57, 67, 66);
IMU imu;
HMC5883L compass;
//ROS          
ros::NodeHandle nh;
//ROS-MSGS          
sensor_msgs::Imu imu_msg;
geometry_msgs::Vector3Stamped mag_msg;
geometry_msgs::Point encoder_msg;
std_msgs::Float32 bat1_msg, bat2_msg;
//ROS-TOPICS            
ros::Publisher pub_imu("imu/data_raw",&imu_msg);
ros::Publisher pub_mag("imu/mag",&mag_msg);
ros::Publisher pub_enc("encoders", &encoder_msg);
ros::Publisher pub_bat1("battery_voltage/1", &bat1_msg);
ros::Publisher pub_bat2("battery_voltage/2", &bat2_msg);
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", motor_cb);



void setup() {
 //I2C        
    Wire.begin();
 //SPI        
    SPI.begin();  
    SPI.setClockDivider(SPI_CLOCK_DIV16); 
    SPI.setBitOrder(MSBFIRST); 
    SPI.setDataMode(SPI_MODE0);
    delay(100);
 //BARO                  
    pinMode(BARO_ENABLE_PIN, OUTPUT);
    digitalWrite(BARO_ENABLE_PIN, HIGH);                                              
 //IMU             
    pinMode(IMU_ENABLE_PIN, OUTPUT);
    imu.Init(IMU_ENABLE_PIN);                   
 //COMPASS            
    compass = HMC5883L();
    compass.SetScale(1.3);
    compass.SetMeasurementMode(Measurement_Continuous);
 //MISCELLANEOUS
    pinMode(25, OUTPUT); // BLUE LED
    pinMode(26, OUTPUT); // YELLOW LED
    pinMode(27, OUTPUT); // RED LED
 // INTERRUPTS
    attachInterrupt(ENCODER0_INT, encoder0_cb, RISING);
    pinMode(ENCODER0_INT_WIRE, INPUT);
    pinMode(ENCODER0_REF_WIRE, INPUT);
    digitalWrite(ENCODER0_INT_WIRE, HIGH); // Turn on internal pull-up
    digitalWrite(ENCODER0_REF_WIRE, HIGH); // Turn on internal pull-up
    attachInterrupt(ENCODER1_INT, encoder1_cb, RISING);
    pinMode(ENCODER1_INT_WIRE, INPUT);
    pinMode(ENCODER1_REF_WIRE, INPUT);
    digitalWrite(ENCODER1_INT_WIRE, HIGH); // Turn on internal pull-up
    digitalWrite(ENCODER1_REF_WIRE, HIGH); // Turn on internal pull-up
 // VAR INITIALIZATION
    encoder0_cnt = 0;
    encoder1_cnt = 0;
 //MOTORS           
    md.init();
 //ROS        
    nh.initNode();
    nh.subscribe(sub);
    nh.advertise(pub_imu);
    nh.advertise(pub_mag);
    nh.advertise(pub_bat1);
    nh.advertise(pub_bat2);
    nh.advertise(pub_enc);
}

void loop()
{  
    if ( (millis()-publish_timer) > 50) {
 //IMU             
        imu_msg.angular_velocity.x = ToD((float)(imu.GyroX()));
        imu_msg.angular_velocity.y = ToD((float)(imu.GyroY()));
        imu_msg.angular_velocity.z = ToD((float)(imu.GyroZ()));
        imu_msg.linear_acceleration.x = ToG((float)(imu.AcceX()));
        imu_msg.linear_acceleration.y = ToG((float)(imu.AcceY()));
        imu_msg.linear_acceleration.z = ToG((float)(imu.AcceZ()));
 //COMPASS                 
        MagnetometerScaled scaled = compass.ReadScaledAxis();
        mag_msg.vector.x = scaled.XAxis;
        mag_msg.vector.y = scaled.YAxis;
        mag_msg.vector.z = scaled.ZAxis;
 //BATTERYS                  
        bat1_msg.data = round(((analogRead(BAT1_PIN))*15000.0*0.968096/1023.0));
        bat2_msg.data = round(((analogRead(BAT2_PIN))*15000.0*0.962191/1023.0));
 //ENCODERS
        encoder_msg.x = encoder0_cnt;
        encoder_msg.y = encoder1_cnt;
        encoder0_cnt = 0;
        encoder1_cnt = 0;
 //ROS             
        mag_msg.header.stamp = nh.now();
        imu_msg.header.stamp = nh.now();
        pub_imu.publish(&imu_msg);
        pub_mag.publish(&mag_msg);
        pub_bat1.publish(&bat1_msg);
        pub_bat2.publish(&bat2_msg);
        pub_enc.publish(&encoder_msg);
        publish_timer = millis();
    }
    nh.spinOnce();
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

    md.setSpeeds(int(right*TRANSLATION_FACTOR), int(left*TRANSLATION_FACTOR));
}

void encoder0_cb(){
  if (digitalRead(ENCODER0_REF_WIRE) == HIGH) encoder0_cnt --;
  else encoder0_cnt ++;
}

void encoder1_cb(){
  if (digitalRead(ENCODER1_REF_WIRE) == HIGH) encoder1_cnt --;
  else encoder1_cnt ++;
}
