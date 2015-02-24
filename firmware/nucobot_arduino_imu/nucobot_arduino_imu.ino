#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <Wire.h>

#include <ros.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
ros::NodeHandle nh;

geometry_msgs::PoseStamped imu_msg;
long int publish_timer;

/*****************************************************************************/
//
//  Node for reading PmodGYRO and PmodACL
//  Pins usage: SDA and SCL only (A4 and A5)
//  Topic name imu_data
//  Topic structure: position - gyro data, orientation - acl data
//
/*****************************************************************************/


#define GYRO_ADDR 0x69
#define GYRO_CTRL_REG1_ADDR 0x20
#define GYRO_CTRL_REG1_VALUE 0x0F
#define GYRO_OUT_X_L 0x28
#define GYRO_OUT_X_H 0x29
#define GYRO_OUT_Y_L 0x2A
#define GYRO_OUT_Y_H 0x2B
#define GYRO_OUT_Z_L 0x2C
#define GYRO_OUT_Z_H 0x2D

#define ACL_ADDR 0x1D
#define ACL_POWER_CTL_ADDR 0x2D
#define ACL_POWER_CTL_VALUE 0x08
#define ACL_DATA_FORMAT_ADDR 0x31
#define ACL_DATA_FORMAT_VALUE 0x08
#define ACL_DATAX_L 0x32
#define ACL_DATAX_H 0x33
#define ACL_DATAY_L 0x34
#define ACL_DATAY_H 0x35
#define ACL_DATAZ_L 0x36
#define ACL_DATAZ_H 0x37

/*****************************************************************************/

int readRegisters(char address, char low, char hi)
{
  int result = 0;
  Wire.beginTransmission(address); // transmit to device #address
  Wire.write(hi);             // sets register pointer to high register 
  Wire.endTransmission();   
  Wire.requestFrom(address, 1);    // request  byte from slave device 
  if (Wire.available())   // slave may send less than requested
  {
    result = Wire.read(); // receive a byte as character
  }
  result *= 256;
  Wire.beginTransmission(address); // transmit to device #address
  Wire.write(low);             // sets register pointer to low register 
  Wire.endTransmission();   
  Wire.requestFrom(address, 1);    // request  byte from slave device 
  if (Wire.available())   // slave may send less than requested
  {
    result += Wire.read(); // receive a byte as character
  }
  return result;
}

ros::Publisher pub_imu("imu_raw_data",&imu_msg);

/*****************************************************************************/

void read_imu()
{
  
  imu_msg.pose.position.x = (float)(readRegisters(GYRO_ADDR, GYRO_OUT_X_L, GYRO_OUT_X_H));
  imu_msg.pose.position.y = (float)(readRegisters(GYRO_ADDR, GYRO_OUT_Y_L, GYRO_OUT_Y_H));
  imu_msg.pose.position.z = (float)(readRegisters(GYRO_ADDR, GYRO_OUT_Z_L, GYRO_OUT_Z_H));
  imu_msg.pose.orientation.x = (float)(readRegisters(ACL_ADDR, ACL_DATAX_L, ACL_DATAX_H));
  imu_msg.pose.orientation.y = (float)(readRegisters(ACL_ADDR, ACL_DATAY_L, ACL_DATAY_H));
  imu_msg.pose.orientation.z = (float)(readRegisters(ACL_ADDR, ACL_DATAZ_L, ACL_DATAZ_H));
}

/*****************************************************************************/

void setup()
{
  Wire.begin();        // join i2c bus (address optional for master)
  
  Wire.beginTransmission(GYRO_ADDR);   // transmiting to gyro
  Wire.write(GYRO_CTRL_REG1_ADDR);     // sending control register address
  Wire.write(GYRO_CTRL_REG1_VALUE);    // sending control register data
  Wire.endTransmission();              // stop transmitting
  
  Wire.beginTransmission(ACL_ADDR);   // transmiting to acl
  Wire.write(ACL_POWER_CTL_ADDR);     // sending control register address
  Wire.write(ACL_POWER_CTL_VALUE);    // sending control register data
  Wire.endTransmission();             // stop transmitting
  
  Wire.beginTransmission(ACL_ADDR);   // transmiting to acl
  Wire.write(ACL_DATA_FORMAT_ADDR);     // sending control register address
  Wire.write(ACL_DATA_FORMAT_VALUE);    // sending control register data
  Wire.endTransmission();             // stop transmitting
  
  nh.initNode();
  nh.advertise(pub_imu);
}

/*****************************************************************************/

void loop()
{
  if ( (millis()-publish_timer) > 100){
    read_imu();
    imu_msg.header.stamp = nh.now();
    pub_imu.publish(&imu_msg);
    publish_timer = millis();
  }
  nh.spinOnce();
}
