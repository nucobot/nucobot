#include <SPI.h>
#include <math.h>
#include <IMU.h>


IMU imu;

void setup() {
  Serial.begin(9600);  

  //As per APM standard code, stop the barometer from holding the SPI bus
  pinMode(40, OUTPUT);
  digitalWrite(40, HIGH);

  SPI.begin();  
  SPI.setClockDivider(SPI_CLOCK_DIV16); 

  SPI.setBitOrder(MSBFIRST); 
  SPI.setDataMode(SPI_MODE0);
  delay(100);
  
  pinMode(53, OUTPUT);

  imu.Init(53);  // configure chip
}

void loop()
{
  Serial.print("\tGyro X ");
  Serial.print(imu.GyroX());
  Serial.print("   ");
  Serial.print("\tGyro Y ");
  Serial.print(imu.GyroY());
  Serial.print("   ");
  Serial.print("\tGyro Z ");  
  Serial.print(imu.GyroZ());  
  
  Serial.print("\t\tAcc X ");
  Serial.print(imu.AcceX());
  Serial.print("   ");
  Serial.print("\tAcc Y ");
  Serial.print(imu.AcceY());
  Serial.print("   ");
  Serial.print("\tAcc Z ");  
  Serial.println(imu.AcceZ());  
}

