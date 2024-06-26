#include "GY_85.h"
#include <ros.h>
#include <sensor_msgs/Imu.h>

GY_85 GY85;

ros::NodeHandle nh;
sensor_msgs::Imu IMU;  // variavel
ros::Publisher pub_IMU("IMU", &IMU);

void setup() {
 // Serial.begin(57600);
  GY85.init();

  nh.initNode();
  nh.advertise(pub_IMU);

}

void loop() {

    int* BusDADOS = GY85.readFromCompass();
    int Busx = GY85.compass_x(BusDADOS);
    int Busy = GY85.compass_y(BusDADOS);
    int Busz = GY85.compass_z(BusDADOS);
    
    int* AcelDADOS = GY85.readFromAccelerometer();
    int Acelx = GY85.accelerometer_x(AcelDADOS);
    int Acely = GY85.accelerometer_y(AcelDADOS);
    int Acelz = GY85.accelerometer_z(AcelDADOS);
    
    float* GirDADOS = GY85.readGyro();
    float Girx = GY85.gyro_x(GirDADOS);
    float Giry = GY85.gyro_y(GirDADOS);
    float Girz = GY85.gyro_z(GirDADOS);
    float Girt = GY85.temp(GirDADOS);


  IMU.orientation.x = Busx;
  IMU.orientation.y = Busy;
  IMU.orientation.z = Busz;
  IMU.linear_acceleration.x = Acelx;
  IMU.linear_acceleration.y = Acely;
  IMU.linear_acceleration.z = Acelz;
  IMU.angular_velocity.x = Girx;
  IMU.angular_velocity.y = Giry;
  IMU.angular_velocity.z = Girz;
  pub_IMU.publish(&IMU);
  nh.spinOnce();

}

