
#include <Servo.h>
#include <ArduinoHardware.h>
#include <geometry_msgs/Twist.h>
#include <TinyGPS++.h>
#include "GY_85.h"
#include <NewPing.h>
#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Range.h>

Servo RQTservo;
GY_85 GY85;
TinyGPSPlus gps;
ros::NodeHandle nh;

float Velocidade_linear, Velocidade_angular;
void CTRL_Baja(const geometry_msgs::Twist& vel) {
  Velocidade_linear = vel.linear.x;
  Velocidade_angular = vel.angular.z;
}
ros::Subscriber<geometry_msgs::Twist> sub_CTRL("cmd_vel", &CTRL_Baja);

std_msgs::Float64 EncoderL;
ros::Publisher Encoder_Left("EncoderLeft", &EncoderL);

std_msgs::Float64 EncoderR;
ros::Publisher Encoder_Right("EncoderRight", &EncoderR);

sensor_msgs::NavSatFix GPS;
ros::Publisher pub_GPS("GPS", &GPS);

sensor_msgs::Imu IMU;
ros::Publisher pub_IMU("IMU", &IMU);

sensor_msgs::Range Ultrason0;
ros::Publisher pub_RANGE0("Sonar0", &Ultrason0);

sensor_msgs::Range Ultrason1;
ros::Publisher pub_RANGE1("Sonar1", &Ultrason1);

sensor_msgs::Range Ultrason2;
ros::Publisher pub_RANGE2("Sonar2", &Ultrason2);

sensor_msgs::Range Ultrason3;
ros::Publisher pub_RANGE3("Sonar3", &Ultrason3);

sensor_msgs::Range Ultrason4;
ros::Publisher pub_RANGE4("Sonar4", &Ultrason4);

sensor_msgs::Range Ultrason5;
ros::Publisher pub_RANGE5("Sonar5", &Ultrason5);

sensor_msgs::Range Ultrason6;
ros::Publisher pub_RANGE6("Sonar6", &Ultrason6);

sensor_msgs::Range Ultrason7;
ros::Publisher pub_RANGE7("Sonar7", &Ultrason7);

//################################ VARIAVEIS SERVO E PONTE-H

float CTRL_Velocidade;
int AnguloServo;
int FrentePWM1 = 4;
int ReversoPWM1 = 5;
int FrentePWM2 = 6;
int ReversoPWM2 = 7;

//################################ VARIAVEIS IMU/GPS

float lat, lon, alt;
float speed;

int Busx, Busy, Busz, Acelx, Acely, Acelz, Girx, Giry, Girz, Girt;

// ############################### VARIAVEIS ENCODER

volatile int LeituraBin;
volatile int EncoderAnterior, LeituraEncoder, ValorEncoder, LeituraGray;
volatile int LeituraBin_R;
volatile int EncoderAnterior_R, LeituraEncoder_R, ValorEncoder_R, LeituraGray_R;

//################################# VARIAVEIS SONAR

#define TRIG_1 22
#define ECHO_1 23
#define TRIG_2 24
#define ECHO_2 25
#define TRIG_3 26
#define ECHO_3 27
#define TRIG_4 28
#define ECHO_4 29
#define TRIG_5 30
#define ECHO_5 31
#define TRIG_6 32
#define ECHO_6 33
#define TRIG_7 34
#define ECHO_7 35
#define TRIG_8 36
#define ECHO_8 37

NewPing sonar1(TRIG_1, ECHO_1, 400);  // 400 cm
NewPing sonar2(TRIG_2, ECHO_2, 400);
NewPing sonar3(TRIG_3, ECHO_3, 400);
NewPing sonar4(TRIG_4, ECHO_4, 400);
NewPing sonar5(TRIG_5, ECHO_5, 400);
NewPing sonar6(TRIG_6, ECHO_6, 400);
NewPing sonar7(TRIG_7, ECHO_7, 400);
NewPing sonar8(TRIG_8, ECHO_8, 400);


int distancia0 = 0;
int distancia1 = 0;
int distancia2 = 0;
int distancia3 = 0;
int distancia4 = 0;
int distancia5 = 0;
int distancia6 = 0;
int distancia7 = 0;

void setup() {

  RQTservo.attach(8);  //SERVO E PONTE-H
  pinMode(FrentePWM2, OUTPUT);
  pinMode(ReversoPWM2, OUTPUT);

  pinMode(18, INPUT);  // ENCODER LEFT
  pinMode(49, INPUT);
  pinMode(47, INPUT);
  pinMode(45, INPUT);

  pinMode(19, INPUT);  // ENCODER RIGHT
  pinMode(48, INPUT);
  pinMode(46, INPUT);
  pinMode(44, INPUT);


  Serial.begin(9600);
  Serial3.begin(9600);
  GY85.init();

  nh.initNode();
  nh.advertise(pub_IMU);
  nh.advertise(pub_GPS);
  nh.advertise(Encoder_Left);
  nh.advertise(Encoder_Right);
  nh.advertise(pub_RANGE0);
  nh.advertise(pub_RANGE1);
  nh.advertise(pub_RANGE2);
  nh.advertise(pub_RANGE3);
  nh.advertise(pub_RANGE4);
  nh.advertise(pub_RANGE5);
  nh.advertise(pub_RANGE6);
  nh.advertise(pub_RANGE7);
  nh.subscribe(sub_CTRL);
}

void loop() {




  attachInterrupt(digitalPinToInterrupt(18), bin_gray, CHANGE);
  attachInterrupt(digitalPinToInterrupt(19), bin_gray, CHANGE);

  bin_gray();


  //======== Sonar

  distancia0 = sonar1.ping_cm();
  distancia1 = sonar2.ping_cm();
  distancia2 = sonar3.ping_cm();
  distancia3 = sonar4.ping_cm();
  distancia4 = sonar5.ping_cm();
  distancia5 = sonar6.ping_cm();
  distancia6 = sonar7.ping_cm();
  distancia7 = sonar8.ping_cm();

  //======== Encoder


  EncoderL.data = ValorEncoder;
  Encoder_Left.publish(&EncoderL);
  EncoderR.data = ValorEncoder_R;
  Encoder_Right.publish(&EncoderR);
  nh.spinOnce();

  //======== Direção
  AnguloServo = map(Velocidade_angular, -10, 10, 0, 180);
  RQTservo.write(AnguloServo);

  //======== Deslocamento
  if (Velocidade_linear == 0) {
    CTRL_Velocidade = 512;
  } else {
    CTRL_Velocidade = (Velocidade_linear * 512) + 512;
  }

  if (CTRL_Velocidade < 512) {
    // Roda para Trás
    int reversePWM = -(CTRL_Velocidade - 511) / 2;
    analogWrite(FrentePWM1, 0);
    analogWrite(ReversoPWM1, reversePWM);
    analogWrite(FrentePWM2, 0);
    analogWrite(ReversoPWM2, reversePWM);

  } else {
    // Roda para Frente
    int forwardPWM = (CTRL_Velocidade - 512) / 2;
    analogWrite(FrentePWM1, forwardPWM);
    analogWrite(ReversoPWM1, 0);
    analogWrite(FrentePWM2, forwardPWM);
    analogWrite(ReversoPWM2, 0);
  }

  //======== GPS
  while (Serial3.available()) {
    if (gps.encode(Serial3.read())) {
      lat = gps.location.lat();
      lon = gps.location.lng();
      GPS.latitude = lat;
      GPS.longitude = lon;
      pub_GPS.publish(&GPS);
      Serial.print("lat : ");
      Serial.println(lat);
    }
  }

  //======== IMU
  int* BusDADOS = GY85.readFromCompass();
  Busx = GY85.compass_x(BusDADOS);
  Busy = GY85.compass_y(BusDADOS);
  Busz = GY85.compass_z(BusDADOS);
  int* AcelDADOS = GY85.readFromAccelerometer();
  Acelx = GY85.accelerometer_x(AcelDADOS);
  Acely = GY85.accelerometer_y(AcelDADOS);
  Acelz = GY85.accelerometer_z(AcelDADOS);
  float* GirDADOS = GY85.readGyro();
  Girx = GY85.gyro_x(GirDADOS);
  Giry = GY85.gyro_y(GirDADOS);
  Girz = GY85.gyro_z(GirDADOS);
  Girt = GY85.temp(GirDADOS);



  IMU.orientation.x = Busx;
  IMU.orientation.y = Busy;
  IMU.orientation.z = Busz;
  IMU.linear_acceleration.x = Acelx;
  IMU.linear_acceleration.y = Acely;
  IMU.linear_acceleration.z = Acelz;
  IMU.angular_velocity.x = Girx;
  IMU.angular_velocity.y = Giry;
  IMU.angular_velocity.z = Girz;
  Ultrason0.range = distancia0;
  Ultrason1.range = distancia1;
  Ultrason2.range = distancia2;
  Ultrason3.range = distancia3;
  Ultrason4.range = distancia4;
  Ultrason5.range = distancia5;
  Ultrason6.range = distancia6;
  Ultrason7.range = distancia7;
  pub_IMU.publish(&IMU);
  pub_RANGE0.publish(&Ultrason0);
  pub_RANGE1.publish(&Ultrason1);
  pub_RANGE2.publish(&Ultrason2);
  pub_RANGE3.publish(&Ultrason3);
  pub_RANGE4.publish(&Ultrason4);
  pub_RANGE5.publish(&Ultrason5);
  pub_RANGE6.publish(&Ultrason6);
  pub_RANGE7.publish(&Ultrason7);
  nh.spinOnce();
}
//CONVERSOR DE BIN PARA GRAY
void bin_gray() {

  LeituraBin = (!digitalRead(18) & 0x1) | ((!digitalRead(49) & 0x1) << 1) | ((!digitalRead(47) & 0x1) << 2) | ((!digitalRead(45) & 0x1) << 3);
  LeituraBin_R = (!digitalRead(19) & 0x1) | ((!digitalRead(48) & 0x1) << 1) | ((!digitalRead(46) & 0x1) << 2) | ((!digitalRead(44) & 0x1) << 3);


  switch (LeituraBin) {
    case 0: LeituraGray = 0; break;
    case 1: LeituraGray = 1; break;
    case 2: LeituraGray = 3; break;
    case 3: LeituraGray = 2; break;
    case 4: LeituraGray = 7; break;
    case 5: LeituraGray = 6; break;
    case 6: LeituraGray = 4; break;
    case 7: LeituraGray = 5; break;
    case 8: LeituraGray = 15; break;
    case 9: LeituraGray = 14; break;
    case 10: LeituraGray = 12; break;
    case 11: LeituraGray = 13; break;
    case 12: LeituraGray = 8; break;
    case 13: LeituraGray = 9; break;
    case 14: LeituraGray = 11; break;
    case 15: LeituraGray = 10; break;
  }

  switch (LeituraBin_R) {
    case 0: LeituraGray_R = 0; break;
    case 1: LeituraGray_R = 1; break;
    case 2: LeituraGray_R = 3; break;
    case 3: LeituraGray_R = 2; break;
    case 4: LeituraGray_R = 7; break;
    case 5: LeituraGray_R = 6; break;
    case 6: LeituraGray_R = 4; break;
    case 7: LeituraGray_R = 5; break;
    case 8: LeituraGray_R = 15; break;
    case 9: LeituraGray_R = 14; break;
    case 10: LeituraGray_R = 12; break;
    case 11: LeituraGray_R = 13; break;
    case 12: LeituraGray_R = 8; break;
    case 13: LeituraGray_R = 9; break;
    case 14: LeituraGray_R = 11; break;
    case 15: LeituraGray_R = 10; break;
  }



  // CONTADOR DE PULSO

  LeituraEncoder = (LeituraGray - EncoderAnterior);
  switch (LeituraEncoder) {
    case 1: ValorEncoder++; break;
    case -15: ValorEncoder++; break;
    case -1: ValorEncoder--; break;
    case 15: ValorEncoder--; break;
    case 2: ValorEncoder = ValorEncoder + 2; break;
    case -2: ValorEncoder = ValorEncoder - 2; break;
  }

  EncoderAnterior = LeituraGray;

  LeituraEncoder_R = (LeituraGray_R - EncoderAnterior_R);
  switch (LeituraEncoder_R) {
    case 1: ValorEncoder_R++; break;
    case -15: ValorEncoder_R++; break;
    case -1: ValorEncoder_R--; break;
    case 15: ValorEncoder_R--; break;
    case 2: ValorEncoder_R = ValorEncoder_R + 2; break;
    case -2: ValorEncoder_R = ValorEncoder_R - 2; break;
  }

  EncoderAnterior_R = LeituraGray_R;
}