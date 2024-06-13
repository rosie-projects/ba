#include <Servo.h>
#include <ArduinoHardware.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>

Servo RQTservo;

float CTRL_Velocidade, AnguloServo;
float Velocidade_linear, Velocidade_angular;

int FrentePWM1 = 4;
int ReversoPWM1 = 5;
int FrentePWM2 = 6;
int ReversoPWM2 = 7;

ros::NodeHandle nh;
void CTRL_Baja(const geometry_msgs::Twist& vel) {
  Velocidade_linear = vel.linear.x;
  Velocidade_angular = vel.angular.z;
}

ros::Subscriber<geometry_msgs::Twist> sub_CTRL("cmd_vel", &CTRL_Baja);

void setup() {
  RQTservo.attach(8);
  pinMode(FrentePWM2, OUTPUT);
  pinMode(ReversoPWM2, OUTPUT);
  nh.initNode();
  nh.subscribe(sub_CTRL);
  
}
void loop() {
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
    // reverse rotation
    int reversePWM = -(CTRL_Velocidade - 511) / 2;
    analogWrite(FrentePWM1, 0);
    analogWrite(ReversoPWM1, reversePWM);
    analogWrite(FrentePWM2, 0);
    analogWrite(ReversoPWM2, reversePWM);

  } else {
    // forward rotation
    int forwardPWM = (CTRL_Velocidade - 512) / 2;
    analogWrite(FrentePWM1, forwardPWM);
    analogWrite(ReversoPWM1, 0);
    analogWrite(FrentePWM2, forwardPWM);
    analogWrite(ReversoPWM2, 0);
  }

  nh.spinOnce();
}
