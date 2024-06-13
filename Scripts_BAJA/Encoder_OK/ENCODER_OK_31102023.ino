#include <Servo.h>
#include <ArduinoHardware.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>

ros::NodeHandle nh;
std_msgs::Float64 EncoderL;
ros::Publisher Encoder_Left("EncoderLeft", &EncoderL);

std_msgs::Float64 EncoderR;
ros::Publisher Encoder_Right("EncoderRight", &EncoderR);

int LeituraBin;
float EncoderAnterior, LeituraEncoder, ValorEncoder, LeituraGray;

int LeituraBin_R;
float EncoderAnterior_R, LeituraEncoder_R, ValorEncoder_R, LeituraGray_R;

Servo RQTservo;

float CTRL_Velocidade, AnguloServo;
float Velocidade_linear, Velocidade_angular;

int FrentePWM1 = 4;
int ReversoPWM1 = 5;
int FrentePWM2 = 6;
int ReversoPWM2 = 7;



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
  nh.advertise(Encoder_Left);
  nh.advertise(Encoder_Right);

  pinMode(51, INPUT);
  pinMode(49, INPUT);
  pinMode(47, INPUT);
  pinMode(45, INPUT);

  pinMode(50, INPUT);
  pinMode(48, INPUT);
  pinMode(46, INPUT);
  pinMode(44, INPUT);
}
void loop() {
  LeituraBin_R = (!digitalRead(51) & 0x1) | ((!digitalRead(49) & 0x1) << 1) | ((!digitalRead(47) & 0x1) << 2) | ((!digitalRead(45) & 0x1) << 3);
  LeituraBin = (!digitalRead(50) & 0x1) | ((!digitalRead(48) & 0x1) << 1) | ((!digitalRead(46) & 0x1) << 2) | ((!digitalRead(44) & 0x1) << 3);
  bin_gray();
  velocidade();
  velocidade_R();

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


//###################### CONVERSOR DE BIN PARA GRAY
void bin_gray() {
  switch (LeituraBin) {
    case 0: LeituraGray = 0; break;
    case 1: LeituraGray = 1; break;
    case 2: LeituraGray = 9; break;
    case 3: LeituraGray = 8; break;
    case 4: LeituraGray = 13; break;
    case 5: LeituraGray = 4; break;
    case 6: LeituraGray = 12; break;
    case 7: LeituraGray = 5; break;
    case 8: LeituraGray = 15; break;
    case 9: LeituraGray = 2; break;
    case 10: LeituraGray = 10; break;
    case 11: LeituraGray = 7; break;
    case 12: LeituraGray = 14; break;
    case 13: LeituraGray = 3; break;
    case 14: LeituraGray = 11; break;
    case 15: LeituraGray = 6; break;
  }

  switch (LeituraBin_R) {
    case 0: LeituraGray_R = 0; break;
    case 1: LeituraGray_R = 1; break;
    case 2: LeituraGray_R = 9; break;
    case 3: LeituraGray_R = 8; break;
    case 4: LeituraGray_R = 13; break;
    case 5: LeituraGray_R = 4; break;
    case 6: LeituraGray_R = 12; break;
    case 7: LeituraGray_R = 5; break;
    case 8: LeituraGray_R = 15; break;
    case 9: LeituraGray_R = 2; break;
    case 10: LeituraGray_R = 10; break;
    case 11: LeituraGray_R = 7; break;
    case 12: LeituraGray_R = 14; break;
    case 13: LeituraGray_R = 3; break;
    case 14: LeituraGray_R = 11; break;
    case 15: LeituraGray_R = 6; break;
  }
}

// CALCULO DE VELOCIDADE
void velocidade() {
  LeituraEncoder = (EncoderAnterior - LeituraGray);
  if (LeituraEncoder == 1 || LeituraEncoder == -15) {  // Giro no sentido horário
    ValorEncoder++;
  } else if (LeituraEncoder == -1 || LeituraEncoder == 15) {  // Giro no sentido anti-horário
    ValorEncoder--;
  } else {
    ValorEncoder = ValorEncoder + LeituraEncoder;
  }
  EncoderAnterior = LeituraGray;
}

void velocidade_R() {
  LeituraEncoder_R = (EncoderAnterior_R - LeituraGray_R);
  if (LeituraEncoder_R == 1 || LeituraEncoder_R == -15) {  // Giro no sentido horário
    ValorEncoder_R++;
  } else if (LeituraEncoder_R == -1 || LeituraEncoder_R == 15) {  // Giro no sentido anti-horário
    ValorEncoder_R--;
  } else {
    ValorEncoder_R = ValorEncoder_R + LeituraEncoder_R;
  }
  EncoderAnterior_R = LeituraGray_R;
}
