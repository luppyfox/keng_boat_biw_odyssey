#define Serial SerialUSB
#define US_USBCON 1

#include <CytronMotorDriver.h>
#include <ArduinoHardware.h>
#include <ros.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float32.h>

CytronMD motor_r(PWM_DIR, 4, 3);
CytronMD motor_l(PWM_DIR, 6, 5);

int encoderL_A = 12; 
int encoderL_B = 13;
int encoderR_A = 10;
int encoderR_B = 11;

double total_pulse = 40120;

long prevT_L = 0;
long prevT_R = 0;

long posPrev_L = 0;
float vlFilt = 0;
float vlPrev = 0;
float vt_L = 0;
long posPrev_R = 0;
float vrFilt = 0;
float vrPrev = 0;
float vt_R = 0;

volatile long pulse_L = 0;
volatile long pulse_R = 0;
int pulsesChanged = 0;

float kp = 35;
float ki = 650;

float e_L = 0;
float e_R = 0;
float eintegral_L = 0;
float eintegral_R = 0;

ros::NodeHandle nh;

std_msgs::Int64 encL_msg;
std_msgs::Int64 encR_msg;

ros::Publisher EncL("Enc_L", &encL_msg);
ros::Publisher EncR("Enc_R", &encR_msg);

void CallBack_L(const std_msgs::Float32& vel_l) {
  vt_L = vel_l.data;
}
void CallBack_R(const std_msgs::Float32& vel_r) {
  vt_R = vel_r.data;
}

ros::Subscriber <std_msgs::Float32> Motor_L("/control_left_wheel/command", CallBack_L);
ros::Subscriber <std_msgs::Float32> Motor_R("/control_right_wheel/command", CallBack_R);

void setup() {
  nh.getHardware() -> setBaud(57600);
  nh.initNode();
  nh.subscribe(Motor_L);
  nh.subscribe(Motor_R);
  nh.advertise(EncL);
  nh.advertise(EncR);

  attachInterrupt(encoderL_A, PULSE_L_A_CHANGE, CHANGE);
  attachInterrupt(encoderL_B, PULSE_L_B_CHANGE, CHANGE);
  attachInterrupt(encoderR_A, PULSE_R_A_CHANGE, CHANGE);
  attachInterrupt(encoderR_B, PULSE_R_B_CHANGE, CHANGE);
}
void loop() {
  while(!nh.connected()){
    nh.spinOnce();
    motor_l.setSpeed(0);
    motor_r.setSpeed(0);
    e_L = 0;
    e_R = 0;
    eintegral_L = 0;
    eintegral_R = 0;
  }

  nh.spinOnce();

  if (pulsesChanged != 0) {
    pulsesChanged = 0;

    encL_msg.data = pulse_L;
    encR_msg.data = pulse_R;
    EncL.publish(&encR_msg);
    EncR.publish(&encR_msg);
  }

  int pos_L = 0;
  int pos_R = 0;
  noInterrupts();
  pos_L = pulse_L;
  pos_R = pulse_R;
  interrupts();

  long currT_L = micros();
  float deltaT_L = ((float)(currT_L - prevT_L)) / 1.0e6;
  float velocity_L = (pos_L - posPrev_L) / deltaT_L;
  posPrev_L = pos_L;
  prevT_L = currT_L;

  long currT_R = micros();
  float deltaT_R = ((float)(currT_R - prevT_R)) / 1.0e6;
  float velocity_R = (pos_R - posPrev_R) / deltaT_R;
  posPrev_R = pos_R;
  prevT_R = currT_R;

  double vl = velocity_L / total_pulse * 60.0;
  double vr = velocity_R / total_pulse * 60.0;

  vlFilt = 0.854 * vlFilt + 0.0728 * vl + 0.0728 * vlPrev;
  vlPrev = vl;
  vlFilt = 0.854 * vrFilt + 0.0728 * vr + 0.0728 * vrPrev;
  vrPrev = vr;

  e_L = vt_L - vlFilt;
  eintegral_L = eintegral_L + e_L * deltaT_L;
  e_R = vt_R - vlFilt;
  eintegral_R = eintegral_R + e_R * deltaT_R;

  float ul = kp * e_L + ki * eintegral_L;
  float ur = kp * e_R + ki * eintegral_R;

  int dir_L = 1;
  if (ul < 0) {
    dir_L = -1;
  }
  int pwr_L = ((int) fabs(ul))*dir_L;
  if (pwr_L > 255){
    pwr_L = 255;
  }
  if (pwr_L < -255) {
    pwr_L = -255;
  }

  int dir_R = 1;
  if (ur < 0) {
    dir_R = -1;
  }
  int pwr_R = ((int) fabs(ur))*dir_R;
  if (pwr_R > 255) {
    pwr_R = 255;
  }
  if (pwr_R < -255) {
    pwr_R = -255;
  }

  if (vt_L == 0) motor_l.setSpeed(0);
  else motor_l.setSpeed(pwr_L);
  if (vt_R == 0) motor_r.setSpeed(0);
  else motor_r.setSpeed(pwr_R);

  delay(1);
}

void PULSE_L_A_CHANGE() {
  if (digitalRead(encoderL_B) == 0) {
    if (digitalRead(encoderL_A) == 0) {pulse_L--;}
    else pulse_L++;
  }
  else {
    if (digitalRead(encoderL_A) == 0) {pulse_L--;}
    else pulse_L++;
  }
  pulsesChanged = 1;
}

void PULSE_L_B_CHANGE() {
  if (digitalRead(encoderL_A) == 0) {
    if (digitalRead(encoderL_B) == 0) {pulse_L--;}
    else pulse_L++;
  }
  else {
    if (digitalRead(encoderL_B) == 0) {pulse_L--;}
    else pulse_L++;
  }
  pulsesChanged = 1;
}

void PULSE_R_A_CHANGE() {
  if (digitalRead(encoderR_B) == 0) {
    if (digitalRead(encoderR_A) == 0) {pulse_R--;}
    else pulse_R++;
  }
  else {
    if (digitalRead(encoderR_A) == 0) {pulse_R--;}
    else pulse_R++;
  }
  pulsesChanged = 1;
}

void PULSE_R_B_CHANGE() {
  if (digitalRead(encoderR_B) == 0) {
    if (digitalRead(encoderR_A) == 0) {pulse_R--;}
    else pulse_L++;
  }
  else {
    if (digitalRead(encoderR_A) == 0) {pulse_R--;}
    else pulse_R++;
  }
  pulsesChanged = 1;
}
