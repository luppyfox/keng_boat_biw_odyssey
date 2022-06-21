// request code 
// [-255,255]: rpm
// 300: goingUp
// 400: goingDown
// 999: reset

#define Serial SerialUSB
#define USE_USBCON 1

#include "CytronMotorDriver.h"

CytronMD motorR(PWM_DIR, 4, 3); // PWM 1=Pin 4, DIR1=Pin3
CytronMD motorL(PWM_DIR, 6, 5); // PWM 2=Pin 6, DIR2=Pin5
int encoderL_A = 10;
int encoderL_B = 11;
int encoderR_A = 12;
int encoderR_B = 13;

volatile long pulse_L = 0; 
int pulsesChanged_L = 0;     
volatile long pulse_R = 0;     
int pulsesChanged_R = 0;                     

#define total 40120  // total pulses

String input;
int motorSpeedL;             
int motorSpeedR;

void setup(){

  Serial.begin(115200);
  Serial.setTimeout(1);
  
  pinMode(encoderL_A, INPUT);
  pinMode(encoderL_B, INPUT);
  pinMode(encoderR_A, INPUT);
  pinMode(encoderR_B, INPUT);

  attachInterrupt(encoderL_A, PULSE_L_A_CHANGE, CHANGE);
  attachInterrupt(encoderL_B, PULSE_L_B_CHANGE, CHANGE);
  attachInterrupt(encoderR_A, PULSE_R_A_CHANGE, CHANGE);
  attachInterrupt(encoderR_B, PULSE_R_B_CHANGE, CHANGE);
}

void loop(){
  
  input = Serial.readString();
  for (int i = 0; i < input.length(); i++) {
    if (input.substring(i, i + 1) == ",") {
      motorSpeedL = input.substring(0, i).toInt();
      motorSpeedR = input.substring(i + 1).toInt();
      break;
    }
  }

  if (motorSpeedL == 999 || motorSpeedR == 999)
  { 
    reset(); 
  }
  else{ 
    motorL.setSpeed(motorSpeedL);
    motorR.setSpeed(motorSpeedR);
  }

  if (pulsesChanged_L != 0 || pulsesChanged_R != 0) {
    Serial.println(String(pulse_L)+' '+String(pulsesChanged_L)+' '+String(pulse_R)+' '+String(pulsesChanged_R ));
    pulsesChanged_L = 0;
    pulsesChanged_R = 0;
  }
  
}

void reset(){
  pulse_L = 0;
  pulse_R = 0;
  motorSpeedL = 0;
  motorSpeedR = 0;
}

void PULSE_L_A_CHANGE() {
  if ( digitalRead(encoderL_B) == 0 ) {
    if ( digitalRead(encoderL_A) == 0 ) {
      // A fell, B is low
      pulse_L--; // moving reverse
    } else {
      // A rose, B is low
      pulse_L++; // moving forward
    }
  } else {
    if ( digitalRead(encoderL_A) == 0 ) {
      // B fell, A is high
      pulse_L++; // moving reverse
    } else {
      // B rose, A is high
      pulse_L--; // moving forward
    }
  }
  pulsesChanged_L = 1;
}


void PULSE_L_B_CHANGE() {
  if ( digitalRead(encoderL_A) == 0 ) {
    if ( digitalRead(encoderL_B) == 0 ) {
      // B fell, A is low
      pulse_L++; // moving forward
    } else {
      // B rose, A is low
      pulse_L--; // moving reverse
    }
  } else {
    if ( digitalRead(encoderL_B) == 0 ) {
      // B fell, A is high
      pulse_L--; // moving reverse
    } else {
      // B rose, A is high
      pulse_L++; // moving forward
    }
  }
  pulsesChanged_L = 1;
}

void PULSE_R_A_CHANGE() {
  if ( digitalRead(encoderR_B) == 0 ) {
    if ( digitalRead(encoderR_A) == 0 ) {
      // A fell, B is low
      pulse_R--; // moving reverse
    } else {
      // A rose, B is low
      pulse_R++; // moving forward
    }
  } else {
    if ( digitalRead(encoderR_A) == 0 ) {
      // B fell, A is high
      pulse_R++; // moving reverse
    } else {
      // B rose, A is high
      pulse_R--; // moving forward
    }
  }
  pulsesChanged_R = 1;
}

void PULSE_R_B_CHANGE() {
  if ( digitalRead(encoderR_A) == 0 ) {
    if ( digitalRead(encoderR_B) == 0 ) {
      // B fell, A is low
      pulse_R++; // moving forward
    } else {
      // B rose, A is low
      pulse_R--; // moving reverse
    }
  } else {
    if ( digitalRead(encoderR_B) == 0 ) {
      // B fell, A is high
      pulse_R--; // moving reverse
    } else {
      // B rose, A is high
      pulse_R++; // moving forward
    }
  }
  pulsesChanged_R = 1;
}
