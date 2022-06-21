#define Serial SerialUSB
#define USE_USBCON 1

#include <CytronMotorDriver.h>
#include <ArduinoHardware.h>
#include <ros.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Int32.h>


/*For manual PID adjust (Ctrl+F to jump)
 * set 
 * 1. Serial.begin(115200); 
 * 2. Set a target(Manual)
 * 3. monitor(for manual check)
 * 
 * unset
 * while (!nh.connected() )
*/

// Pins
CytronMD motor_r(PWM_DIR, 4, 3); // PWM 1=Pin 4, DIR1=Pin3
CytronMD motor_l(PWM_DIR, 6, 5); // PWM 2=Pin 6, DIR2=Pin5

int encoderL_A = 12;
int encoderL_B = 13;

int encoderR_A = 10;
int encoderR_B = 11;

double total_pulse = 40120;  // total pulses

// globals
int pwm_L = 0;
int pwm_R = 0;

// Use the "volatile" directive for variables
// used in an interrupt
volatile long pulse_L = 0;
volatile long pulse_R = 0;
int pulsesChanged = 0;

//create ros node 
ros::NodeHandle nh;

//config message variable
std_msgs::Int64 encL_msg; // message encoder of left wheel
std_msgs::Int64 encR_msg; // message encoder of right wheel

//call publisher
ros::Publisher EncL("enc_L", &encL_msg); // Publish encoder of left wheel
ros::Publisher EncR("enc_R", &encR_msg); // Publish encoder of right wheel

//callback
void CallBack_L(const std_msgs::Int32& vel_l) { pwm_L = vel_l.data; }
void CallBack_R(const std_msgs::Int32& vel_r) { pwm_R = vel_r.data; }

//call subscriber
ros::Subscriber <std_msgs::Int32> Motor_L("/pwm_L", CallBack_L); // Subscribe pwm for drive left motor
ros::Subscriber <std_msgs::Int32> Motor_R("/pwm_R", CallBack_R); // Subscribe pwm for drive right motor

void setup() {
//  Serial.begin(115200);
  
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

  while (!nh.connected() ){
    nh.spinOnce();
    motor_l.setSpeed(0);
    motor_r.setSpeed(0);  
  }

  nh.spinOnce();

  if (pulsesChanged != 0) {
    pulsesChanged = 0;

    encL_msg.data = pulse_L;
    encR_msg.data = pulse_R;
    EncL.publish( &encL_msg );
    EncR.publish( &encR_msg );
  }

//  setMotor
  motor_l.setSpeed(pwm_L);
  motor_r.setSpeed(pwm_R);
  
  delay(1);
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
  pulsesChanged = 1;
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
  pulsesChanged = 1;
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
  pulsesChanged = 1;
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
  pulsesChanged = 1;
}
