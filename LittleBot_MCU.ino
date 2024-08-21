/* ---------LittleBot MCU--------
- reads a Joint State message on /mcu_cmd_motor
- applies PID velocity control to 4 motors with _kP, _kI, _kD, _Max_ITerm
- reports wheel velocity and position travelled between last publish on \enc_feedback
- all terms are angular position, angular velocity: Mx_theta, Mx_w
*/
#include <micro_ros_arduino.h>
#include <Adafruit_NeoPixel.h>
#include <Encoder.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/joint_state.h>
#include <std_msgs/msg/int32.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

#define LED_PIN 21
#define RC_CH1 0
#define RC_CH2 1
#define RC_CH3 22
#define RC_CH4 23
#define M1_IN_A 2
#define M1_IN_B 3
#define M2_IN_A 4
#define M2_IN_B 5
#define M3_IN_A 6
#define M3_IN_B 7
#define M4_IN_A 8
#define M4_IN_B 9
#define M1_ENC_A 10
#define M1_ENC_B 11
#define M2_ENC_A 12
#define M2_ENC_B 13
#define M3_ENC_A 14
#define M3_ENC_B 15
#define M4_ENC_A 16
#define M4_ENC_B 17

#define ticksPerRev 1024 // encoder ticks per revolution
#define sampleTimeROS 100 // 50ms, 20Hz
#define sampleTimePub 100 // 50ms, 20Hz
#define sampleTimePID 10 // 10ms, 100Hz

#define w_min -15.5  // Vbat (15) [V] * kV (207) [rpm/V] / gear_ratio (18) * 2Pi [rad/rot] / 60 [s/min]
#define w_max 15.5
#define PWM_freq 30000

rcl_subscription_t subscriber_cmd_motor;
rcl_publisher_t publisher_enc;
rcl_publisher_t publisher_2;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

sensor_msgs__msg__JointState cmd_msg;
sensor_msgs__msg__JointState enc_msg;
std_msgs__msg__Int32 PIDrate_msg;
Adafruit_NeoPixel strip = Adafruit_NeoPixel(48, LED_PIN, NEO_GRB + NEO_KHZ800);
long unsigned int nextUpdateROS = 0;
long unsigned int nextUpdatePID = 0;
long unsigned int nextUpdatePub = 0;
long unsigned int lastUpdatePID = 0;
long unsigned int PIDtimer = 0;

//encoder ticks
double M1_ticks = 0;
double M2_ticks = 0;
double M3_ticks = 0;
double M4_ticks = 0;
//The measured angular rate: w [rad/s]
float M1_w = 0;
float M2_w = 0;
float M3_w = 0;
float M4_w = 0;
//The accumulated rotation in publish period [rad]
double M1_theta_pub=0;
double M2_theta_pub=0;
double M3_theta_pub=0;
double M4_theta_pub=0;
//The target angular rate: w [rad/s]
double M1_w_SetPoint = 0;
double M2_w_SetPoint = 0;
double M3_w_SetPoint = 0;
double M4_w_SetPoint = 0;
// previous speed, used by the PID controller [rad/s]
float M1_w_LastSpeed = 0;
float M2_w_LastSpeed = 0;
float M3_w_LastSpeed = 0;
float M4_w_LastSpeed = 0;
//motor control output
short int M1_Pwm = 0;
short int M2_Pwm = 0;
short int M3_Pwm = 0;
short int M4_Pwm = 0;
//Encoders
Encoder M1_Enc(M1_ENC_A, M1_ENC_B);
Encoder M2_Enc(M2_ENC_A, M2_ENC_B);
Encoder M3_Enc(M3_ENC_A, M3_ENC_B);
Encoder M4_Enc(M4_ENC_A, M4_ENC_B);


float ch1Value, ch2Value, ch3Value, ch4Value;
int test_count = 0;

//PID Params
double _kP = 3.2;
double _kI = 0.011;//0.0002;
double _kD = 0.0022;//0.00000005;
double Max_ITerm = 5.0;
double M1_ITerm;
double M2_ITerm;
double M3_ITerm;
double M4_ITerm;

void error_loop(){
   while(1){
    for(uint16_t i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, strip.Color(abs(155), abs(100), 0));
    }
  strip.show();
  delay(1500);
    for(uint16_t i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, strip.Color(0, 0, 0));
    }
  strip.show();
  delay(1500);
  }
}

int limitPWM(int pwm)
{
  if (pwm < -255)
    pwm = -255;

  if (pwm > 255)
    pwm = 255;
  return pwm;
}

long readChannel(int channelInput, int minLimit, int maxLimit, int defaultValue){
  int ch = pulseIn(channelInput, HIGH, 30000);
  if (ch < 100) return defaultValue;
  return map(ch, 1100, 1950, minLimit*1000.0, maxLimit*1000.0);  //scaled up by 1000 for higher resolution of integer
}

void subscription_callback_cmd_motor(const void * msgin)
{  
  const sensor_msgs__msg__JointState * msg = (const sensor_msgs__msg__JointState *)msgin;
  M1_w_SetPoint = -(double)msg->velocity.data[0];
  M2_w_SetPoint = -(double)msg->velocity.data[1];
  M3_w_SetPoint = (double)msg->velocity.data[2];
  M4_w_SetPoint = (double)msg->velocity.data[3];
}
void adjustMotorPWM(int M1_Pwm, int M2_Pwm, int M3_Pwm, int M4_Pwm)
{
  if (M1_Pwm+M2_Pwm == 0) {
      analogWrite(M1_IN_A, 0);
      analogWrite(M1_IN_B, 0);
      analogWrite(M2_IN_A, 0);
      analogWrite(M2_IN_B, 0);
      //Serial.println(M1_Pwm);
    }
  else if (M1_Pwm+M2_Pwm > 0)
     {
      analogWrite(M1_IN_A, abs(M1_Pwm));
      analogWrite(M1_IN_B, 0);
      analogWrite(M2_IN_A, abs(M2_Pwm));
      analogWrite(M2_IN_B, 0);
      //Serial.print("forwards: ");Serial.println(M1_Pwm);
     }
  else if (M1_Pwm+M2_Pwm < 0)
     {
     analogWrite(M1_IN_A, 0);
     analogWrite(M1_IN_B, abs(M1_Pwm));
     analogWrite(M2_IN_A, 0);
     analogWrite(M2_IN_B, abs(M2_Pwm));
     //Serial.print("backwards: ");Serial.println(M1_Pwm);
    }
    // M2_______
    if (M3_Pwm+M4_Pwm == 0) {
      analogWrite(M3_IN_A, 0);
      analogWrite(M3_IN_B, 0);
      analogWrite(M4_IN_A, 0);
      analogWrite(M4_IN_B, 0);
      //Serial.println(M2_Pwm);
    }
  else if (M3_Pwm+M4_Pwm > 0)
     {
      analogWrite(M3_IN_A, abs(M3_Pwm));
      analogWrite(M3_IN_B, 0);
      analogWrite(M4_IN_A, abs(M4_Pwm));
      analogWrite(M4_IN_B, 0);
      //Serial.print("forwards: ");Serial.println(M2_Pwm);
     }
  else if (M3_Pwm+M4_Pwm < 0)
     {
     analogWrite(M3_IN_A, 0);
     analogWrite(M3_IN_B, abs(M3_Pwm));
     analogWrite(M4_IN_A, 0);
     analogWrite(M4_IN_B, abs(M4_Pwm));
     //Serial.print("backwards: ");Serial.println(M2_Pwm);
    } 
}

int motorPID(double M_w_SetPoint, double M_w, float& M_w_LastSpeed, int M_Pwm, double& M_ITerm)
{
  double error = M_w_SetPoint - M_w;  // calculate error
  M_ITerm += (_kI * (double)error); // calculate integral term
  if (M_ITerm > Max_ITerm) {M_ITerm = Max_ITerm;}
  else if (M_ITerm < -Max_ITerm) {M_ITerm = -Max_ITerm;}
  double dInput = M_w - M_w_LastSpeed; // calculate derivative
  float adjustment = (_kP * (double)error) + M_ITerm - (_kD * dInput);
  M_Pwm += adjustment;
  M_Pwm = limitPWM(M_Pwm);
  M_w_LastSpeed = M_w;
  // Serial.print("SetPoint:" );Serial.print(  M_w_SetPoint);Serial.print("  M_w:" );Serial.print(M_w);Serial.print("  error:" );Serial.print(error);Serial.print(" M_Pwm");Serial.println(M_Pwm);
  return M_Pwm;
}
void setup() {
  pinMode(RC_CH1, INPUT);
  pinMode(RC_CH2, INPUT);
  pinMode(RC_CH3, INPUT);
  pinMode(RC_CH4, INPUT);
  pinMode(M1_IN_A, OUTPUT); 
  pinMode(M1_IN_B, OUTPUT);
  pinMode(M2_IN_A, OUTPUT); 
  pinMode(M2_IN_B, OUTPUT);
  pinMode(M3_IN_A, OUTPUT); 
  pinMode(M3_IN_B, OUTPUT);
  pinMode(M4_IN_A, OUTPUT); 
  pinMode(M4_IN_B, OUTPUT);
  //remove annoying noise by changing the PWM freq above audible. 
  analogWriteFrequency(M1_IN_A, PWM_freq);
  analogWriteFrequency(M1_IN_B, PWM_freq);
  analogWriteFrequency(M2_IN_A, PWM_freq);
  analogWriteFrequency(M1_IN_A, PWM_freq);
  analogWriteFrequency(M2_IN_B, PWM_freq);
  analogWriteFrequency(M3_IN_A, PWM_freq);
  analogWriteFrequency(M3_IN_B, PWM_freq);
  analogWriteFrequency(M4_IN_A, PWM_freq);
  analogWriteFrequency(M4_IN_B, PWM_freq);
  set_microros_transports();
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  
  
  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "littlebot_mcu_node", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber_cmd_motor,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),"mcu_cmd_motor"));

  RCCHECK(rclc_publisher_init_default(
    &publisher_enc,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),"enc_feedback"));

  RCCHECK(rclc_publisher_init_default(
    &publisher_2,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),"test_int_pub"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_cmd_motor, &cmd_msg, &subscription_callback_cmd_motor, ON_NEW_DATA));

  // Turn on the LEDs
  strip.begin();
  for(uint16_t i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, strip.Color(0, 120, 0));
  }
  strip.show();
//---------------cmd_msg memory alloc---------------
  cmd_msg.name.capacity = 4;
  cmd_msg.name.data = (rosidl_runtime_c__String*) malloc(cmd_msg.name.capacity *sizeof(rosidl_runtime_c__String));
  cmd_msg.name.size = 0;

  cmd_msg.position.capacity = 4; // 4 JOINTS!
  cmd_msg.position.data = (double*) malloc(cmd_msg.position.capacity * sizeof(double));
  cmd_msg.position.size = 0;

  cmd_msg.velocity.capacity = 4; // 4 JOINTS!
  cmd_msg.velocity.data = (double*) malloc(cmd_msg.velocity.capacity * sizeof(double));
  cmd_msg.velocity.size = 0;

  cmd_msg.effort.capacity = 4; // 4 JOINTS!
  cmd_msg.effort.data = (double*) malloc(cmd_msg.effort.capacity * sizeof(double));
  cmd_msg.effort.size = 0;
  //-----------------enc_msg memory alloc--------------
  enc_msg.name.capacity = 4;
  enc_msg.name.data = (rosidl_runtime_c__String*) malloc(enc_msg.name.capacity *sizeof(rosidl_runtime_c__String));
  enc_msg.name.size = 0;

  enc_msg.position.capacity = 4; // 4 JOINTS!
  enc_msg.position.data = (double*) malloc(enc_msg.position.capacity * sizeof(double));
  enc_msg.position.size = 0;

  enc_msg.velocity.capacity = 4; // 4 JOINTS!
  enc_msg.velocity.data = (double*) malloc(enc_msg.velocity.capacity * sizeof(double));
  enc_msg.velocity.size = 0;

  enc_msg.effort.capacity = 4; // 4 JOINTS!
  enc_msg.effort.data = (double*) malloc(enc_msg.effort.capacity * sizeof(double));
  enc_msg.effort.size = 0;

  //https://stackoverflow.com/questions/69317245/what-is-rosidl-runtime-c-double-sequence-type
  //currently not working
  // enc_msg.name.data[0].capacity = 5;
  // enc_msg.name.data[0].data = "M1";
  // enc_msg.name.data[0].size = strlen("M1");
  // enc_msg.name.data[1].capacity = 5;
  // enc_msg.name.data[1].data = "M2";
  // enc_msg.name.data[1].size = strlen("M2");
  // enc_msg.name.data[2].capacity = 5;
  // enc_msg.name.data[2].data = "M3";
  // enc_msg.name.data[2].size = strlen("M3");
  // enc_msg.name.data[3].capacity = 5;
  // enc_msg.name.data[3].data = "M4";
  // enc_msg.name.data[3].size = strlen("M4");
  // enc_msg.name.size = 4;
  // fill in zeros
  for(int32_t i = 0; i < 4; i++){
    enc_msg.position.data[i] = 0;
    enc_msg.position.size++;
    enc_msg.velocity.data[i] = 0;
    enc_msg.velocity.size++;
  }
}

void loop() {
  //timer for ROS
  if (millis() >= nextUpdateROS) {
    nextUpdateROS = millis() +  sampleTimeROS;
    RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  }
  //timer for publishing Encoders
  if (millis() >= nextUpdatePub) {
    nextUpdatePub = millis() +  sampleTimePub;

    //publish the position
    enc_msg.position.data[0]=-M1_theta_pub;
    enc_msg.position.data[1]=-M2_theta_pub;
    enc_msg.position.data[2]=M3_theta_pub;
    enc_msg.position.data[3]=M4_theta_pub;
    // //reset accumulator
    M1_theta_pub = 0;
    M2_theta_pub = 0;
    M3_theta_pub = 0;
    M4_theta_pub = 0;
    //publish the velocity
    enc_msg.velocity.data[0] = -M1_w;
    enc_msg.velocity.data[1] = -M2_w;
    enc_msg.velocity.data[2] = -M3_w;
    enc_msg.velocity.data[3] = -M4_w;
    // PIDrate_msg.data=52;
    RCSOFTCHECK(rcl_publish(&publisher_enc, &enc_msg, NULL));
    RCSOFTCHECK(rcl_publish(&publisher_2, &PIDrate_msg, NULL));
  }

  //timer for PID loop
  PIDtimer = millis();
  if (PIDtimer >= nextUpdatePID)
  {
    nextUpdatePID = PIDtimer + sampleTimePID; //50ms sampling time
    PIDrate_msg.data=PIDtimer-lastUpdatePID;
    lastUpdatePID = PIDtimer;

    // ch3Value = readChannel(RC_CH3, -1.0, 1.0, 0)/1000.0;
    // if (ch3Value > 0.0){
    //   ch1Value = readChannel(RC_CH1, w_min, w_max, 0)/1000.0;
    //   ch2Value = readChannel(RC_CH2, w_min, w_max, 0)/1000.0;
    //   M1_w_SetPoint = ch1Value;
    //   M2_w_SetPoint = ch1Value;
    //   M3_w_SetPoint = ch2Value;
    //   M4_w_SetPoint = ch2Value;
    // }
    //Read the encoders
    M1_ticks = M1_Enc.read();
    M2_ticks = M2_Enc.read();
    M3_ticks = M3_Enc.read();
    M4_ticks = M4_Enc.read();

    //Set encoder counts to zero
    M1_Enc.write(0);
    M2_Enc.write(0);
    M3_Enc.write(0);
    M4_Enc.write(0);

    //calculate the angular rate measured from the encoder increment
    //angular rate [rad/s] =  tickCount / ticksPerRev * rad per rev / gr * sample/second 
    M1_w = M1_ticks / 1024.0 * 2.0 * 3.14159 / 18.0 * 1000 / sampleTimePID;
    M2_w = M2_ticks / 1024.0 * 2.0 * 3.14159 / 18.0 * 1000 / sampleTimePID;
    M3_w = M3_ticks / 1024.0 * 2.0 * 3.14159 / 18.0 * 1000 / sampleTimePID;
    M4_w = M4_ticks / 1024.0 * 2.0 * 3.14159 / 18.0 * 1000 / sampleTimePID;

    //accumulate rotation to be published
    //assumes PID loop is sampling faster than pub
    M1_theta_pub += M1_ticks / 1024.0 * 2.0 * 3.14159 / 18.0;
    M2_theta_pub += M2_ticks / 1024.0 * 2.0 * 3.14159 / 18.0;
    M3_theta_pub += M3_ticks / 1024.0 * 2.0 * 3.14159 / 18.0;
    M4_theta_pub += M4_ticks / 1024.0 * 2.0 * 3.14159 / 18.0;

    //Run the PID loop
    M1_Pwm = motorPID(M1_w_SetPoint, M1_w, M1_w_LastSpeed, M1_Pwm, M1_ITerm);
    M2_Pwm = motorPID(M2_w_SetPoint, M2_w, M2_w_LastSpeed, M2_Pwm, M2_ITerm);
    M3_Pwm = motorPID(M3_w_SetPoint, M3_w, M3_w_LastSpeed, M3_Pwm, M3_ITerm);
    M4_Pwm = motorPID(M4_w_SetPoint, M4_w, M4_w_LastSpeed, M4_Pwm, M4_ITerm);

    //Write the Pwm to each motor
    adjustMotorPWM(M1_Pwm,M2_Pwm,M3_Pwm,M4_Pwm);

    for(uint16_t i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, strip.Color(abs(M1_Pwm), 0, abs(M3_Pwm)));
    }
    strip.show();
  }
}
