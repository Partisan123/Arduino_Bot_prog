// #include <Arduino.h>
#include <micro_ros_arduino.h>
#include <math.h>

#include "pinout_esp32.h"
#include "motor.h"

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/float64.h>


// #define LED_BUILTIN 
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }
}

/*
 *    SkateCore 0.31eb
 * 
 *  This program can control motor speed & direction,
 *  & and read from hall encoder, using OOP.
 *  Run on Esp32 Wroom.
 * 
 *    Autobotix modular robot G-Bot2.71b
 */



// Declare motors
Motor Motor_LF(motor_lf, dir_lf1, dir_lf2, true, LF_ENCODER_P, LF_ENCODER_N);
Motor Motor_LB(motor_lb, dir_lb1, dir_lb2, true, LB_ENCODER_P, LB_ENCODER_N);
Motor Motor_RF(motor_rf, dir_rf1, dir_rf2, true, RF_ENCODER_P, RF_ENCODER_N);
Motor Motor_RB(motor_rb, dir_rb1, dir_rb2, true, RB_ENCODER_P, RB_ENCODER_N);

//uROS declarations
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_timer_t timer;

// Executors
rclc_executor_t executor_pub;
rclc_executor_t executor_sub;

//Declare uROS Pub/Sub

// Pub
rcl_publisher_t encLF_pub;
rcl_publisher_t encLB_pub;
rcl_publisher_t encRF_pub;
rcl_publisher_t encRB_pub;

std_msgs__msg__Float64 rpm_lf;
std_msgs__msg__Float64 rpm_lb;
std_msgs__msg__Float64 rpm_rf;
std_msgs__msg__Float64 rpm_rb;

//Sub
rcl_subscription_t twist_sub;
geometry_msgs__msg__Twist msg_move;




// Hardware interrupt handler functions

void cntEnc0() {
// cntEnc is activated if DigitalPin is going from LOW to HIGH
// Check pin to determine direction
    if(digitalRead(LF_ENCODER_N)==LOW) 
      Motor_LF.encoder.encoderCount++;
    else 
      Motor_LF.encoder.encoderCount--;
}
void cntEnc1() {
// cntEnc is activated if DigitalPin is going from LOW to HIGH
// Check pin to determine the direction
    if(digitalRead(LF_ENCODER_P)==LOW) 
      Motor_LF.encoder.encoderCount--;
    else 
      Motor_LF.encoder.encoderCount++;
}

void cntEnc2() {

    if(digitalRead(LB_ENCODER_N)==LOW) 
      Motor_LB.encoder.encoderCount++;
    else 
      Motor_LB.encoder.encoderCount--;
  
}
void cntEnc3() {

    if(digitalRead(LB_ENCODER_P)==LOW) 
      Motor_LB.encoder.encoderCount--;
    else 
      Motor_LB.encoder.encoderCount++;

}

void cntEnc4() {

    if(digitalRead(RF_ENCODER_N)==LOW) 
      Motor_RF.encoder.encoderCount++;
    else 
      Motor_RF.encoder.encoderCount--;
  
}
void cntEnc5() {

    if(digitalRead(RF_ENCODER_P)==LOW) 
      Motor_RF.encoder.encoderCount--;
    else 
      Motor_RF.encoder.encoderCount++;

}

void cntEnc6() {

    if(digitalRead(RB_ENCODER_N)==LOW) 
      Motor_RB.encoder.encoderCount++;
    else 
      Motor_RB.encoder.encoderCount--;
  
}
void cntEnc7() {
 
    if(digitalRead(RB_ENCODER_P)==LOW) 
      Motor_RB.encoder.encoderCount--;
    else 
      Motor_RB.encoder.encoderCount++;
}


// Motion directions declaration
float power_y = 0, power_x = 0, power_z = 0, alpha = 0;


//ROS functions & callbacks
void pub_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&encLF_pub, &rpm_lf, NULL));
    RCSOFTCHECK(rcl_publish(&encLB_pub, &rpm_lb, NULL));
    RCSOFTCHECK(rcl_publish(&encRF_pub, &rpm_rf, NULL));
    RCSOFTCHECK(rcl_publish(&encRB_pub, &rpm_rb, NULL));

    rpm_lf.data = Motor_LF.encoder.rpm;
    rpm_lb.data = Motor_LB.encoder.rpm;
    rpm_rf.data = Motor_RF.encoder.rpm;
    rpm_rb.data = Motor_RB.encoder.rpm;
  }
}

void sub_callback(const void *msgin)
{
  const geometry_msgs__msg__Twist *msg_move = (const geometry_msgs__msg__Twist *)msgin;
  // (condition) ? (true exec):(false exec)
  power_x = msg_move->linear.x;
  power_y = msg_move->linear.y;
  power_z = msg_move->angular.z;
}

void linear_y(float power){
  Motor_RF.drive(power);
  Motor_LF.drive(power);
  Motor_RB.drive(power);
  Motor_LB.drive(power);
}


void linear_x(float power){
  Motor_RF.drive(power);
  Motor_LF.drive(-power);
  Motor_RB.drive(-power);
  Motor_LB.drive(power);
}


void angular_z(float power){ //clockwise
  Motor_RF.drive(-power);
  Motor_LF.drive(power);
  Motor_RB.drive(-power);
  Motor_LB.drive(power);
}

void mixed_motion_xy(float pow_y, float pow_x){
  float power;
  
  power = sqrt(sq(pow_y)+sq(pow_x));
  alpha = atan(pow_y/pow_x);
    
  Motor_RF.drive(power*sin(alpha - (M_PI/4)));
  Motor_LF.drive(power*sin(alpha + (M_PI/4)));
  Motor_RB.drive(power*sin(alpha + (M_PI/4)));
  Motor_LB.drive(power*sin(alpha - (M_PI/4)));
}

    

// Init hardware function
void setup() {
  
  // Start serial output 
  Serial.begin(115200);
  set_microros_transports();
  delay(2000);

  allocator = rcl_get_default_allocator();
  
  // Setup hardware interrupt pins for encoder sensors
  attachInterrupt(digitalPinToInterrupt(LF_ENCODER_P), cntEnc0, RISING);   //Hardware interrupt pin 1 for encoder 
  attachInterrupt(digitalPinToInterrupt(LF_ENCODER_N), cntEnc1, RISING);   //Hardware interrupt pin 2 for encoder 

  attachInterrupt(digitalPinToInterrupt(LB_ENCODER_P), cntEnc2, RISING);
  attachInterrupt(digitalPinToInterrupt(LB_ENCODER_N), cntEnc3, RISING);

  attachInterrupt(digitalPinToInterrupt(RF_ENCODER_P), cntEnc4, RISING);
  attachInterrupt(digitalPinToInterrupt(RF_ENCODER_N), cntEnc5, RISING);

  attachInterrupt(digitalPinToInterrupt(RB_ENCODER_P), cntEnc6, RISING);
  attachInterrupt(digitalPinToInterrupt(RB_ENCODER_N), cntEnc7, RISING);


  // Create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Create node
  RCCHECK(rclc_node_init_default(&node, "skateNode", "", &support));

  // Create publishers
  RCCHECK(rclc_publisher_init_default(
    &encLF_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
    "rpm_lf_pub"));

  RCCHECK(rclc_publisher_init_default(
    &encLB_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
    "rpm_lb_pub"));

  RCCHECK(rclc_publisher_init_default(
    &encRF_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
    "rpm_rf_pub"));

  RCCHECK(rclc_publisher_init_default(
    &encRB_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
    "rpm_rb_pub"));

  // Create subscribers
  RCCHECK(rclc_subscription_init_default(
    &twist_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));


  // Create executor & Timers

  const unsigned int timer_timeout = 100;
  RCCHECK(rclc_timer_init_default(
            &timer,
            &support,
            RCL_MS_TO_NS(timer_timeout),
            pub_timer_callback));
  
  RCCHECK(rclc_executor_init(&executor_pub, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor_pub, &timer));

  RCCHECK(rclc_executor_init(&executor_sub, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor_sub, &twist_sub, &msg_move, &sub_callback, ON_NEW_DATA));

  rpm_lf.data = 0;
  rpm_lb.data = 0;
  rpm_rf.data = 0;
  rpm_rb.data = 0;

}

// Do stuff here
void loop() {
  
  Serial.print("RPM LF: "); Serial.println(Motor_LB.encoder.rpm);
  Serial.print("RPM LB: "); Serial.println(Motor_LF.encoder.rpm);
  Serial.print("RPM RF: "); Serial.println(Motor_RF.encoder.rpm);
  Serial.print("RPM RB: "); Serial.println(Motor_RB.encoder.rpm);

 // Driving functions
  if(power_x == 0 && power_z == 0 ){
    linear_y(power_y);
  }
  else if(power_y == 0 && power_z == 0){
    linear_x(power_x);  
  }
  else if(power_y == 0 && power_x == 0){
    angular_z(power_z);  
  }
  else {
    mixed_motion_xy(power_y , power_x);
  }

  // Spin executors
  RCSOFTCHECK(rclc_executor_spin_some(&executor_pub, RCL_MS_TO_NS(100)));
  RCSOFTCHECK(rclc_executor_spin_some(&executor_sub, RCL_MS_TO_NS(100)));
  delay(50);

}

  
