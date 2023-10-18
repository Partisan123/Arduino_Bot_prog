//#include <Arduino.h>

#include "pinout.h"
#include "motor.h"
#include <math.h>
#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>

/*
 *    SkateCore 0.19
 * 
 *  This program can control motor speed & direction,
 *  & and read from hall encoder, using OOP.
 *  Run on RaspberryPi Pico RP2040.
 * 
 *    Autobotix modular robot G-Bot0.9d1
 */

ros::NodeHandle nh;

// declare motors
Motor Motor_LF(motor_lf, dir_lf1, dir_lf2, false, LF_ENCODER_P, LF_ENCODER_N);
Motor Motor_LB(motor_lb, dir_lb1, dir_lb2, false, LB_ENCODER_P, LB_ENCODER_N);
Motor Motor_RF(motor_rf, dir_rf1, dir_rf2, true, RF_ENCODER_P, RF_ENCODER_N);
Motor Motor_RB(motor_rb, dir_rb1, dir_rb2, false, RB_ENCODER_P, RB_ENCODER_N);


//ROS declarations
std_msgs::Float64 rpm_rf;
std_msgs::Float64 rpm_lf;
std_msgs::Float64 rpm_rb;
std_msgs::Float64 rpm_lb;

float power_y = 0, power_x = 0, power_z = 0, alpha = 0;


//ROS functions

void set_bot_speed(const geometry_msgs::Twist& set_bot_speed){
    power_x = set_bot_speed.linear.x;
    power_y = set_bot_speed.linear.y;
    power_z = set_bot_speed.angular.z;
}


//Declare ROS Pub/Sub

ros::Publisher rpm_rf_pub("RPM_RF", &rpm_rf);
ros::Publisher rpm_lf_pub("RPM_LF", &rpm_lf);
ros::Publisher rpm_rb_pub("RPM_RB", &rpm_rb);
ros::Publisher rpm_lb_pub("RPM_LB", &rpm_lb);

ros::Subscriber<geometry_msgs::Twist> sub_drive("drive_bot" , set_bot_speed);


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


// motion directions

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
  
    // Setup hardware interrupt pins for encoder sensors
    attachInterrupt(digitalPinToInterrupt(LF_ENCODER_P), cntEnc0, RISING);   //Hardware interrupt pin 1 for encoder 
    attachInterrupt(digitalPinToInterrupt(LF_ENCODER_N), cntEnc1, RISING);   //Hardware interrupt pin 2 for encoder 

    attachInterrupt(digitalPinToInterrupt(LB_ENCODER_P), cntEnc2, RISING);
    attachInterrupt(digitalPinToInterrupt(LB_ENCODER_N), cntEnc3, RISING);

    attachInterrupt(digitalPinToInterrupt(RF_ENCODER_P), cntEnc4, RISING);
    attachInterrupt(digitalPinToInterrupt(RF_ENCODER_N), cntEnc5, RISING);

    attachInterrupt(digitalPinToInterrupt(RB_ENCODER_P), cntEnc6, RISING);
    attachInterrupt(digitalPinToInterrupt(RB_ENCODER_N), cntEnc7, RISING);

    nh.initNode();
    
    nh.advertise(rpm_rf_pub);
    nh.advertise(rpm_lf_pub);
    nh.advertise(rpm_rb_pub);
    nh.advertise(rpm_lb_pub);
    
    nh.subscribe(sub_drive);

    while(!nh.connected()){
      nh.spinOnce();
    }


}

// Do stuff here
void loop() {


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
    
    
    rpm_rf.data = Motor_RF.encoder.rpm;
    rpm_lf.data = Motor_LF.encoder.rpm;
    rpm_rb.data = Motor_RB.encoder.rpm;
    rpm_lb.data = Motor_LB.encoder.rpm;
    
    rpm_rf_pub.publish(&rpm_rf);
    rpm_lf_pub.publish(&rpm_lf);
    rpm_rb_pub.publish(&rpm_rb);
    rpm_lb_pub.publish(&rpm_lb);
      
    nh.spinOnce();
    delay(50);

}

  
