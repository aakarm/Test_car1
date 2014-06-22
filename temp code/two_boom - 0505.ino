/***********************************************************************************
 * Code for running the motor using a PWM signal inputed after solving a model 
 * independent version of the QP (closed form solution). The testing is being 
 * done on two electric cars. 
 * 
 * The same code will be used for the Model Dependent QP as well. This code only 
 * measuer the current rpm on the wheels and outputs the current velocity of the cars 
 * to be solved with any solver.
 * This code has been adapted from the Servo Motor Cotnrol Example on ROS Wiki.
 *
 * Also, parts of it has been taken from Forrest Berg's code for the Arduino DUE.
 *
 * The code is basically a node for the ROS environemnt which is run using the terminal
 * after we upload the code onto the port connecting the Arduino board. 
 * This is test code for the main project of Adaptive Crusie Control before testing PID.
 *
 **************  Author  ************  Date  *************  Project  *************
 *             Aakar Mehra          May 28, 2014      Adaptive Cruise Control 
 * 
 ****************************************************************************************
 */
 

#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <Servo.h>
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Int64.h>
//#include <std_msgs/UInt32.h>

#define encoder0PinA  3
#define encoder0PinB  4
#define encoder1PinA  5
#define encoder1PinB  6
volatile int encoder0Pos = 0;
volatile int encoder1Pos = 0;
int encoder_ticks = 0;
int encoder0Aold;
int encoder1Aold;
int distance = 0; 
ros::NodeHandle  nh;

int pwm = 1460; 
Servo motor;


void servoCb( const std_msgs::UInt16& rpm_msg){
  
  pwm = rpm_msg.data;
}

ros::Subscriber<std_msgs::UInt16> motor_sub("motor_pub", servoCb );


std_msgs::UInt16 hall_msg;
ros::Publisher pub_hall( "hall_data", &hall_msg);

//std_msgs::Int64 encoder0_msg;
//ros::Publisher pub_encoder0( "encoder0_data", &encoder0_msg);
//
//std_msgs::Int64 encoder1_msg;
//ros::Publisher pub_encoder1( "encoder1_data", &encoder1_msg);

std_msgs::Int64 distance_msg;
ros::Publisher pub_distance( "distance_data", &distance_msg);

const int Hall_Sensor_Pin = 7;    // Hal sensor attached to pin 5. 

volatile byte half_revolutions;
unsigned int rpm;
unsigned long timeold;

void setup()
{
  nh.initNode();
  nh.advertise(pub_hall);
//  nh.advertise(pub_encoder0);  
//  nh.advertise(pub_encoder1);
  nh.advertise(pub_distance);
  nh.subscribe(motor_sub);
  
  motor.attach(9);
  
  for (int x = 0; x<1; x++){
  motor.writeMicroseconds(1450);  // Needs to be in the publisher 

  delay(3000); 
  }
  
  attachInterrupt(Hall_Sensor_Pin, rpm_fun, RISING);// note: 0 refers to inturrupt on pin 2
  half_revolutions = 0;
  rpm = 0;
  timeold = 0;
  
  pinMode(encoder0PinA, INPUT); 
  digitalWrite(encoder0PinA, HIGH);       // turn on pullup resistor
  pinMode(encoder0PinB, INPUT); 
  digitalWrite(encoder0PinB, HIGH);  // turn on pullup resistor
  attachInterrupt(encoder0PinA, doEncoder0, CHANGE);  // encoder pin on interrupt 0 - pin 2
  
  pinMode(encoder1PinA, INPUT); 
  digitalWrite(encoder1PinA, HIGH);       // turn on pullup resistor
  pinMode(encoder1PinB, INPUT); 
  digitalWrite(encoder1PinB, HIGH);  // turn on pullup resistor
  attachInterrupt(encoder1PinA, doEncoder1, CHANGE);  // encoder pin on interrupt 0 - pin 2
  
}

void loop()
{
  if (half_revolutions >= 2) { 
     //Update RPM every 20 counts, increase this for better RPM resolution,
     //decrease for faster update
     rpm = 30*1000/(millis() - timeold)*half_revolutions;
     timeold = millis();
     half_revolutions = 0;
     hall_msg.data = rpm;
     pub_hall.publish( &hall_msg);
                    }
  
  motor.writeMicroseconds(pwm);
  
     nh.spinOnce(); 

//     encoder0_msg.data = 0;        //encoder0Pos
//     pub_encoder0.publish( &encoder0_msg);
     
     if(encoder_ticks >= 10)
     {
//     encoder0_msg.data = encoder0Pos;        //encoder0Pos
//     pub_encoder0.publish( &encoder0_msg);
//     
//     encoder1_msg.data = encoder1Pos;
//     pub_encoder1.publish( &encoder1_msg);
     
     distance = encoder1Pos - encoder0Pos;
     
     distance_msg.data = distance;
     pub_distance.publish( &distance_msg);
     
     encoder_ticks = 0;
     }

  nh.spinOnce();
  // delay(1.5);
}

void rpm_fun()
 {
   half_revolutions++;
   //Each rotation, this interrupt function is run twice
   
 }

void doEncoder0() {
  
  encoder0Aold = digitalRead(encoder0PinA);
 int encoder1Anew = digitalRead(encoder1PinA);
  /* If pinA and pinB are both high or both low, it is spinning
   * forward. If they're different, it's going backward.
   *
   * For more information on speeding up this process, see
   * [Reference/PortManipulation], specifically the PIND register.
   */ 
  if (encoder0Aold == digitalRead(encoder0PinB)) {
    encoder0Pos++;
   encoder_ticks++;
  } else {
    encoder0Pos--;
    encoder_ticks++;
  }
  
  if (encoder1Aold != encoder1Anew){
    if (encoder1Anew == digitalRead(encoder1PinB)) {
      encoder1Pos++;
     encoder_ticks++;
    }
      else {
      encoder1Pos--;
      encoder_ticks++;
      }
  }  
   encoder1Aold = encoder1Anew;
   

 }
  
   void doEncoder1() {
  /* If pinA and pinB are both high or both low, it is spinning
   * forward. If they're different, it's going backward.
   *
   * For more information on speeding up this process, see
   * [Reference/PortManipulation], specifically the PIND register.
   */ 
   encoder1Aold = digitalRead(encoder1PinA);
 int encoder0Anew = digitalRead(encoder0PinA);
   
  if (encoder1Aold == digitalRead(encoder1PinB)) {
    encoder1Pos++;
   encoder_ticks++;
  }
    else {
    encoder1Pos--;
    encoder_ticks++;
          }
    if (encoder0Aold != encoder0Anew){      
      if (encoder0Anew == digitalRead(encoder0PinB)) {
      encoder0Pos++;
      encoder_ticks++;
    }   else {
      encoder0Pos--;
      encoder_ticks++;
    }      
   }
    encoder0Aold = encoder0Anew;
   
}

