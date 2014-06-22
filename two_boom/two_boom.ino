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
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
//#include <std_msgs/UInt32.h>

#define encoder0PinA  3
#define encoder0PinB  4
#define encoder1PinA  5
#define encoder1PinB  6
const int Hall_Sensor_Pin = 7;    // Hal sensor attached to pin 5. 
volatile int encoder0Pos = 0;
volatile int encoder1Pos = 110;
volatile int prevEncoderPos=0;

double EncoderTime = 0;
double prevTime = 0;
double Speed = 0;
int velocity_dt_ms = 10;


//int encoder_ticks0 = 0;
//int encoder_ticks1 = 0;

ros::NodeHandle  nh;

int pwm = 1460; 
Servo motor;


void servoCb( const std_msgs::UInt16& rpm_msg){
  
  pwm = rpm_msg.data;
}

ros::Subscriber<std_msgs::UInt16> motor_sub("motor_pub", servoCb );


std_msgs::UInt16 hall_msg;
ros::Publisher pub_hall( "hall_data", &hall_msg);

std_msgs::Int16 encoder0_msg;
ros::Publisher pub_encoder0( "encoder0_data", &encoder0_msg);

std_msgs::Int16 encoder1_msg;
ros::Publisher pub_encoder1( "encoder1_data", &encoder1_msg);

std_msgs::Float64 speed_msg;
ros::Publisher pub_speed( "speed_data", &speed_msg);


volatile byte half_revolutions;
unsigned int rpm;
unsigned long timeold;

void setup()
{
  nh.initNode();
  nh.advertise(pub_hall);
  nh.advertise(pub_encoder0);  
  nh.advertise(pub_encoder1);
//  nh.advertise(pub_speed);
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

     encoder0_msg.data = encoder0Pos;
     pub_encoder0.publish( &encoder0_msg);
     
     encoder1_msg.data = encoder1Pos;
     pub_encoder1.publish( &encoder1_msg);
     
     nh.spinOnce(); 
     
    
     
     EncoderTime = millis() - prevTime;
     if (EncoderTime >= velocity_dt_ms){
 
        // circumference is 23 meters. measuring velocity every .1 sec. 4096 encoder ticks per revolution
        Speed = (encoder0Pos - prevEncoderPos)/EncoderTime;     // scaler conversion to m/s: 230/4096
     
        prevTime = millis();
        prevEncoderPos = encoder0Pos;
        //encoder_ticks = 0;
        }
     
     speed_msg.data = Speed;
     pub_speed.publish( &speed_msg);

     nh.spinOnce();
    // delay(1.5);
}

void rpm_fun()
 {
   half_revolutions++;
   //Each rotation, this interrupt function is run twice
   
 }

void doEncoder0() {
  /* If pinA and pinB are both high or both low, it is spinning
   * forward. If they're different, it's going backward.
   *
   * For more information on speeding up this process, see
   * [Reference/PortManipulation], specifically the PIND register.
   */ 
  if (digitalRead(encoder0PinA) == digitalRead(encoder0PinB)) {
    encoder0Pos++;
   //encoder_ticks0++;
  } else {
    encoder0Pos--;
    //encoder_ticks0++;
  }
 }
  
   void doEncoder1() {
  /* If pinA and pinB are both high or both low, it is spinning
   * forward. If they're different, it's going backward.
   *
   * For more information on speeding up this process, see
   * [Reference/PortManipulation], specifically the PIND register.
   */ 
  if (digitalRead(encoder1PinA) == digitalRead(encoder1PinB)) {
    encoder1Pos++;
   //encoder_ticks1++;
  }
    else {
    encoder1Pos--;
    //encoder_ticks1++;
          }
}

//void get_rpm_Encoder(){
//  EncoderTime = millis() - prevTime;
//  if (EncoderTime >= velocity_dt_ms){
// 
//    // circumference is 23 meters. measuring velocity every .1 sec. 4096 encoder ticks per revolution
//    Speed = (encoder0Pos - prevEncoderPos)/EncoderTime;     // scaler conversion to m/s: 230/4096
//    //Serial.println(encoder_ticks*230/4096);
//    // conversion to m/s
//    //Serial.println(Speed);
//    prevTime = millis();
//    prevEncoderPos = encoder0Pos;
//    //encoder_ticks = 0;
//  }
//}
