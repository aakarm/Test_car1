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

#define encoder0PinA  3
#define encoder0PinB  4
int encoder_ticks = 0;
double EncoderTime = 0;
double prevTime = 0;
double Speed = 0;

volatile unsigned int encoder0Pos = 0;
ros::NodeHandle  nh;

int pwm = 1420; 
Servo motor;


void servoCb( const std_msgs::UInt16& rpm_msg){
  
  pwm = rpm_msg.data;
}

ros::Subscriber<std_msgs::UInt16> motor_sub("motor_pub", servoCb );


std_msgs::UInt16 hall_msg;
ros::Publisher pub_hall( "hall_data", &hall_msg);

std_msgs::UInt16 encoder_msg;
ros::Publisher pub_encoder( "encoder_data", &encoder_msg);


const int Hall_Sensor_Pin = 5;    // Hal sensor attached to pin 5. 

volatile byte half_revolutions;
unsigned int rpm;
unsigned long timeold;

void setup()
{
  nh.initNode();
  nh.advertise(pub_hall);
  nh.subscribe(motor_sub);
  nh.advertise(pub_encoder);  
  
  pinMode(encoder0PinA, INPUT); 
  digitalWrite(encoder0PinA, HIGH);       // turn on pullup resistor
  pinMode(encoder0PinB, INPUT); 
  digitalWrite(encoder0PinB, HIGH);  // turn on pullup resistor
  attachInterrupt(3, doEncoder, CHANGE);  // encoder pin on interrupt 0 - pin 2
 
  motor.attach(9);
  
  for (int x = 0; x<1; x++){
  motor.writeMicroseconds(1450);  // Needs to be in the publisher 

  delay(3000); 
  }
  
  attachInterrupt(Hall_Sensor_Pin, rpm_fun, RISING);// note: 0 refers to inturrupt on pin 2
  half_revolutions = 0;
  rpm = 0;
  timeold = 0;
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
  
  nh.spinOnce();
  
  motor.writeMicroseconds(pwm);

  //get_rpm_Encoder();
  encoder_msg.data = encoder0Pos;
  pub_encoder.publish( &encoder_msg);
  
  nh.spinOnce();
  // delay(1.5);
}

void rpm_fun()
 {
   half_revolutions++;
   //Each rotation, this interrupt function is run twice
   
 }

void doEncoder() {
  /* If pinA and pinB are both high or both low, it is spinning
   * forward. If they're different, it's going backward.
   *
   * For more information on speeding up this process, see
   * [Reference/PortManipulation], specifically the PIND register.
   */
  if (digitalRead(encoder0PinA) == digitalRead(encoder0PinB)) {
    encoder0Pos++;
    encoder_ticks++;
  } else {
    encoder0Pos--;
  }

  //Serial.println (encoder0Pos, DEC);
}

/*void get_rpm_Encoder(){
  EncoderTime = millis() - prevTime;
  if (EncoderTime == 100){
    // circumference is 23 meters. measuring velocity every .1 sec. 4096 encoder ticks per revolution
    // Serial.println(encoder_ticks*230/4096); // conversion to m/s
    Speed = encoder_ticks*230/4096;
    prevTime = millis();
    encoder_ticks = 0;
  }
}
*/
