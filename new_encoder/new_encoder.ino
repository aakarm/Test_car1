#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Int16.h>

//#include <geometry_msgs/Vector3.h>

ros::NodeHandle  nh;
std_msgs::Int16 encoder0_msg;
ros::Publisher pub_encoder0( "encoder0_data", &encoder0_msg);

std_msgs::Int16 encoder1_msg;
ros::Publisher pub_encoder1( "encoder1_data", &encoder1_msg);

//std_msgs::UInt16 dist_msg;
//ros::Publisher pub_dist( "dist", &dist_msg);

#define encoder0PinA  3
#define encoder0PinB  4
#define encoder1PinA  5
#define encoder1PinB  6


volatile int encoder0Pos = 0;
volatile int encoder1Pos = 111;
volatile int encoder0Flag =0;
volatile int encoder1Flag =0;
int indexing = 0 ;



void setup()
 {
  nh.initNode();
  nh.advertise(pub_encoder0);  
  nh.advertise(pub_encoder1);
 // nh.advertise(pub_dist);
  
  pinMode(encoder0PinA, INPUT); 
  digitalWrite(encoder0PinA, HIGH);       // turn on pullup resistor
  pinMode(encoder0PinB, INPUT); 
  digitalWrite(encoder0PinB, HIGH);  // turn on pullup resistor
  attachInterrupt(encoder0PinA, doEncoder0, RISING);  // encoder pin on interrupt 0 - pin 2
  
//  pinMode(encoder1PinA, INPUT); 
//  digitalWrite(encoder1PinA, HIGH);       // turn on pullup resistor
//  pinMode(encoder1PinB, INPUT); 
//  digitalWrite(encoder1PinB, HIGH);  // turn on pullup resistor
//  attachInterrupt(encoder1PinA, doEncoder1, RISING);  // encoder pin on interrupt 0 - pin 2   // This is rising edge, so the multiplying constant changes in ROS
//  
 }

 void loop()
 {
 
     nh.spinOnce();
     indexing++;
     
     if (indexing ==10000)   // changing this indexing limit to get the right speed of communication  // Shishir Kolathaya
     {
       indexing =0;
       encoder0_msg.data = encoder0Pos;
       pub_encoder0.publish( &encoder0_msg);
     
//       encoder1_msg.data = encoder1Pos;
//       pub_encoder1.publish( &encoder1_msg);
     }
     
//     if (encoder0Flag >0)
//     {
//           if (digitalRead(encoder0PinA) == digitalRead(encoder0PinB)) encoder0Pos+=encoder0Flag;
//           else   encoder0Pos-=encoder0Flag;
//           encoder0Flag =0;
//     }
//     
//     if (encoder1Flag >0)
//     {
//           if (digitalRead(encoder1PinA) == digitalRead(encoder1PinB)) encoder1Pos+=encoder1Flag;
//           else   encoder1Pos-=encoder1Flag;
//           encoder1Flag =0;
//     }
   
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
  } else {
    encoder0Pos--;
   }
//encoder0Flag++;
 }
  
//   void doEncoder1() {
//  /* If pinA and pinB are both high or both low, it is spinning
//   * forward. If they're different, it's going backward.
//   *
//   * For more information on speeding up this process, see
//   * [Reference/PortManipulation], specifically the PIND register.
//   */ 
//  if (digitalRead(encoder1PinA) == digitalRead(encoder1PinB)) {
//    encoder1Pos++;
//  } else {
//    encoder1Pos--;
//  }
////encoder1Flag++;
//}

