#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
//#include <geometry_msgs/Vector3.h>

ros::NodeHandle  nh;
std_msgs::Int16 encoder_msg;
ros::Publisher pub_encoder( "encoder_data", &encoder_msg);

std_msgs::Float64 Speed_msg;
ros::Publisher pub_Speed( "Speed", &Speed_msg);

#define encoder0PinA  3
#define encoder0PinB  4
//volatile int encoder_ticks = 0;
double EncoderTime = 0;
double prevTime = 0;
double Speed = 0;
int velocity_dt_ms = 10;
double cur_time = millis();

int encoder0Pos = 0;
int prevEncoderPos=0;
void setup() { 
  //Serial.begin (115200);
  //Serial.println("start");                // a personal quirk
  pinMode(encoder0PinA, INPUT); 
  digitalWrite(encoder0PinA, HIGH);       // turn on pullup resistor
  pinMode(encoder0PinB, INPUT); 
  digitalWrite(encoder0PinB, HIGH);  // turn on pullup resistor
  attachInterrupt(encoder0PinA, doEncoder, CHANGE);  // encoder pin on interrupt 0 - pin 2
} 


 void loop()
 {
     
     encoder_msg.data = encoder0Pos;
     pub_encoder.publish( &encoder_msg);
     
//    EncoderTime = cur_time - prevTime;
//    //get_rpm_Encoder();
//     
//    if (EncoderTime >= velocity_dt_ms){
//    // circumference is 23 meters. measuring velocity every .1 sec. 4096 encoder ticks per revolution
//    Speed = (encoder0Pos - prevEncoderPos)/EncoderTime;     // scaler conversion to m/s: 230/4096
//    prevTime = cur_time;
//    prevEncoderPos = encoder0Pos;
//    nh.spinOnce();
//    //encoder_ticks = 0;
//    }
  
     encoder_msg.data = encoder0Pos;
     pub_encoder.publish( &encoder_msg);
     
     Speed_msg.data = Speed;
     pub_Speed.publish( &Speed_msg);
     
     //encoder_ticks =0;
     
   
   nh.spinOnce();
   delay(1);
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
   //encoder_ticks++;
  } else {
    encoder0Pos--;
  }

  //Serial.println (encoder0Pos, DEC);
}

void get_rpm_Encoder(){
  
  if (EncoderTime >= velocity_dt_ms){
 
    // circumference is 23 meters. measuring velocity every .1 sec. 4096 encoder ticks per revolution
    Speed = (encoder0Pos - prevEncoderPos)/EncoderTime;     // scaler conversion to m/s: 230/4096
    //Serial.println(encoder_ticks*230/4096);
    // conversion to m/s
    //Serial.println(Speed);
    prevTime = millis();
    prevEncoderPos = encoder0Pos;
    //encoder_ticks = 0;
  }

}


