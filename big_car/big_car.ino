/*
 * rosserial PubSub Example
 * Prints "hello world!" and toggles led
 */


#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <Servo.h>
#include <ros.h>
#include <std_msgs/UInt16.h>

ros::NodeHandle  nh;

int pwm = 1420; 
Servo motor;

void servoCb( const std_msgs::UInt16& rpm_msg){
  
  pwm = rpm_msg.data;
}

ros::Subscriber<std_msgs::UInt16> motor_sub("motor_pub", servoCb );


std_msgs::UInt16 hall_msg;
ros::Publisher pub_hall( "hall_data", &hall_msg);

const int Hall_Sensor_Pin = 5;    // Hal sensor attached to pin 5. 

volatile byte half_revolutions;
unsigned int rpm;
unsigned long timeold;

void setup()
{
  nh.initNode();
  nh.advertise(pub_hall);
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
  // delay(1.5);
}

void rpm_fun()
 {
   half_revolutions++;
   //Each rotation, this interrupt function is run twice
   
 }


