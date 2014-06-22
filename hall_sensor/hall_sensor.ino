#include <ros.h>
#include <ros/time.h>
#include <std_msgs/UInt16.h>
//#include <geometry_msgs/Vector3.h>

ros::NodeHandle  nh;
std_msgs::UInt16 hall_msg;
ros::Publisher pub_hall( "hall_data", &hall_msg);

const int Hall_Sensor_Pin = 5;    // Hal sensor attached to pin 5. 

volatile byte half_revolutions;
unsigned int rpm;
unsigned long timeold;

//geometry_msgs/Vecotr3 magnetic_field 
//float64[9] magnetic_filed_covariance

void setup()
 {
  // Serial.begin(9600);
  nh.initNode();
  nh.advertise(pub_hall);  
  
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
     //Serial.println(rpm,DEC);]
     hall_msg.data = rpm;
     pub_hall.publish( &hall_msg);
   }
    nh.spinOnce();
 }
 void rpm_fun()
 {
   half_revolutions++;
   //Each rotation, this interrupt function is run twice
   
 }


