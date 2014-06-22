/***********************************************************************************
 * Code for running the motor in a sinusoidal manner using PWM signals.
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

ros::NodeHandle nh;

int pwm = 1600; 
Servo motor;


void servo_cb( const std_msgs::UInt16& cmd_msg){   // Call back function from the Subscriber 
                                                   // Reads the data off of the publisher
  //servo.write(cmd_msg.data);                       // Send in the start value for the Servo motor
  pwm = cmd_msg.data;
  // motor.writeMicroseconds(pwm);
  digitalWrite(13, HIGH-digitalRead(13));    //Toggle LED on pin 13
 
}

ros::Subscriber<std_msgs::UInt16> sub("motor", servo_cb);    // Subscribing to "motor1" and 
                                                              // put it into the callback function (1600)
void setup() {
  
  //Serial.begin(57600);
  pinMode(13, OUTPUT);
  
  nh.initNode();
  nh.subscribe(sub);
 
  motor.attach(9);
  for (int x = 0; x<1; x++){
  motor.writeMicroseconds(pwm);  // Needs to be in the publisher 

  delay(4000); 
  }


}

// neutral range begins at 1590 and ends at 1690?
void loop() {
  
  
// Sin wave code for both directions. Can be smoothed out by decreasing the increments
 
  for (int i = (1700); i < 1800; i = i + 20){    // pwm +70 = 1670 
    motor.writeMicroseconds(i);  // start from 1670 to 1800
   // Serial.println(i);
    }
  nh.spinOnce();
  delay(1);
  
  for (int i = (1800); i > 1700; i = i - 20){    // pwm +200 = 1800 
    motor.writeMicroseconds(i);   
   //Serial.println(i); 
   } 
   
  nh.spinOnce();
  delay(1);
  
  for (int i = (1590); i > 1400; i = i - 20){    // pwm -10 = 1590
    motor.writeMicroseconds(i);  
    //Serial.println(i);
   }
  nh.spinOnce();
  delay(1);
  
   for (int i = (1400); i < 1600; i = i + 20){    // pwm - 200 = 1400
    motor.writeMicroseconds(i);   
    //Serial.println(i);
    //delay(1000);    
   }
 
  nh.spinOnce();
  delay(1);
  
}

