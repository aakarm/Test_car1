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
 **************  Author  **************  Date  **************  Project  *************
 *             Aakar Mehra           May 28, 2014      Adaptive Cruise Control 
 * 
 ****************************************************************************************
 */
 
#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <Servo.h> 
//#define USE_USBCON
#include <ros.h>
#include <std_msgs/UInt16.h>

ros::NodeHandle nh;

//int pwm = 1600; 
Servo motor;
int Hall_Sensor_Pin = 5; // 1 for uno // 
int motor_Pin = 9;

//Parameter declaration
double V_desired = 1200;                // rpm
int IntThreshold = 700;            // rpm error threshold at which integral control is initiated
int LastRpm = 0;
double Kp = .0000003; //Kp = .00000008 
double Error = 0;
double P = 0;
double Input;
double Hall_Speed;
int min_Velocity = 1690; // input in milliseconds to achieve 0 speed
int max_Velocity = 1900;     // input in milliseconds to accieve max speed

/*
// open loop input to velocity equation parameters
double p1 = 5.5541e-12;
double p2 = 1.4184e-07;
double p3 = -0.00042008;
double p4 = 0.47892;
double p5 = 1567;
*/

// Values for rpm sensor
volatile byte half_revolutions;

double CurrentRpm;
unsigned long timeold;


void servo_cb( const std_msgs::UInt16& cmd_msg) {  // Call back function from the Subscriber 
                                                   // Reads the data off of the publisher
  Input = cmd_msg.data;                              // Send in the start value for the Servo motor 
}

ros::Subscriber<std_msgs::UInt16> sub("motor", servo_cb);    // Subscribing to "motor1" and 
                                                              // put it into the callback function (1600)
                                                              
std_msgs::UInt16 hall_msg;
ros::Publisher pub_hall( "hall_data", &hall_msg);


void setup() {
  
  //nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(pub_hall);
  nh.subscribe(sub);
 
 // Serial.begin(115200);
  pinMode(Hall_Sensor_Pin, INPUT);
  attachInterrupt(Hall_Sensor_Pin, rpm_fun, RISING);// note: 0 refers to inturrupt on pin 2
  motor.attach(motor_Pin);
  motor.writeMicroseconds(1600); // neutral signal to initialize car
  delay(4000);
}

void loop() {
  
  nh.spinOnce();
  get_rpm();
  
  Error = V_desired - CurrentRpm;
  calc_speed();
  
  //nh.spinOnce();
  
  motor.write(Input);
  LastRpm = CurrentRpm;
  
  hall_msg.data = Hall_Speed;
  pub_hall.publish( &hall_msg);
  
  //nh.spinOnce();
  //delay(1);
  
}


void rpm_fun(){
   half_revolutions++;
   //Each rotation, this interrupt function is run twice
}


void get_rpm(){
  if (half_revolutions >= 2) { 
    //Update RPM every 20 counts, increase this for better RPM resolution,
    //decrease for faster update
    CurrentRpm = 30*1000/(millis() - timeold)*half_revolutions;
    timeold = millis();
    half_revolutions = 0;
    Hall_Speed = CurrentfRpm*(.108*3.14159) / 60; //conversion to m/s
    
   // nh.spinOnce();
  //  Serial.println(Hall_Speed);
   }
}


double calc_speed(){
  /*
  if (Error < -50){
    //Input = Input + Error;
    Input = p1*pow(V_desired,4) + p2*pow(V_desired,3) + p3*pow(V_desired,2) + p4*V_desired + p5;
    Input = Input*.99; 
    motor.detach();
  }
  */
 
   if(Error < 0){
   Error = 0;  
 }
      //nh.spinOnce();
      motor.attach(motor_Pin);
      P = pow(Error,2)*Kp;
      Input = Input + P;  // this value needs to be scaled or mapped to the operating range of angles
      if (abs(Input)<min_Velocity) {
      Input = min_Velocity ;
      }
      if (abs(Input)> max_Velocity){
      Input = max_Velocity;
      }
      return Input;
 
}


