#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/LaserScan.h>
#include <math.h>

#include <SR04.h>


/************************
Exercise the motor using
the L293D chip
************************/
// Motor pins
#define LEFT_ENABLE 3
#define LEFT_DIRA 4
#define LEFT_DIRB 2

// Motor pins
#define RIGHT_ENABLE 5
#define RIGHT_DIRA 6
#define RIGHT_DIRB 7

// SR04 sensor pins
#define TRIG_RIGHT 9
#define ECHO_RIGHT 8

#define ECHO_CENTER 10
#define TRIG_CENTER 11

#define TRIG_LEFT 13
#define ECHO_LEFT 12

// Right and Left sensors are at 45' 
SR04 sr_center(ECHO_CENTER,TRIG_CENTER);
SR04 sr_right(ECHO_RIGHT,TRIG_RIGHT);
SR04 sr_left(ECHO_LEFT,TRIG_LEFT);

enum motor_direction {
  BACK,
  FORWARD
};

enum side {
  LEFT,
  RIGHT
};

void set_direction(side s, motor_direction d)
{
    const int dira = (s == LEFT) ? LEFT_DIRA : RIGHT_DIRA;
    const int dirb = (s == LEFT) ? LEFT_DIRB : RIGHT_DIRB;
    digitalWrite(dira, (d == BACK) ? HIGH : LOW); //one way
    digitalWrite(dirb, (d == BACK) ? LOW : HIGH);
}

void move(motor_direction d, int power = 128)
{
   analogWrite(LEFT_ENABLE, power); // enable on
   analogWrite(RIGHT_ENABLE, power); // enable on
   set_direction(LEFT, d);
   set_direction(RIGHT, d);
}

void stop()
{
  digitalWrite(LEFT_ENABLE, LOW);
  digitalWrite(RIGHT_ENABLE, LOW);
}

// angle: -180 : 180 degrees
void turn(int angle, int power = 64)
{
   analogWrite(LEFT_ENABLE, power);
   analogWrite(RIGHT_ENABLE, power);
   set_direction(LEFT, (angle < 0) ? BACK : FORWARD);
   set_direction(RIGHT,(angle > 0) ? FORWARD : BACK);
   delay(abs(angle));
   stop();
}

// ROS stuff
ros::NodeHandle  nh;
sensor_msgs::LaserScan scan;
ros::Publisher pub_scan( "laser_scan", &scan);
const int num_ranges = 3; // number of range sensors 

void setup() {
  //---set pins for motors.
  pinMode(LEFT_ENABLE,OUTPUT);
  pinMode(LEFT_DIRA,OUTPUT);
  pinMode(LEFT_DIRB,OUTPUT);
  pinMode(RIGHT_ENABLE,OUTPUT);
  pinMode(RIGHT_DIRA,OUTPUT);
  pinMode(RIGHT_DIRB,OUTPUT);

  // Set up ROS messaging system
  //nh.getHardware()->SetBaud(500000);
  nh.initNode();
  nh.advertise(pub_scan);

    // We pretend we have a laser scanner that sans -45 to +45 degrees with 3 measurment points
    const float radians_for_45_degrees  = PI/4.0;
    scan.header.frame_id = "laser_frame";
    scan.angle_min = -radians_for_45_degrees;
    scan.angle_max = radians_for_45_degrees;
    scan.angle_increment = radians_for_45_degrees;
    scan.time_increment = 0.025;    // time btw scans => 25ms fixed
    scan.range_min = 0.04;
    scan.range_max = 2.0; 
}



void loop() {

// Take one reading at a time
// each measurment takes about 12 + 25ms (due to artifical delay in library)

// local storage becase scan msg uses a pointer and we dont want to alloc!
  float ranges[num_ranges];
 
  for (int i = 0; i < num_ranges; ++i) 
  {
    long range = 0;
    switch (i % 3) {
      case 0: 
       range = sr_left.Distance();
       break;
       
      case 1:
        range = sr_center.Distance();
        break;
  
      case 2:
        range = sr_right.Distance();
       break;
    }

    ranges[i] = range/100.0;
  }

  scan.header.stamp = nh.now();
  scan.ranges = ranges;
  scan.ranges_length = num_ranges;
  
   // publish the reading
  pub_scan.publish(&scan);

  nh.spinOnce();
  
/* MOTOR CONTROL 
   if (center > 0 && center < 10) 
   {
      stop();
      Serial.println("STOP!");
   }
   else 
   {
      move(FORWARD);
      delay(500);
      
      //turn(90);
      //delay(1000);
   }

   Serial.print("left:");
   Serial.print(left);
   Serial.print(" center:");
   Serial.print(center);
   Serial.print(" right:");
   Serial.println(right);
   */
}
   
