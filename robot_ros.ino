#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>

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
sensor_msgs::Range range_msg;
ros::Publisher pub_range( "ranges", &range_msg);
// unsigned long range_timer;

void setup() {
  //---set pins for motors.
  pinMode(LEFT_ENABLE,OUTPUT);
  pinMode(LEFT_DIRA,OUTPUT);
  pinMode(LEFT_DIRB,OUTPUT);
  pinMode(RIGHT_ENABLE,OUTPUT);
  pinMode(RIGHT_DIRA,OUTPUT);
  pinMode(RIGHT_DIRB,OUTPUT);

  // Set up ROS messaging system
  nh.initNode();
  nh.advertise(pub_range);
  //range_timer =  millis();

  range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg.field_of_view = 0.07; // this is in radians = ~4'
  range_msg.min_range = 0.05;
  range_msg.max_range = 2.0;
}

unsigned long sensor = 0;

void loop() {

// Take one reading at a time
// each measurment takes about 12 + 25ms (due to artifical delay in library)
  long range = 0;
  switch (sensor % 3) {
    case 0: 
     range = sr_left.Distance();
     range_msg.header.frame_id =  "L";
     break;
     
    case 1:
      range = sr_center.Distance();
      range_msg.header.frame_id =  "C";
      break;

    case 2:
      range = sr_right.Distance();
      range_msg.header.frame_id =  "R";
     break;
  }
  ++sensor;

 // publish the reading
  range_msg.range = range/100.0;
  range_msg.header.stamp = nh.now();
  pub_range.publish(&range_msg);

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
   
