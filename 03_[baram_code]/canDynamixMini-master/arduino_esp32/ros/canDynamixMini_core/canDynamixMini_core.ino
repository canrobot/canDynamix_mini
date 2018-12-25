#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>


#include <WiFi.h> 

#include "canDynamixMini.h"


const char*    ssid     = "";
const char*    password = "";
IPAddress      server(0,0,0,0);            // Set the rosserial socket server IP address
const uint16_t serverPort = 11411;             // Set the rosserial socket server port



#define WHEEL_RADIUS                     0.0165          // meter
#define WHEEL_SEPARATION                 0.086           // meter (canDynamix n20 motor)
#define TURNING_RADIUS                   0.080           // meter (BURGER : 0.080, WAFFLE : 0.1435)
#define ROBOT_RADIUS                     0.105           // meter (BURGER : 0.105, WAFFLE : 0.220)
#define ENCODER_MIN                     -2147483648      // raw
#define ENCODER_MAX                      2147483648      // raw

#define LEFT  0
#define RIGHT 1



#define VELOCITY_CONSTANT_VALUE           300.1090523 // V = r * w = 1400 / 2 * PI * r 
                                                      // = 0.0165 * 0.229 * Goal RPM * 0.10472
                                                      // Goal Speed = V * 1263.632956882

#define DEG2RAD(x)                        (x * 0.01745329252)  // *PI/180
#define RAD2DEG(x)                        (x * 57.2957795131)  // *180/PI
                                                      
// Limit values 
#define LIMIT_X_MAX_VELOCITY              300
#define MAX_LINEAR_VELOCITY               0.22   // m/s
#define MAX_ANGULAR_VELOCITY              2.84 // rad/s




// Callback function prototypes
void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg);

void controlMotorSpeed(void);


/*******************************************************************************
* ROS NodeHandle
*******************************************************************************/
ros::NodeHandle nh;

/*******************************************************************************
* Subscriber
*******************************************************************************/
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", commandVelocityCallback);


/*******************************************************************************
* Publisher
*******************************************************************************/


/*******************************************************************************
* SoftwareTimer of canDynamix
*******************************************************************************/
static uint32_t tTime[4];


/*******************************************************************************
* Declaration for motor
*******************************************************************************/
double goal_linear_velocity  = 0.0;
double goal_angular_velocity = 0.0;


StepMotor step_l(0);
StepMotor step_r(1);
 


void setup() 
{
  Serial.begin(115200);


  // Connect the ESP32 the the wifi AP
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  

  step_l.begin(12, 14, 32, -1);  // enable_pin, step_pin, dir_pin, dir
  step_l.setAcc(1);
  step_l.setSpeed(0);

  step_r.begin(27, 15, 33, 1);   // enable_pin, step_pin, dir_pin, dir
  step_r.setAcc(1);
  step_r.setSpeed(0);

  pinMode(13, OUTPUT);  


  // Set the connection to rosserial socket server
  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();

  // Another way to get IP
  Serial.print("IP = ");
  Serial.println(nh.getHardware()->getLocalIP());

  nh.initNode();  
  nh.subscribe(cmd_vel_sub);
}


void loop()
{
  static uint32_t pre_time;


  if (nh.connected()) 
  {
    // 30Hz
    if ((millis()-tTime[0]) >= (1000 / 30))
    {
      controlMotorSpeed();
      tTime[0] = millis();
    }
  } 
  else 
  {
    step_l.setSpeed(0);
    step_r.setSpeed(0);
  }  

  if (millis()-pre_time >= 10)
  {
    pre_time = millis();
    nh.spinOnce();
  }  
}

/*******************************************************************************
* Callback function for cmd_vel msg
*******************************************************************************/
void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg)
{
  goal_linear_velocity  = cmd_vel_msg.linear.x;
  goal_angular_velocity = cmd_vel_msg.angular.z;

  digitalWrite(13, !digitalRead(13));
}


/*******************************************************************************
* Control motor speed
*******************************************************************************/
void controlMotorSpeed(void)
{
  double wheel_speed_cmd[2];
  double lin_vel_left;
  double lin_vel_right;

  wheel_speed_cmd[LEFT]  = goal_linear_velocity - (goal_angular_velocity * WHEEL_SEPARATION * 10 / 2);
  wheel_speed_cmd[RIGHT] = goal_linear_velocity + (goal_angular_velocity * WHEEL_SEPARATION * 10 / 2);

  lin_vel_left  = constrain(wheel_speed_cmd[LEFT]  * VELOCITY_CONSTANT_VALUE, -LIMIT_X_MAX_VELOCITY, LIMIT_X_MAX_VELOCITY);
  lin_vel_right = constrain(wheel_speed_cmd[RIGHT] * VELOCITY_CONSTANT_VALUE, -LIMIT_X_MAX_VELOCITY, LIMIT_X_MAX_VELOCITY);
  
  Serial.print((int32_t)lin_vel_left);
  Serial.print(" ");
  Serial.println((int32_t)lin_vel_right);
  step_l.setSpeed((int32_t)lin_vel_left);
  step_r.setSpeed((int32_t)lin_vel_right);
}


