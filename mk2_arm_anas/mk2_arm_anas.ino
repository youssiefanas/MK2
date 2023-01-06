#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include<geometry_msgs/Vector3.h>

#include <AccelStepper.h>
#include <MultiStepper.h>
#include <Servo.h>


#define XSTEP 2 //Stepper Motor Step pin
#define YSTEP 3
#define ZSTEP 4
#define XDIR 5 // Stepper motor Direction control pin
#define YDIR 6
#define ZDIR 7
#define ENABLE 8 // CNC Shield Enable Pin
#define XLIMIT 9 // Limit switch pins
#define YLIMIT 10
#define ZLIMIT 11
#define MOTORACC 100 // Acceleration and Max Speed values
#define MOTORMAXSPEED 100

AccelStepper XMOTOR(1,XSTEP,XDIR);
AccelStepper YMOTOR(1,YSTEP,YDIR);
AccelStepper ZMOTOR(1,ZSTEP,ZDIR);
MultiStepper steppers;
Servo gripper;

ros::NodeHandle nh;
geometry_msgs::Vector3 pub_position;
std_msgs::String home_pose;
std_msgs::Int16 gri;


int stepper_base;
int stepper_shoulder;
int stepper_elbow;
int servo_gripper;

  
void steppermsg(const geometry_msgs::Vector3& theta){
  int stepper_base= theta.x;
  int stepper_shoulder= theta.y;
  int stepper_elbow=theta.z;
  pub_position.x=theta.x;
  pub_position.y=theta.y;
  pub_position.z=theta.z;
}

void grippermsg(const std_msgs::Int16& gripper){
 servo_gripper= gripper.data;
 gri.data=gripper.data;
 
}

ros::Subscriber<geometry_msgs::Vector3> joint_move("joints", &steppermsg);
ros::Subscriber<std_msgs::Int16> gripper_move("/servo_gripper", &grippermsg);
ros::Publisher getreadings("getreadings",&pub_position);
ros::Publisher home_pos("/home_pos", &home_pose);

ros::Publisher gripper_reading("/gripper_read",&gri);
char at_home[] = "Reached home Position";



void setup() {
  gripper.attach(12);
  Serial.begin(57600);
  pinMode(ENABLE,OUTPUT);
  digitalWrite(ENABLE,LOW);
  pinMode(XLIMIT,INPUT_PULLUP);
  pinMode(YLIMIT,INPUT_PULLUP);
  pinMode(ZLIMIT,INPUT_PULLUP);
 
  XMOTOR.setMaxSpeed(MOTORMAXSPEED);
  XMOTOR.setAcceleration(MOTORACC);

  YMOTOR.setMaxSpeed(MOTORMAXSPEED);
  YMOTOR.setAcceleration(MOTORACC);

  ZMOTOR.setMaxSpeed(MOTORMAXSPEED);
  ZMOTOR.setAcceleration(MOTORACC);
  ZMOTOR.setCurrentPosition(0);
 home();

  steppers.addStepper(XMOTOR);
  steppers.addStepper(YMOTOR);
  steppers.addStepper(ZMOTOR);

  nh.initNode();
  nh.advertise(getreadings);
  nh.advertise(gripper_reading);
  nh.subscribe(joint_move);
  nh.advertise(home_pos);
  nh.subscribe(gripper_move);



}

void loop() {
  
 gripper.write(gri.data);
   long positions [3];

  positions[0] = pub_position.x;
  positions[1] = pub_position.y;
  positions[2] = pub_position.z;


while (digitalRead(XLIMIT) && digitalRead(YLIMIT)){
      steppers.moveTo(positions);
      steppers.run();
 
   }
   
  delay(1000);
//  gripper.write(min_gripper);


  getreadings.publish(&pub_position);
  gripper_reading.publish(&gri);


nh.spinOnce();
delay(200);

}

void home(){
    int i = 1;
  while (digitalRead(XLIMIT))
{
    XMOTOR.moveTo(i);
    i++;
    XMOTOR.run();
    delay(5);
}
  XMOTOR.setCurrentPosition(0);
  Serial.print("Shoulder is at home Positon ");
  Serial.println(XMOTOR.currentPosition());

  
  i = 1;
  while (digitalRead(YLIMIT))
{
    YMOTOR.moveTo(i);
    i++;
    YMOTOR.run();
    delay(5);
}
  YMOTOR.setCurrentPosition(0);
  Serial.print("Elbow is at Home Position ");
  Serial.println(YMOTOR.currentPosition());
  home_pose.data = at_home;
  home_pos.publish(&home_pose);
}
