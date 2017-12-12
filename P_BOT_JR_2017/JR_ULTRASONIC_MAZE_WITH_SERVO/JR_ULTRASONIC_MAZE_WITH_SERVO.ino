//*********************************************//
//                  P_BOT_JR                   //
//                  JR MAZE                    //
//                                             //
//  This enables a bot to avoid obstacles and  //
//          may also solve mazes.              //
//               Codes by:                     //
//        e-Gizmo Mechatronix Central          //
//         Taft, Manila, Philippines           //
//            http://goo.gl/VyyPvb             //
//                NOV. 24,2017                 //
//*********************************************//

// INITIALIZATION:
#include<Servo.h>
Servo myservo;

int servopin=4;
int trigpin=18;
int echopin=19;
float distance;
float duration;

// INITIALIZATION:
#define MOTOR_1_DIRECTION         8       // Motor control HIGH = FWD, LOW = BWD
#define MOTOR_1_SPEED             9       // Speed input from 0 to 255 (left motor)
#define MOTOR_2_SPEED            10       // Speed input from 0 to 255 (right motor)
#define MOTOR_2_DIRECTION        11       // Motor control HIGH = FWD, LOW = BWD

int NORMAL_SPEED = 50;                    // 255 means fullstop on analog read,0 means fullspeed,feel free to play analog values from 0-255
int FULL_STOP = 255;
/**********************************************************************************/
void setup()
{
  Serial.begin(9600); 

  pinMode(servopin,OUTPUT);
  pinMode(trigpin,OUTPUT);
  pinMode(echopin,INPUT);

  pinMode(MOTOR_1_DIRECTION, OUTPUT);
  pinMode(MOTOR_1_SPEED, OUTPUT);
  pinMode(MOTOR_2_SPEED, OUTPUT);
  pinMode(MOTOR_2_DIRECTION, OUTPUT);
  myservo.attach(servopin); 
}
int ping()
{
  digitalWrite(trigpin,LOW);
  delayMicroseconds(10);
  digitalWrite(trigpin,HIGH);
  delayMicroseconds(10);
  digitalWrite(trigpin,LOW);

  duration= pulseIn(echopin,HIGH);
  distance= duration*0.034/2;
  return(distance);
}
/**********************************************************************************/
void loop()
{ 
  myservo.write(90);
  float value;
  value=ping();
  Serial.println(value);
  delay(10);

  if(value>=20)
  {
    GO_FORWARD();
    delay(100);
  }
  if(value<20 )
  {
    STOP();
    delay(100);
    GO_BACKWARD();
    delay(400);
    STOP();
    delay(100);
    DO_OBSTACLE_AVOIDING();
    delay(200);
  }
  myservo.write(90);
}
void DO_OBSTACLE_AVOIDING ()
{
  int OBSTACLE_ON_LEFT;
  int OBSTACLE_ON_RIGHT;
  myservo.write(10);
  delay(500);
  OBSTACLE_ON_LEFT = ping();
  myservo.write(170);
  delay(500);
  OBSTACLE_ON_RIGHT = ping();
  if(OBSTACLE_ON_LEFT > OBSTACLE_ON_RIGHT)
  {
    SPIN_RIGHT();
    delay(100);     
  }
  else if(OBSTACLE_ON_LEFT < OBSTACLE_ON_RIGHT)
  {
    SPIN_LEFT();
    delay(100);    
  }
  else if(OBSTACLE_ON_LEFT = OBSTACLE_ON_RIGHT)
  {
    GO_BACKWARD();
    delay(400);
    SPIN_RIGHT();
    delay(100);
  }
}
// MOTOR CONTROL FUNCTION
void GO_FORWARD()
{
  digitalWrite(MOTOR_1_DIRECTION, HIGH); 
  digitalWrite(MOTOR_2_DIRECTION, HIGH);
  analogWrite(MOTOR_1_SPEED,NORMAL_SPEED);
  analogWrite(MOTOR_2_SPEED,NORMAL_SPEED);
}
/*---------------------------*/
void GO_BACKWARD()
{
  digitalWrite(MOTOR_1_DIRECTION,LOW);
  digitalWrite(MOTOR_2_DIRECTION,LOW);
  analogWrite(MOTOR_1_SPEED,NORMAL_SPEED);
  analogWrite(MOTOR_2_SPEED,NORMAL_SPEED);
}
/*---------------------------*/
void STOP()
{
  analogWrite(MOTOR_1_SPEED,FULL_STOP);
  analogWrite(MOTOR_2_SPEED,FULL_STOP); 
}
/*---------------------------*/
void SPIN_LEFT()
{
  digitalWrite(MOTOR_1_DIRECTION,LOW);
  digitalWrite(MOTOR_2_DIRECTION,HIGH);
  analogWrite(MOTOR_1_SPEED,NORMAL_SPEED);
  analogWrite(MOTOR_2_SPEED,NORMAL_SPEED);
}
/*---------------------------*/
void SPIN_RIGHT()
{
  digitalWrite(MOTOR_1_DIRECTION,HIGH);
  digitalWrite(MOTOR_2_DIRECTION,LOW);
  analogWrite(MOTOR_1_SPEED,NORMAL_SPEED);
  analogWrite(MOTOR_2_SPEED,NORMAL_SPEED);
}
 // PROGRAM ENDS AND LOOP BACK FOREVER  
//**********************************************************************************//PARZ

