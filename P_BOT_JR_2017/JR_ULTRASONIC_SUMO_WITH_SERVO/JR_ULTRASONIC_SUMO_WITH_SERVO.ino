//*********************************************//
//                    P_BOT_JR                 //
//                    JR ZUMO                  //
//                                             //
//  This enables a bot to ramp the enemy and   //
//          may also avoid outside line.       //
//               Codes by:                     //
//        e-Gizmo Mechatronix Central          //
//         Taft, Manila, Philippines           //
//            http://goo.gl/VyyPvb             //
//                NOV. 24,2017                 //
//*********************************************//

// INITIALIZATION:
#include<Servo.h>
Servo MYSERVO;

int SERVOPIN=4;
int TRIGPIN=18;
int ECHOPIN=19;
float DISTANCE;
float DURATION;

// INITIALIZATION:
#define MOTOR_1_DIRECTION         8       // Motor control HIGH = FWD, LOW = BWD
#define MOTOR_1_SPEED             9       // Speed input from 0 to 255 (left motor)
#define MOTOR_2_SPEED            10       // Speed input from 0 to 255 (right motor)
#define MOTOR_2_DIRECTION        11       // Motor control HIGH = FWD, LOW = BWD

#define LEFT_LINE_SENSOR          5       // Line sensor left 
#define CENTER_LINE_SENSOR        6       // Line sensor center
#define RIGHT_LINE_SENSOR         7       // Line sensor right

int NORMAL_SPEED = 50;                    // 255 means fullstop on analog read,0 means fullspeed,feel free to play analog values from 0-255
int FULL_STOP = 255;
int LINER = 0;
int LINERS = 0;
boolean LINE = false;
boolean GO = false;
/**********************************************************************************/
void setup()
{
  Serial.begin(9600); 

  pinMode(SERVOPIN,OUTPUT);
  pinMode(TRIGPIN,OUTPUT);
  pinMode(ECHOPIN,INPUT);

  pinMode(MOTOR_1_DIRECTION, OUTPUT);
  pinMode(MOTOR_1_SPEED, OUTPUT);
  pinMode(MOTOR_2_SPEED, OUTPUT);
  pinMode(MOTOR_2_DIRECTION, OUTPUT);
  MYSERVO.attach(SERVOPIN);

  pinMode(LEFT_LINE_SENSOR, INPUT_PULLUP);
  pinMode(CENTER_LINE_SENSOR, INPUT_PULLUP);
  pinMode(RIGHT_LINE_SENSOR, INPUT_PULLUP);

}
int ping()
{
  digitalWrite(TRIGPIN,LOW);
  delayMicroseconds(10);
  digitalWrite(TRIGPIN,HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGPIN,LOW);

  DURATION= pulseIn(ECHOPIN,HIGH);
  DISTANCE= DURATION*0.034/2;
  return(DISTANCE);
}
/**********************************************************************************/
void loop()
{ 
  int LINE_SENSE_LEFT =      digitalRead(LEFT_LINE_SENSOR);
  int LINE_SENSE_CENTER =    digitalRead(CENTER_LINE_SENSOR);
  int LINE_SENSE_RIGHT =     digitalRead(RIGHT_LINE_SENSOR);

  float VALUE;
  VALUE=ping();                // value ng ultrasonic
  delay(10);

  // If line is detected:
  if((LINE_SENSE_LEFT == 0|| LINE_SENSE_CENTER == 0) || LINE_SENSE_RIGHT == 0)
  {
    LINER-=1;
    //LINERS-=1;
  }

  // If there is no line detected and there is no enemy
  while(LINER == 1 && VALUE > 20)
  {
    STOP();                     // pag less than 20cm,search mode
    delay(200);
    LOOKING_FOR_OPPONENT();
    delay(200);
    break;
  }
  // If there is an enemy and there is no line detected
  while((VALUE < 20 && VALUE > 0) && LINER == 1) 
  {
    ATTACK();                   // pag greater than 20cm,attack mode
    delay(1000);
    break;
  }

  // Turn around if line is detected:
  if(LINER == 0)
  {
    SPIN_LEFT();
    delay(500);
    GO_FORWARD_SLOW();
  }
  LINER = 1;

  MYSERVO.write(90);
}

/*---------------------------*/
void LOOKING_FOR_OPPONENT ()
{
  int OPPONENT_ON_LEFT;
  int OPPONENT_ON_CENTER;
  int OPPONENT_ON_RIGHT;
  MYSERVO.write(10);
  delay(500);
  OPPONENT_ON_RIGHT = ping();
  WHILE_SEARCHING_LINE_DETECTED();
  MYSERVO.write(170);
  delay(500);
  OPPONENT_ON_LEFT = ping();
  WHILE_SEARCHING_LINE_DETECTED();
  MYSERVO.write(90);
  delay(500);
  OPPONENT_ON_CENTER= ping();
  WHILE_SEARCHING_LINE_DETECTED();
  /*---------------------------*/
  if(OPPONENT_ON_RIGHT <= 20 && LINER == 1 )
  {
    STOP();  
    GO = true;
    if(OPPONENT_ON_RIGHT <=20 && GO == true){
      SPIN_RIGHT();
      delay(300); 
      ATTACK();  
      delay(500);
      GO = false;
    }
  }
  /*---------------------------*/
  else if(OPPONENT_ON_LEFT <= 20 && LINER == 1)
  { 
    SPIN_LEFT();
    delay(300);
    ATTACK();                                       
    delay(500);
  }
  /*---------------------------*/
  else if(OPPONENT_ON_LEFT = OPPONENT_ON_RIGHT)
  {
    GO_FORWARD_SLOW();
  }
}
/*---------------------------*/
// MOTOR CONTROL FUNCTION
void ATTACK()
{
  digitalWrite(MOTOR_1_DIRECTION,HIGH);
  digitalWrite(MOTOR_2_DIRECTION,HIGH);
  analogWrite(MOTOR_1_SPEED,0);
  analogWrite(MOTOR_2_SPEED,0);

}
/*---------------------------*/
void GO_FORWARD_SLOW()
{
  digitalWrite(MOTOR_1_DIRECTION, HIGH); 
  digitalWrite(MOTOR_2_DIRECTION, HIGH);
  analogWrite(MOTOR_1_SPEED,180);
  analogWrite(MOTOR_2_SPEED,180);
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
  digitalWrite(MOTOR_1_DIRECTION,LOW);
  digitalWrite(MOTOR_2_DIRECTION,LOW);
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
//**********************************************************************************
void WHILE_SEARCHING_LINE_DETECTED(){
  int LINE_SENSE_LEFT =      digitalRead(LEFT_LINE_SENSOR);
  int LINE_SENSE_CENTER =    digitalRead(CENTER_LINE_SENSOR);
  int LINE_SENSE_RIGHT =     digitalRead(RIGHT_LINE_SENSOR);
    if((LINE_SENSE_LEFT == 0|| LINE_SENSE_CENTER == 0) || LINE_SENSE_RIGHT == 0)
  {
    LINER-=1;
  }
  if(LINER == 0){
    GO_BACKWARD();
    delay(500);
    SPIN_LEFT();
    delay(500);
    GO_FORWARD_SLOW();
    STOP();
  }
  LINER = 1;
  
}
 // PROGRAM ENDS AND LOOP BACK FOREVER  
//**********************************************************************************//PARZ










