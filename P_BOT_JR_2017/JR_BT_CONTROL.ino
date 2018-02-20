//*********************************************//
//                   P_BOT_JR                  //
//               JR_ BT_ANDROID                //
//                                             //
//  This enables a bot to avoid obstacles and  //
//          may also solve mazes.              //
//               Codes by:                     //
//        e-Gizmo Mechatronix Central          //
//         Taft, Manila, Philippines           //
//           http://www.egizmo.com             //
//                DEC.20,2017                  //
//*********************************************//
// INITIALIZATION:
#include <Servo.h>
Servo myservo;
byte INPUT_FROM_ANDROID_APP;

int MOTOR_1_DIRECTION =  8;                // Motor control HIGH = FWD, LOW = BWD
int MOTOR_1_SPEED =      9;                // Speed input ( digitalWrite  LOW or 0 ) means fullspeed.
int MOTOR_2_SPEED =      10;               // Speed input ( digitalWrite  HIGH or 1 ) means fullstop.
int MOTOR_2_DIRECTION =  11;               // Motor control HIGH = FWD, LOW = BWD 

int NORMAL_SPEED = 0;                      // 0 means fullspeed.
int FULL_STOP = 255;                       // 255 means fullstop on analog read.

// INPUTS AND OUTPUTS
void setup()
{
  Serial.begin(9600); 
  myservo.attach(4);

  pinMode(MOTOR_1_DIRECTION, OUTPUT);
  pinMode(MOTOR_1_SPEED, OUTPUT);
  pinMode(MOTOR_2_DIRECTION, OUTPUT);
  pinMode(MOTOR_2_SPEED, OUTPUT);
  myservo.write(90);
    delay(100);
}
// MAIN PROGRAM
void loop() {
  if (Serial.available() > 0) {
    INPUT_FROM_ANDROID_APP = Serial.read();
    Serial.println(INPUT_FROM_ANDROID_APP);
    myservo.write(INPUT_FROM_ANDROID_APP);
  }
  else if (INPUT_FROM_ANDROID_APP==91){
    FORWARD();
  }
  else if (INPUT_FROM_ANDROID_APP==89){
    BACKWARD ();
  }
  else  if (INPUT_FROM_ANDROID_APP==129){
    RIGHT_TURN();
  }
  else if (INPUT_FROM_ANDROID_APP==38){
    LEFT_TURN();
  }
  else if (INPUT_FROM_ANDROID_APP==90){  
    STOP ();
  }
    else {
          STOP ();
    }
}
// PRESET FUNCTIONS 
void FORWARD()    
{
  digitalWrite(MOTOR_1_DIRECTION,HIGH);
  digitalWrite(MOTOR_2_DIRECTION,HIGH);
  analogWrite(MOTOR_1_SPEED,NORMAL_SPEED);
  analogWrite(MOTOR_2_SPEED,NORMAL_SPEED);
}
void STOP()    
{
  analogWrite(MOTOR_1_SPEED,FULL_STOP);
  analogWrite(MOTOR_2_SPEED,FULL_STOP);
  delay(10);
}
void BACKWARD()    
{
  digitalWrite(MOTOR_1_DIRECTION,LOW);
  digitalWrite(MOTOR_2_DIRECTION,LOW);
  analogWrite(MOTOR_1_SPEED,NORMAL_SPEED);
  analogWrite(MOTOR_2_SPEED,NORMAL_SPEED);
  delay(100);
}
void RIGHT_TURN()    
{
  digitalWrite(MOTOR_1_DIRECTION,HIGH);
  digitalWrite(MOTOR_2_DIRECTION,LOW);
  analogWrite(MOTOR_1_SPEED,NORMAL_SPEED);
  analogWrite(MOTOR_2_SPEED,NORMAL_SPEED);
  delay(180);
}
void LEFT_TURN()    
{
  digitalWrite(MOTOR_1_DIRECTION,LOW);
  digitalWrite(MOTOR_2_DIRECTION,HIGH);
  analogWrite(MOTOR_1_SPEED,NORMAL_SPEED);
  analogWrite(MOTOR_2_SPEED,NORMAL_SPEED);
  delay(180);
}
// PROGRAM ENDS AND LOOP BACK FOREVER                                                                                                                                                   // PARZ

