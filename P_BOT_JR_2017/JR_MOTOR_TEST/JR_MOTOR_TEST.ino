//*********************************************//
//                  P_BOT_JR                   //
//                 MOTOR TEST                  //
//         THE MOTOR ROUTINE SHOULD BE         //
//                  FORWARD                    //
//                 BACKWARD                    //
//               LEFT FORWARD                  //
//               RIGHT FORWARD                 //
//                 Codes by:                   //
//        e-Gizmo Mechatronix Central          //
//         Taft, Manila, Philippines           //
//           http://www.egizmo.com             //
//                NOV. 24,2017                 //
//*********************************************//

// INITIALIZATION:
int MOTOR_1_DIRECTION =  8;                // Motor control HIGH = FWD, LOW = BWD
int MOTOR_1_SPEED =      9;                // Speed input ( digitalWrite  LOW or 0 ) means fullspeed.
int MOTOR_2_SPEED =      10;               // Speed input ( digitalWrite  HIGH or 1 ) means fullstop.
int MOTOR_2_DIRECTION =  11;               // Motor control HIGH = FWD, LOW = BWD

int NORMAL_SPEED = 150;                    // 255 means fullstop on analog read,0 means fullspeed,feel free to play analog values from 0-255
int FULL_STOP = 255;
int PAUSE = 3000;
int PAUSED = 500;

// INPUTS AND OUTPUTS
void setup() 
{  
  // DEFAULT PINS OF THE MOTOR DRIVER (8,9,10,11)
  pinMode(MOTOR_1_DIRECTION, OUTPUT);       
  pinMode(MOTOR_1_SPEED, OUTPUT);
  pinMode(MOTOR_2_SPEED, OUTPUT);
  pinMode(MOTOR_2_DIRECTION, OUTPUT);
}

// MAIN PROGRAM
void loop()  {
  FORWARD();
  delay(PAUSE);
  STOP();
  delay(PAUSED);
  BACKWARD();
  delay(PAUSE);
  STOP();
  delay(PAUSED);

  RIGHT_TURN();
  delay(PAUSE);
  STOP();
  delay(PAUSED);

  LEFT_TURN();
  delay(PAUSE);
  STOP();
  delay(PAUSED);
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
}

void BACKWARD()    
{
  digitalWrite(MOTOR_1_DIRECTION,LOW);
  digitalWrite(MOTOR_2_DIRECTION,LOW);
  analogWrite(MOTOR_1_SPEED,NORMAL_SPEED);
  analogWrite(MOTOR_2_SPEED,NORMAL_SPEED);
}

void RIGHT_TURN()    
{
  digitalWrite(MOTOR_1_DIRECTION,HIGH);
  digitalWrite(MOTOR_2_DIRECTION,LOW);
  analogWrite(MOTOR_1_SPEED,NORMAL_SPEED);
  analogWrite(MOTOR_2_SPEED,255);
}

void LEFT_TURN()    
{
  digitalWrite(MOTOR_1_DIRECTION,LOW);
  digitalWrite(MOTOR_2_DIRECTION,HIGH);
  analogWrite(MOTOR_1_SPEED,255);
  analogWrite(MOTOR_2_SPEED,NORMAL_SPEED);
}
 // PROGRAM ENDS AND LOOP BACK FOREVER  
//**********************************************************************************//PARZ





