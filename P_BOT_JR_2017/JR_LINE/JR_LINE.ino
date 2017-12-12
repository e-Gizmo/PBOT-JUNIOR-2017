//*********************************************//
//                   P_BOT_JR                  //
//                   JR_LINE                   //
//                                             //
// This demo program will  follow a black line //
//  path laid out on a light colored surface   //
//         track width is 15-19mm wide.        //
//                Codes by:                    //
//        e-Gizmo Mechatronix Central          //
//         Taft, Manila, Philippines           //
//           http://www.egizmo.com             //
//                NOV. 24,2017                 //
//*********************************************//

// INITIALIZATION:
int MOTOR_1_DIRECTION =  8;                // Motor control HIGH = FWD, LOW = BWD
int MOTOR_1_SPEED =      9;                // Speed input   LOW or 0 (PWM = 255) fullspeed,  HIGH or 1 (PWM =0) stop.
int MOTOR_2_SPEED =      10;               // Speed input   LOW or 0 (PWM = 255) fullspeed,  HIGH or 1 (PWM =0) stop.
int MOTOR_2_DIRECTION =  11;               // Motor control HIGH = FWD, LOW = BWD

int  LEFT_LINE_SENSOR =   5;
int  CENTER_LINE_SENSOR = 6;
int  RIGHT_LINE_SENSOR =  7;

long int RTC;

// INPUTS AND OUTPUTS
void setup()   {
  Serial.begin(9600);
  // DEFAULT PINS OF THE MOTOR DRIVER (8,9,10,11)  
  pinMode(MOTOR_1_DIRECTION, OUTPUT);
  pinMode(MOTOR_1_SPEED, OUTPUT);
  pinMode(MOTOR_2_SPEED, OUTPUT);
  pinMode(MOTOR_2_DIRECTION, OUTPUT);

  pinMode(13, OUTPUT);
  RTC=millis()+10;
}
int  LINE_SENSE=0;
int GIVE_UP=0;
int LAST_SENSE;
int RUN_SPEED=140;

//TIMERS
byte  RETRY_DELAY=0;
byte  LED_FLASH=25;

// MAIN PROGRAM
void loop()                     
{
  //HARDWARE TIMER SERVICE
  if(millis()>=RTC)
  {
    RTC=RTC+10;
    if(RETRY_DELAY>0) RETRY_DELAY--;
    if(LED_FLASH>0)
    {
      LED_FLASH--;
      if(LED_FLASH==0)
      {
        LED_FLASH=25;
        PORTB ^= 0x20;
      }
    }                 
  }      
  // READ THE STATUS OF THE LINE SENSOR
  LINE_SENSE=0;
  if(digitalRead(LEFT_LINE_SENSOR)==LOW) LINE_SENSE=1;
  if(digitalRead(CENTER_LINE_SENSOR)==LOW) LINE_SENSE=LINE_SENSE+2;
  if(digitalRead(RIGHT_LINE_SENSOR)==LOW) LINE_SENSE=LINE_SENSE+4;  

  // IF NO LINES ARE DETECTED
  if((LINE_SENSE==0) & (RETRY_DELAY==0)){

    // REVERSE FOR 20MS
    if(GIVE_UP<10)
    {
      if(LAST_SENSE==1) RUN_BOT(RUN_SPEED*15/10,RUN_SPEED,LOW);
      if(LAST_SENSE==3) RUN_BOT(RUN_SPEED*12/10,RUN_SPEED,LOW);
      if(LAST_SENSE==4) RUN_BOT(RUN_SPEED,RUN_SPEED*15/10,LOW);
      if(LAST_SENSE==6) RUN_BOT(RUN_SPEED,RUN_SPEED*12/10,LOW);       
      delay(40);
      GIVE_UP++;
    }
  }
  if(LINE_SENSE!=0)
  {
    LAST_SENSE=LINE_SENSE;
    GIVE_UP=0;
    RETRY_DELAY=50;
  }        
  //TURN SOUTH FAST
  if(LINE_SENSE==1)
  {
    FORWARD();
    analogWrite(MOTOR_1_SPEED, RUN_SPEED);
    analogWrite(MOTOR_2_SPEED, 0);
    delay(50);        
  } 
  // GO FORWARD STRAIGTH 
  if((LINE_SENSE==2)|(LINE_SENSE==7))
  {
    RUN_BOT(RUN_SPEED,RUN_SPEED,HIGH);
    delay(50);
  }      
  //TURN SOUTH 
  if(LINE_SENSE==3)
  {
    FORWARD();
    analogWrite(MOTOR_1_SPEED, RUN_SPEED*12/10);
    delay(50);       
  }  
  //TURN NORTH FAST
  if(LINE_SENSE==4)
  {
    FORWARD();
    analogWrite(MOTOR_2_SPEED, RUN_SPEED);
    analogWrite(MOTOR_1_SPEED, 0);
    delay(50);       
  }
  //TURN NORTH
  if(LINE_SENSE==6)
  {
    FORWARD();
    analogWrite(MOTOR_2_SPEED, RUN_SPEED*12/10);
    delay(50);       
  }  
}
// PRESET FUNCTIONS
void RUN_BOT(int spd1,int spd2, boolean direction )
{
  digitalWrite(MOTOR_1_DIRECTION,direction);
  digitalWrite(MOTOR_2_DIRECTION,direction);
  analogWrite(MOTOR_2_SPEED, spd1);
  analogWrite(MOTOR_1_SPEED, spd2); 
}
void FORWARD(void)
{
  digitalWrite(MOTOR_1_DIRECTION,HIGH);
  digitalWrite(MOTOR_2_DIRECTION,HIGH);
}
 // PROGRAM ENDS AND LOOP BACK FOREVER  
//**********************************************************************************//PARZ                                            





