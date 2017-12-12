//*********************************************//
//                  P_BOT_JR                   //
//               PS2_CONTROLLED                //
//                                             //
//         using ps2 controller kit and        //
//           (uhf rx only) reciever            //
//                 Codes by:                   //
//        e-Gizmo Mechatronix Central          //
//         Taft, Manila, Philippines           //
//           http://goo.gl/YDPmJC              //
//                SEP 22,2017                  //
//*********************************************//
//*************************************************************************************************************************************************************************************************************************************************************************.
// INITIALIZATION:
#include <Servo.h>
Servo myservo1;
#define MOTOR_1_DIRECTION         8       // Motor control HIGH = FWD, LOW = BWD
#define MOTOR_1_SPEED             9       // Speed input
#define MOTOR_2_SPEED            10       // Speed input
#define MOTOR_2_DIRECTION        11       // Motor control HIGH = FWD, LOW = BWDD
#define SER_SYNC1 0
#define SER_SYNC2 1
#define SER_FETCHDATA 2

int pos1 = 90;
int val0;
int valx0;
int NORMAL_SPEED = 0;                     // 0 means fullspeed,feel free to play analog values from 0-255
int FULL_STOP = 255;                      // 255 means fullstop on analog read
char SERIAL_DATA;

byte  inputData[15];
byte stage;
byte rxctr;
byte  msg;
byte serial_rxcounter=0;
byte serial_state=0;

boolean serial_available = false;
//boolean CLR = false;

float left_x;
float left_y;
float right_x;
float right_y;

typedef  struct cont cont;
struct cont
{
  boolean  up;
  boolean  down;
  boolean  left;
  boolean  right;
  boolean  triangle;
  boolean  circle;
  boolean  cross;
  boolean  square;
  boolean  left1;
  boolean  left2;
  boolean  right1;
  boolean  right2;
  boolean  start;
  byte  leftx;
  byte  lefty;
  byte  rightx;
  byte  righty;
};
cont  controller;
//*************************************************************************************************************************************************************************************************************************************************************************
// INPUTS AND OUTPUTS
void setup(){
  int i;
  Serial.begin(9600);
    myservo1.attach(4);
      delay(1000);

  pinMode (MOTOR_1_DIRECTION,OUTPUT);
  pinMode (MOTOR_1_SPEED,OUTPUT);
  pinMode (MOTOR_2_SPEED,OUTPUT);
  pinMode (MOTOR_2_DIRECTION,OUTPUT);

}

//*************************************************************************************************************************************************************************************************************************************************************************
// MAIN PROGRAM
void  loop(void)
{
  if (serial_available==true) {
    serial_available = false;
    serial_state=SER_SYNC1;

    left_x = controller.leftx;
    left_y = controller.lefty;
    
        right_x = controller.rightx;
    right_y = controller.righty;
    
        valx0 = controller.rightx;
    val0 = map(valx0, 128, 255, 90, 170);
    myservo1.write(val0);
    
    //LEFT JOYSTICK
    if(controller.lefty == 0){ 
      GO_FORWARD ();
    }
    else if(controller.lefty == 128){
      GO_STOP ();
    }
    else if(controller.lefty == 255){
      GO_BACKWARD ();
    }
    //*************************************************
    else if(controller.leftx == 0){
      LEFT_TURN ();
    }
    else if(controller.leftx == 128){
      GO_STOP ();    
    }
    else if(controller.leftx == 255){
      RIGHT_TURN ();
    }
    else{
      GO_STOP ();
    }
  }
}

    


          
//*************************************************************************************************************************************************************************************************************************************************************************
// PRESET FUNCTIONS:
// MOTOR ROUTINE:: 
void GO_FORWARD()    
{
  digitalWrite(MOTOR_1_DIRECTION,HIGH);
  digitalWrite(MOTOR_2_DIRECTION,HIGH);
  digitalWrite(MOTOR_1_SPEED,LOW);
  digitalWrite(MOTOR_2_SPEED,LOW);
}
void GO_STOP()    
{
  digitalWrite(MOTOR_1_SPEED,HIGH);
  digitalWrite(MOTOR_2_SPEED,HIGH);
}
void GO_BACKWARD()    
{
  digitalWrite(MOTOR_1_DIRECTION,LOW);
  digitalWrite(MOTOR_2_DIRECTION,LOW);
  digitalWrite(MOTOR_1_SPEED,LOW);
  digitalWrite(MOTOR_2_SPEED,LOW);
}
void RIGHT_TURN ()    
{
  digitalWrite(MOTOR_1_DIRECTION,HIGH);
  digitalWrite(MOTOR_2_DIRECTION,LOW);
  digitalWrite(MOTOR_1_SPEED,LOW);
  digitalWrite(MOTOR_2_SPEED,HIGH);
}
void LEFT_TURN ()    
{
  digitalWrite(MOTOR_1_DIRECTION,LOW);
  digitalWrite(MOTOR_2_DIRECTION,HIGH);
  digitalWrite(MOTOR_1_SPEED,HIGH);
  digitalWrite(MOTOR_2_SPEED,LOW);
}
//*************************************************************************************************************************************************************************************************************************************************************************
/*     Do not modify anything past this line
 Unless you are absolutely sure you know
 what you are doing
 
 SerialEvent occurs whenever a new data comes in the
 hardware serial RX.  This routine is run between each
 time loop() runs, so using delay inside loop can delay
 response.  Multiple bytes of data may be available.
 */
void serialEvent() {
  while (Serial.available()) {
    byte inChar = (byte)Serial.read(); 
    switch(serial_state){
    case SER_SYNC1:

      serial_rxcounter=0;
      if(inChar==0x5A)serial_state=SER_SYNC2;
      break;
    case SER_SYNC2:
      if(inChar==0xA5) 
        serial_state=SER_FETCHDATA;
      else
        serial_state=SER_SYNC1;
      break;
    case  SER_FETCHDATA:
      inputData[serial_rxcounter]= inChar;
      serial_rxcounter++;
      if(serial_rxcounter>=10){
        if(checksum()==inputData[9]){
          refresh_controller();
          serial_available=true;   
        }
        else{    
          serial_state=SER_SYNC1;
          serial_available=false;
        }
      }
    }
  }
}
//*************************************************************************************************************************************************************************************************************************************************************************
/* Translate raw gamepad controller data
 into something more meaningful for you
 */
void  refresh_controller(void){
  controller.up=false;
  if((inputData[3]&0x08)==0) controller.up=true;
  controller.down=false;
  if((inputData[3]&0x02)==0) controller.down=true;
  controller.left=false;
  if((inputData[3]&0x01)==0) controller.left=true;
  controller.right=false;
  if((inputData[3]&0x04)==0) controller.right=true;
  controller.triangle=false;
  if((inputData[4]&0x08)==0) controller.triangle=true;
  controller.circle=false;
  if((inputData[4]&0x04)==0) controller.circle=true;
  controller.cross=false;
  if((inputData[4]&0x02)==0) controller.cross=true;
  controller.square=false;
  if((inputData[4]&0x01)==0) controller.square=true;
  controller.left1=false;
  if((inputData[4]&0x20)==0) controller.left1=true;
  controller.left2=false;
  if((inputData[4]&0x80)==0) controller.left2=true;
  controller.right1=false;
  if((inputData[4]&0x10)==0) controller.right1=true;
  controller.right2=false;
  if((inputData[4]&0x40)==0) controller.right2=true;
  controller.start=false;
  if((inputData[3]&0x10)==0) controller.start=true;
  controller.leftx=inputData[7];
  controller.lefty=inputData[8];
  controller.rightx=inputData[5];
  controller.righty=inputData[6];
}
byte  checksum(void){
  int  i;
  byte chk= 0x5A+0xA5;
  for(i=0;i<9;i++) chk += inputData[i];
  return(chk);
}
// PROGRAM ENDS AND LOOP BACK FOREVER
//*********************************************************************************//PARZ
















