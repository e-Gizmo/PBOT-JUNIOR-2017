//********************************************//
//              JUNIOR WITH EGRA              //
//               PS2 CONTROLLED               //
//                                            //
//  A complete code to make you controls      //
// the robot wirelessly using PS2 controller  //
// with built-in UHF STD transmitter paired   //
// to UHF STD RX only (Receiver) place on the //
// PBOT JR board.                             //
//                                            //
// Controls:                                  //
//  MOTORS                                    //
//    FORWARD    / JS LEFT Y   = UP           //
//    BACKWARD   / JS LEFT Y   = DOWN         //
//    TURN LEFT  / JS LEFT X   = LEFT         //
//    TURN RIGHT / JS LEFT X   = RIGHT        //
//                                            //
//  EGRA GRIPPER (17)                         //
//    RELEASE   = L2                          //
//    GRAB/HOLD = L1                          //
//                                            //
//  EGRA ELBOW (16)                           //
//    RETRACT = R1                            //
//    EXTEND  = R2                            //
//                                            //
//  EGRA SHOULDER (15)                        //
//    TURN UP   = TRIANGLE                    //
//    TURN DOWN = CROSS                       //
//                                            //
//  EGRA BASE (14)                            //
//    TURN RIGHT = CIRCLE                     //
//    TURN LEFT  = SQUARE                     // 
//                                            //
//  RESERVED                                  //
//    RIGHT JOYSTICK                          //
//      JS RIGHT Y & JS RIGHT X               //
//    START BUTTONS                           //
//                                            //
//                 Codes by:                  //
//         e-Gizmo Mechatronix Central        //
//            http://goo.gl/JNRpjx            //
//                 DEC.12, 2017               //
//********************************************//
#include <Servo.h>

byte serial_rxcounter=0;
byte serial_state=0;
boolean serial_available = false;

#define SER_SYNC1 0
#define SER_SYNC2 1
#define SER_FETCHDATA 2

byte  inputData[15];

typedef  struct cont cont;
struct cont{
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

//SERVOS
Servo GRIPPER;
Servo ELBOW;
Servo SHOULDER;
Servo BASE;
//INITIAL POSITIONS (MUST CHANGE)
// TO GET THE EXACT CALIBRATION
int G_POS = 150;
int E_POS = 40;
int S_POS = 40;
int B_POS = 80;


//MOTOR PINS ASSIGNMENT
#define MOTOR_1_DIRECTION 8
#define MOTOR_1_SPEED 9
#define MOTOR_2_SPEED 10
#define MOTOR_2_DIRECTION 11

int RUNSPEED = 180;
int FULL_STOP = 255;
int DEGREES = 10;

void setup() {

  Serial.begin(9600);
  Serial.println("E-GZIMO PBOT JUNIOR WITH E-GRA (PS2 CONTROLLED)");

  GRIPPER.attach(17);
  ELBOW.attach(16);
  SHOULDER.attach(15);
  BASE.attach(14);

  pinMode(MOTOR_1_DIRECTION, OUTPUT);
  pinMode(MOTOR_1_SPEED, OUTPUT);
  pinMode(MOTOR_2_SPEED, OUTPUT);
  pinMode(MOTOR_2_DIRECTION, OUTPUT);

  //MOTOR ALL OFF
  digitalWrite(MOTOR_1_DIRECTION, 1);
  digitalWrite(MOTOR_1_SPEED, 1);
  digitalWrite(MOTOR_2_SPEED, 1);
  digitalWrite(MOTOR_2_DIRECTION, 1);

  //E-GRA INITIAL POSITIONS (MUST CHANGE)
  GRIPPER.write(100);
  delay(1000);
  GRIPPER.write(G_POS);
  ELBOW.write(E_POS);
  SHOULDER.write(S_POS);
  BASE.write(B_POS);
  delay(1000);

}

void loop() {
  if (serial_available==true) {
    serial_available = false;
    serial_state=SER_SYNC1;

    //>>>>>>>>>>>> PS CONTROLLER FUNTIONS <<<<<<<<<<<<<//
    //        >>>>>  CONTROL STICK <<<                 //

     // >>> SHAPE BUTTONS <<< //
   if(controller.triangle==true){ // SHOULDER TURN UP
      S_POS = S_POS+DEGREES;
      if(S_POS>=150){
        S_POS=150;
      }
      SHOULDER.write(S_POS);
    } 
    else if(controller.circle==true){ // BASE TURN RIGHT
      B_POS = B_POS-DEGREES;
      if(B_POS<=10){
        B_POS=10;
      }
      BASE.write(B_POS);

    } 
    else if(controller.cross==true){ // SHOULDER TURN DOWN
      S_POS = S_POS-DEGREES;
      if(S_POS<=40){
        S_POS=40;
      }
      SHOULDER.write(S_POS);
    } 
    else if(controller.square==true){ // BASE TURN LEFT
      B_POS = B_POS+DEGREES;
      if(B_POS>=160){
        B_POS=160;
      }
      BASE.write(B_POS);
    }

    // >>> LEFT L1 / L2  & RIGHT  R1 / R2 <<< //
    else if(controller.left1==true){ // GRIPPER GRAB/HOLD THE OBJECT   
      G_POS = G_POS+DEGREES;
      if(G_POS>=150){
        G_POS=150; 
      }
      GRIPPER.write(G_POS);
    }
    else if(controller.left2==true){ // GRIPPER RELEASE THE OBJECT
      G_POS = G_POS-DEGREES;
      if(G_POS<=100){
        G_POS=100;
      } 
      GRIPPER.write(G_POS); 
    }  
    else if(controller.right1 == true){ // ELBOW RETRACT
      E_POS = E_POS+DEGREES;
      if(E_POS>=150){
        E_POS=150; 
      }
      ELBOW.write(E_POS);
    }
    else if(controller.right2 == true){// ELBOW EXTEND
      E_POS = E_POS-DEGREES;
      if(E_POS<=40){
        E_POS=40; 
      }
      ELBOW.write(E_POS);
    }
    
    // >>>  + CONTROL PAD  <<< //
    else if(controller.up==true) // FORWARD
    { 
      MOVE_FORWARD();
      controller.up = false;
    }
    else if(controller.right==true){ // TURN RIGHT
      TURN_RIGHT();
      controller.right = false;
    } 
    else if(controller.down==true){ // BACKWARD
      MOVE_BACKWARD();
      controller.down = false;
    } 
    else if(controller.left==true){  // TURN LEFT
      TURN_LEFT();
      controller.left = false;
    }

    // >>> RIGHT JOYSTICK <<< // RESERVED  
    //controller.rightx;
    //controller.righty;

    // >>> LEFT JOYSTICK <<< //
    else if(controller.lefty == 0){
      MOVE_FORWARD();
    }
    else if(controller.lefty == 128){
      STOP();
    }
    else if(controller.lefty == 255){
      MOVE_BACKWARD();
    }
    else if(controller.leftx == 0){
      TURN_LEFT();
    }
    else if(controller.leftx == 128){
      STOP();    
    }
    else if(controller.leftx == 255){
      TURN_RIGHT();
    }

    // >>> START BUTTON <<< // RESERVED
    //controller.start;
  }

}
/*
MOTOR CONTROLS
 */
void MOVE_FORWARD(){
  digitalWrite(MOTOR_1_DIRECTION, HIGH);
  analogWrite(MOTOR_1_SPEED, RUNSPEED);
  analogWrite(MOTOR_2_SPEED, RUNSPEED);
  digitalWrite(MOTOR_2_DIRECTION, HIGH);
}
void MOVE_BACKWARD(){
  digitalWrite(MOTOR_1_DIRECTION, LOW);
  analogWrite(MOTOR_1_SPEED, RUNSPEED);
  analogWrite(MOTOR_2_SPEED, RUNSPEED);
  digitalWrite(MOTOR_2_DIRECTION, LOW);
}
void TURN_RIGHT(){
  digitalWrite(MOTOR_1_DIRECTION, HIGH);
  analogWrite(MOTOR_1_SPEED, RUNSPEED);
  analogWrite(MOTOR_2_SPEED, RUNSPEED);
  digitalWrite(MOTOR_2_DIRECTION, LOW);
}
void TURN_LEFT(){
  digitalWrite(MOTOR_1_DIRECTION, LOW);
  analogWrite(MOTOR_1_SPEED, RUNSPEED);
  analogWrite(MOTOR_2_SPEED, RUNSPEED);
  digitalWrite(MOTOR_2_DIRECTION, HIGH);
}
void STOP(){
  digitalWrite(MOTOR_1_DIRECTION, LOW);
  analogWrite(MOTOR_1_SPEED, FULL_STOP);
  analogWrite(MOTOR_2_SPEED, FULL_STOP);
  digitalWrite(MOTOR_2_DIRECTION, LOW);
}

//AMORE

/*****************************************************************/
/*     Do not modify anything past this line
 Unless you are absolutely sure you know
 what you are doing
 */

/*
  SerialEvent occurs whenever a new data comes in the
 hardware serial RX.  This routine is run between each
 time loop() runs, so using delay inside loop can delay
 response.  Multiple bytes of data may be available.
 */


void serialEvent() {
  while (Serial.available()) {

    // get the new byte:
    byte inChar = (byte)Serial.read(); 

    switch(serial_state){
    case SER_SYNC1:
      // Look for first sync byte 0x5A
      serial_rxcounter=0;
      if(inChar==0x5A)serial_state=SER_SYNC2;
      break;

    case SER_SYNC2:
      // Verify if valid packet by checking second byte = 0xA5
      if(inChar==0xA5) 
        serial_state=SER_FETCHDATA;
      else
        serial_state=SER_SYNC1;
      break;

    case  SER_FETCHDATA:
      // Store data packet
      inputData[serial_rxcounter]= inChar;
      serial_rxcounter++;
      // Process when 10 bytes is completed
      if(serial_rxcounter>=10){
        //checksum should match

        if(checksum()==inputData[9]){
          refresh_controller();
          serial_available=true;    // data packet is ready
        }
        else{
          // bad checksum, disregard packet and wait for a new packet
          serial_state=SER_SYNC1;
          serial_available=false;
        }
      }

    }

  }
}

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

// calculate checksum

byte  checksum(void){
  int  i;
  byte chk= 0x5A+0xA5;
  for(i=0;i<9;i++) chk += inputData[i];
  return(chk);

}






















