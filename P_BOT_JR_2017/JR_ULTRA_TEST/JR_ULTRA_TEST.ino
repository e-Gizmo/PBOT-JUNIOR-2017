//*********************************************//
//                  P_BOT_JR                   //
//                 ULTRA TEST                  //
//         THE MOTOR SHOULD GO FORWARD         //
//       WHEN THE ULTRASONIC DISTANCEE IS      //
//               15 CM AND BELOW               //
//                 Codes by:                   //
//        e-Gizmo Mechatronix Central          //
//         Taft, Manila, Philippines           //
//           http://www.egizmo.com             //
//                NOV. 24,2017                 //
//*********************************************//

// INITIALIZATION:
#include <NewPing.h>
#define TRIGGER_PIN 18                    // US TRIGGER_PIN pin
#define ECHO_PIN 19                        // US ECHO_PIN pin 

#define MOTOR_1_DIRECTION         8       // Motor control HIGH = FWD, LOW = BWD
#define MOTOR_1_SPEED             9       // Speed input
#define MOTOR_2_SPEED            10       // Speed input
#define MOTOR_2_DIRECTION        11       // Motor control HIGH = FWD, LOW = BWD

int MAX_DISTANCE = 100;
int FULL_SPEED = 0;
int PAUSE = 50;
NewPing sonar(TRIGGER_PIN,ECHO_PIN,MAX_DISTANCE); // NewPing setup of pins and maximum DISTANCE..
float DISTANCE;

// INPUTS AND OUTPUTS
void setup() 
{
  Serial.begin(9600);  
  // DEFAULT PINS OF THE MOTOR DRIVER (8,9,10,11)
  pinMode(MOTOR_1_DIRECTION, OUTPUT);       
  pinMode(MOTOR_1_SPEED, OUTPUT);
  pinMode(MOTOR_2_SPEED, OUTPUT);
  pinMode(MOTOR_2_DIRECTION, OUTPUT);
} 
// MAIN PROGRAM
void loop() {
  ULTRASONIC_READING();
  if(DISTANCE < 15)
  {
    FORWARD();
    Serial.println("FORWARD");
    delay(PAUSE);
  }
  else{
    STOP();
    Serial.println("STOP");
    delay(PAUSE);
  }
}
void ULTRASONIC_READING()
{
  delay(10);
  unsigned int uS = sonar.ping();         // Send ping, get ping time in microseconds (uS).
  DISTANCE = sonar.convert_cm(uS); 
  Serial.print("Ping: ");
  Serial.print(uS / US_ROUNDTRIP_CM);     // Convert ping time to DISTANCE in cm and print result (0 = outside set DISTANCE range)
  Serial.println("cm");
}
void FORWARD()
{    
  digitalWrite(MOTOR_1_DIRECTION,HIGH);
  digitalWrite(MOTOR_2_DIRECTION,HIGH);
  analogWrite(MOTOR_1_SPEED,FULL_SPEED);
  analogWrite(MOTOR_2_SPEED,FULL_SPEED);   
}
void STOP()
{
  analogWrite(MOTOR_1_SPEED,255);
  analogWrite(MOTOR_2_SPEED,255);
}
 // PROGRAM ENDS AND LOOP BACK FOREVER  
//**********************************************************************************//PARZ

