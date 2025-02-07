
#include <Arduino.h>
#include <Stepper.h>
//Motors
//Currently 400 steps per rev 
const int stepPinM1 = 22; //PUL+ Green Current setp
const int dirPinM1 = 23; //DIR+ Blue
const int enPinM1 = 24; //ENA+ Red
long rotaryPosition;
long previousPosition;
int m1Step = 1;
unsigned long previousM1Micros = 0;

unsigned long previousSlowStepM1 = 0;

long m1Speed = 300; // 1000 is 70/min.... 750 = 90/min... If change, to change speed change the m1 speed in else of runMotorM1()...
long m2Speed = 910; //150 is 70/min.... 95 = 90/min @800 steps/rev --- 910 @200 steps/rev
long m3Speed = 1400; //200 is 70/min.... 130 = 90/min @800 steps/rev --- 1250 @200 steps/rev
double m1PulsePerRevMultiplier = 0.9; //.9 for 400, .45 for 800 on driver

long calculateDegrees(long rotaryPosition) //converts the steps the stepper has stepped to degrees //a 400 step goes 0.9 degrees per step. 200 stepper motor is 1.8 degrees per step. Currently 800!
{
  long result = rotaryPosition * m1PulsePerRevMultiplier; 
  return result;
}


// void runMotorM2()
// {
//     unsigned long currentMicros = micros();
//   digitalWrite(dirPinM2, HIGH);
//   for (int x = 0; x < 1; x++)
//   {
// if((currentMicros - previousM2Micros)> m2Speed)
//   {
//     if(m2Step ==1){
//     digitalWrite(stepPinM2, HIGH);
//       ++m2Step;
//     }
//     else if(m2Step ==2){
//       digitalWrite(stepPinM2, LOW);
//       m2Step = 1;
//     }
//   previousM2Micros = currentMicros; 
  
//   }
//   }
// }
void runMotorM1()
{
  digitalWrite(dirPinM1, LOW);
  unsigned long currentMicros = micros();
  for (int x = 0; x < 1; x++)
  {
    if((currentMicros - previousM1Micros)> m1Speed){
      if(m1Step ==1){
        digitalWrite(stepPinM1, HIGH);
        ++m1Step;
        previousPosition = rotaryPosition;
        rotaryPosition = rotaryPosition + 1;
      }
      else if(m1Step ==2){
          digitalWrite(stepPinM1, LOW);
          m1Step = 1;
      }
      previousM1Micros = currentMicros; 
    }
  }
}
// void stepM1()
// {
//   digitalWrite(dirPinM1, HIGH);
//   digitalWrite(stepPinM1, HIGH);
//   delayMicroseconds(9000);
//   digitalWrite(stepPinM1, LOW); 
// }

void setup()
{
 
  //Motors
  pinMode(stepPinM1, OUTPUT);
  pinMode(dirPinM1, OUTPUT);
  pinMode(enPinM1, OUTPUT);
  
}

void loop()
{
  for (int i = 0; i < i+10; i++) {
        digitalWrite(stepPinM1, HIGH);
        delayMicroseconds(500);
        digitalWrite(stepPinM1, LOW);
        delayMicroseconds(500);
    }
}
