// #include "AccelStepper.h"

// #define dirPin 1
// #define stepPin 2
// #define motorInterfaceType 3
// #define indicator_LED 7

// AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);

// void setup() 
// {
//   stepper.setMaxSpeed(1000); 
//   pinMode(indicator_LED, OUTPUT);
// }

// void loop() 
// {
//   stepper.setCurrentPosition(0);
//   digitalWrite(indicator_LED, HIGH);
//   delay(3000);
//   digitalWrite(indicator_LED, LOW);

//   while(stepper.currentPosition() != 400)
//   {
//     stepper.setSpeed(200);
//     stepper.runSpeed();
//   }

//   delay(1000);
// }

#include "AccelStepper.h"

#define dirPin 1
#define stepPin 2
#define motorInterfaceType 3

AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);

void setup() 
{
  stepper.setMaxSpeed(1600);
  stepper.setAcceleration(960);
  stepper.setCurrentPosition(0);
}

void loop() 
{
  stepper.moveTo(6400);
  stepper.runToPosition();

  delay(1000);

  stepper.moveTo(0);
  stepper.runToPosition();

  delay(1000);

}

