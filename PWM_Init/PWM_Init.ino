#include <ESP32Servo.h>
#include "AccelStepper.h"

#define PIN_UpperLeg 4
#define PIN_LowerUpperLeg 5
#define PIN_Knee 6
#define PIN_LeftAnkle 7
#define PIN_RightAnkle 15

#define PWM_FREQ 50
#define PWM_RES 16

#define PIN_DIR_LEG 1
#define PIN_STEP_LEG 2
#define motorInterfaceType 3

void setup() 
{
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);

  ESP32PWM Servo_1;
  ESP32PWM Servo_2;
  ESP32PWM Servo_3;
  ESP32PWM Servo_4;
  ESP32PWM Servo_5;

  AccelStepper Stepper_1 = AccelStepper(motorInterfaceType, PIN_STEP_LEG, PIN_DIR_LEG);

  // Stepper_1.setMaxSpeed(1600);
  // Stepper_1.setAcceleration(960);  THESE 3 LINES OF CODE NEED TO BE TESTED FIRST BEFORE UNCOMMENTING
  // Stepper_1.setCurrentPosition(0);

  Servo_1.attachPin(PIN_UpperLeg, PWM_FREQ, PWM_RES);
  Servo_2.attachPin(PIN_LowerUpperLeg, PWM_FREQ, PWM_RES);
  Servo_3.attachPin(PIN_Knee, PWM_FREQ, PWM_RES);
  Servo_4.attachPin(PIN_LeftAnkle, PWM_FREQ, PWM_RES);
  Servo_5.attachPin(PIN_RightAnkle, PWM_FREQ, PWM_RES);

  writeDegree(Servo_1, 90);
  writeDegree(Servo_2, 108);
  writeDegree(Servo_3, 90);
  writeDegree(Servo_4, 95);
  writeDegree(Servo_5, 90);

}
void writeDegree(ESP32PWM &pwm, int degree) 
{
  uint16_t pwmValue = map(degree, 0, 180, 0, 65535);
  pwm.write(pwmValue); 
}

void loop()
{
 
}
