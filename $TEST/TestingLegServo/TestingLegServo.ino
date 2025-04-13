#include <ESP32Servo.h>
#include <AccelStepper.h>

#define dirPin 1
#define stepPin 2
#define motorInterfaceType 1  // 1: DRV8825 Driver

#define pin_Swing 41
#define pin_Raise 20
#define pin_Knee 21
#define pin_InnerAnkle 36
#define pin_OuterAnkle 38

//const int homePosition[5] = {90, 92, 90, 87, 94};   // Brian's Homing Position
//const int homePosition[5] = {90, 90, 95, 85, 95};   // Venn's Homing Position
//const int homePosition[5] = {90, 124, 100, 97, 84};   // Bent Position #1
const int homePosition[5] = {90, 44, 150, 77, 104};   // Bent Position #2

AccelStepper stepper(motorInterfaceType, stepPin, dirPin);
long targetPosition = 0; // Default position

Servo servo_1;
Servo servo_2;
Servo servo_3;
Servo servo_4;
Servo servo_5;
Servo* servos[5] = {&servo_1, &servo_2, &servo_3, &servo_4, &servo_5};

void setup() 
{
  servo_1.attach(pin_Swing);
  servo_2.attach(pin_Raise);
  servo_3.attach(pin_Knee);
  servo_4.attach(pin_InnerAnkle);
  servo_5.attach(pin_OuterAnkle);
  
  for (int i = 0; i < 5; i++) 
  {
    servos[i]->write(homePosition[i]);
    delay(1000);
  }

  Serial.begin(115200); // Start serial monitor
  stepper.setMaxSpeed(200);       // steps per second
  stepper.setAcceleration(100);   // steps per second squared
  stepper.setCurrentPosition(0);  // Start at zero

  Serial.println("Enter a target position (in steps):");
}

void loop() {
  stepper.run();

  // Check for serial input
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');  // Read input until Enter is pressed
    input.trim();  // Remove whitespace
    if (input.length() > 0) {
      long newTarget = input.toInt(); // Convert input to integer
      targetPosition = newTarget;
      stepper.moveTo(targetPosition);
      Serial.print("New target set to: ");
      Serial.println(targetPosition);
    }
  }

}
