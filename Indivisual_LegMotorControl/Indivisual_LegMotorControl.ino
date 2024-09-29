#include <ESP32Servo.h>

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;
int pos = 90;
int j = 90;

void setup() 
{
  servo1.attach(4);
  servo2.attach(5);
  servo3.attach(6);
  servo4.attach(7);
  servo5.attach(15);
  
  delay(1000);

  servo1.write(90);
  servo2.write(108);  // 108 (Straight)
  servo3.write(90);  // 90 (Straight)
  servo4.write(95);
  servo5.write(pos);
  delay(1000);

}

void loop() 
{
  // servo1.write(80);
  // servo2.write(108);  // 108 (Straight)
  // servo3.write(90);  // 90 (Straight)
  // servo4.write(95);
  // servo5.write(pos);

  // delay(500);

  // SERVO#3

  /*for(int i = 30; i <= 150; i++)
  {
    servo3.write(i);
    delay(30);
  }

  delay(1000);

  for(int i = 150; i >= 30; i--)
  {
    servo3.write(i);
    delay(30);
  }

  delay(1000); */

 // SERVO#2
 
  /*for(int i = 30; i <= 168; i++)
  {
    servo2.write(i);
    delay(30);
  }

  delay(1000);

  for(int i = 168; i >= 30; i--)
  {
    servo2.write(i);
    delay(30);
  }

  delay(500); */

  // SERVO#1
 
  for(int i = 90; i <= 150; i++)
  {
    servo3.write(i);
    delay(30);
  }

  delay(1000);

  for(int i = 150; i >= 90; i--)
  {
    servo3.write(i);
    delay(30);
  }

  delay(1000);

}
