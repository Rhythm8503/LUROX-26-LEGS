//  Brian Ordonez {Computer Engineer}

//  Text based User Interface to control the indivisual
//  servo motors by having the user pick the motor and 
//  send the value to the motor.

#include <ESP32Servo.h>

Servo servo_1;
Servo servo_2;
Servo servo_3;
Servo servo_4;
Servo servo_5;

void setup() 
{
  Serial.begin(115200);
  delay(1000);
  Serial.println("");
  Serial.flush();
}

void loop() 
{
  int servoChoice = 0;

  while(servoChoice <= 0 || servoChoice >= 7)
  {
    displayMenu();
    while(Serial.available() == 0){}
    servoChoice = Serial.parseInt();
    Serial.println(servoChoice);
  }

  attachMotor(servoChoice);
  direction(servoChoice);
}

void displayMenu() 
{
  Serial.println("");
  Serial.println("===== MENU =====");
  Serial.println("1. Leg Swing Left to Right");
  Serial.println("2. Upper Leg Bend");
  Serial.println("3. Knee Bend");
  Serial.println("4. Ankle_1");
  Serial.println("5. Ankle_2");
  Serial.println("6. Tasks");
  Serial.print("Enter your choice: ");
}

void attachMotor(int servoChoice)
{
  switch(servoChoice)
  {
    case 1: servo_1.attach(4); 
            Serial.println("Servo_1: Attached");
            servo_1.write(90);
            break;

    case 2: servo_2.attach(5);
            Serial.println("Servo_2: Attached");
            servo_2.write(108);
            break;

    case 3: servo_3.attach(21);
            Serial.println("Servo_3: Attached");
            servo_3.write(90);
            break;

    case 4: servo_4.attach(7);
            Serial.println("Servo_4: Attached");
            servo_4.write(95);
            break;

    case 5: servo_5.attach(15);
            Serial.println("Servo_5: Attached");
            servo_5.write(90);
            break;

    case 6: servo_1.attach(4);
            servo_2.attach(5);
            servo_3.attach(6);
            servo_4.attach(7);
            servo_5.attach(15);

            Serial.println("Servo_1: Attached");
            Serial.println("Servo_2: Attached");
            Serial.println("Servo_3: Attached");
            Serial.println("Servo_4: Attached");
            Serial.println("Servo_5: Attached");

            servo_5.write(90);
            servo_5.write(108);
            servo_5.write(90);
            servo_5.write(95);
            servo_5.write(90);
            break;

    default: Serial.println("Invalid Input...Please Try Again!");

  }
}

void direction(int servoChoice)
{
  int dir_choice = 0;

  if(servoChoice == 1)
  {
    while(dir_choice <= 0 || dir_choice >= 3)
    {
      Serial.println("");
      Serial.println("1. Left");
      Serial.println("2. Right");
      Serial.print("Enter Direction: ");
      while(Serial.available() == 0){}
      dir_choice = Serial.parseInt();
      Serial.println(dir_choice);
    }

    switch(dir_choice)
    {
      case 1: degreeLeftChoice(); break;

      case 2: degreeRightChoice(); break;

      default: break;
    }
  }

  else if(servoChoice == 2 || servoChoice == 3)
  {
    while(dir_choice <= 0 || dir_choice >= 3)
    {
      Serial.println("");
      Serial.println("1. Backwards");
      Serial.println("2. Forward");
      Serial.print("Enter Direction: ");
      while(Serial.available() == 0){}
      dir_choice = Serial.parseInt();
      Serial.println(dir_choice);
    }

    switch(dir_choice)
    {
      case 1: degreeBackwardsChoice(servoChoice); break;

      case 2: degreeForwardsChoice(servoChoice); break;

      default: break;
    }
  }

  else if(servoChoice == 4 || servoChoice == 5)
  {
    while(dir_choice <= 0 || dir_choice >= 3)
    {
      Serial.println("");
      Serial.println("1. Up");
      Serial.println("2. Down");
      Serial.print("Enter Direction: ");
      while(Serial.available() == 0){}
      dir_choice = Serial.parseInt();
      Serial.println(dir_choice);
    }

    if(servoChoice == 4)
    {
      switch(dir_choice)
      {
        case 1: degreeUpChoice(servoChoice); break;

        case 2: degreeDownChoice(servoChoice); break;

        default: break;
      }
    }
    else if(servoChoice == 5)
    {
      switch(dir_choice)
      {
        case 1: degreeDownChoice(servoChoice); break;

        case 2: degreeUpChoice(servoChoice); break;

        default: break;
      }
    }
  }

  // else if(servoChoice == 6)
  // {

  // }
}

void degreeLeftChoice()
{
  bool hold = holdOrReturn();
  int degree;

 while(degree < 90 || degree > 150)
 {
    Serial.println("");
    Serial.print("Enter a degree (90 - 150): ");
    while(Serial.available() == 0){}
    degree = Serial.parseInt();
    Serial.println(degree);
 }

  for(int i = 90; i <= degree; i++)
  {
    servo_1.write(i);
    delay(30);
  }

  delay(1000);

  if(hold == false)
  {
    for(int i = degree; i >= 90; i--)
    {
      servo_1.write(i);
      delay(30);
    }

    delay(1000);
  }
}

void degreeRightChoice()
{
  bool hold = holdOrReturn();
  int degree;

  while(degree < 40 || degree > 90)
  {
    Serial.println("");
    Serial.print("Enter a degree (40 - 90): ");
    while(Serial.available() == 0){}
    degree = Serial.parseInt();
    Serial.println(degree);
  }

  for(int i = 90; i >= degree; i--)
  {
    servo_1.write(i);
    delay(30);
  }

  delay(1000);

  if(hold == false)
  {
    for(int i = degree; i <= 90; i++)
    {
      servo_1.write(i);
      delay(30);
    }

    delay(1000);
  }
}

void degreeBackwardsChoice(int servoChoice)
{
  bool hold = holdOrReturn();
  int degree;

  if(servoChoice == 2)
  {
    while(degree < 108 || degree > 150)
    {
      Serial.println("");
      Serial.print("Enter a degree (108 - 150): ");
      while(Serial.available() == 0){}
      degree = Serial.parseInt();
      Serial.println(degree);
    }
  }

  else if(servoChoice == 3)
  {
    while(degree < 90 || degree > 150)
    {
      Serial.println("");
      Serial.print("Enter a degree (90 - 150): ");
      while(Serial.available() == 0){}
      degree = Serial.parseInt();
      Serial.println(degree);
    }
  }

  int ini_degree = 108;

  if(servoChoice == 3)
  {
    ini_degree = 90;
  }

  for(int i = ini_degree; i <= degree; i++)
  {
    if(servoChoice == 2)
    {
      servo_2.write(i);
    }
    else
    {
      servo_3.write(i);
    }
    
    delay(30);
  }

  delay(1000);

  if(hold == false)
  {
    for(int i = degree; i >= ini_degree; i--)
    {
      if(servoChoice == 2)
      {
        servo_2.write(i);
      }
      else
      {
        servo_3.write(i);
      }

      delay(30);
    }

    delay(1000);
  } 
}

void degreeForwardsChoice(int servoChoice)
{
  bool hold = holdOrReturn();
  int degree;

  if(servoChoice == 2)
  {
    while(degree < 40 || degree > 108)
    {
      Serial.println("");
      Serial.print("Enter a degree (40 - 108): ");
      while(Serial.available() == 0){}
      degree = Serial.parseInt();
      Serial.println(degree);
    }
  }

  else if(servoChoice == 3)
  {
    while(degree < 40 || degree > 90)
    {
      Serial.println("");
      Serial.print("Enter a degree (40 - 90): ");
      while(Serial.available() == 0){}
      degree = Serial.parseInt();
      Serial.println(degree);
    }
  }

  int ini_degree = 108;

  if(servoChoice == 3)
  {
    ini_degree = 90;
  }
  

  for(int i = ini_degree; i >= degree; i--)
  {
    if(servoChoice == 2)
    {
      servo_2.write(i);
    }
    else
    {
      servo_3.write(i);
    }

    delay(30);
  }

  delay(1000);

  if(hold == false)
  {
    for(int i = degree; i <= ini_degree; i++)
    {
      if(servoChoice == 2)
      {
        servo_2.write(i);
      }
      else
      {
        servo_3.write(i);
      }

      delay(30);
    }

    delay(1000);
  }
}

void degreeUpChoice(int servoChoice)
{
  bool hold = holdOrReturn();
  int degree;

 if(servoChoice == 4)
 {
    while(degree < 95 || degree > 110)
    {
      Serial.println("");
      Serial.print("Enter a degree (95 - 110): ");
      while(Serial.available() == 0){}
      degree = Serial.parseInt();
      Serial.println(degree);
    }
  }

 else if(servoChoice == 5)
 {
    while(degree < 90 || degree > 105)
    {
      Serial.println("");
      Serial.print("Enter a degree (90 - 105): ");
      while(Serial.available() == 0){}
      degree = Serial.parseInt();
      Serial.println(degree);
    }
  }

 int ini_degree = 95;

  if(servoChoice == 5)
  {
    ini_degree = 90;
  }

  for(int i = ini_degree; i <= degree; i++)
  {
    if(servoChoice == 4)
    {
      servo_4.write(i);
    }
    else if(servoChoice == 5)
    {
      servo_5.write(i);
    }
    
    delay(30);

  }

  delay(1000);

  if(hold == false)
  {
    for(int i = degree; i >= ini_degree; i--)
    {
      if(servoChoice == 4)
      {
        servo_4.write(i);
      }
      else if(servoChoice == 5)
      {
        servo_5.write(i);
      }
    
      delay(30);
    }

    delay(1000);
  }
}

void degreeDownChoice(int servoChoice)
{
  bool hold = holdOrReturn();
  int degree;

 if(servoChoice == 4)
 {
    while(degree < 80 || degree > 95)
    {
      Serial.println("");
      Serial.print("Enter a degree (80 - 95): ");
      while(Serial.available() == 0){}
      degree = Serial.parseInt();
      Serial.println(degree);
    }
  }

 else if(servoChoice == 5)
 {
    while(degree < 75 || degree > 90)
    {
      Serial.println("");
      Serial.print("Enter a degree (75 - 90): ");
      while(Serial.available() == 0){}
      degree = Serial.parseInt();
      Serial.println(degree);
    }
  }

  int ini_degree = 95;

  if(servoChoice == 5)
  {
    ini_degree = 90;
  }

  for(int i = ini_degree; i >= degree; i--)
  {
    if(servoChoice == 4)
    {
      servo_4.write(i);
    }
    else if(servoChoice == 5)
    {
      servo_5.write(i);
    }
    
    delay(30);
  }

  delay(1000);

  if(hold == false)
  {
    for(int i = ini_degree; i <= 90; i++)
    {
      if(servoChoice == 4)
      {
        servo_4.write(i);
      }
      else if(servoChoice == 5)
      {
        servo_5.write(i);
      }
      
      delay(30);
    }

    delay(1000);
  }
}

bool holdOrReturn()
{
  int c = 0;

  while(c <= 0 || c >= 3)
  {
    Serial.println("");
    Serial.println("1. Hold");
    Serial.println("2. Return");
    Serial.print("Enter Selection: ");
    while(Serial.available() == 0){}
    c = Serial.parseInt();
    Serial.println(c);
  }

    switch(c)
    {
      case 1: return true; break;

      case 2: return false; break;

      default: break;
    }
}

