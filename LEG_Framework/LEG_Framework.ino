// LEG Framework - Brian Ordonez

#include <ESP32Servo.h>
#include <ESP32Servo.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

TaskHandle_t TASK1;    // SPI/UART Communication/ISR :    Core 0
TaskHandle_t TASK2;    // Variable Read/Write and Math:   Core 0
TaskHandle_t TASK3;    // Servo & Stepper Motor Cotnrol:  Core 1
TaskHandle_t TASK4;    // IMU, ASS600. VS56LOX:           Core 1

// Swing Servo Motor
Servo swingServo;
float SwingPA[3] = {0.0, 0.0, 0.0};
bool SwingPAWR = false;

// Raise Servo Motor
Servo raiseServo;
float RaisePA[3] = {0.0, 0.0, 0.0};
bool RaisePAWR = false;

// Knee Servo Motor
Servo kneeServo;
float KneePA[3] = {0.0, 0.0, 0.0};
bool KneePAWR = false;

// InnerAnkle Servo Motor
Servo innerAnkleServo;
float InnerAnklePA[3] = {0.0, 0.0, 0.0};
bool InnerAnklePAWR = false;

// OuterAnkle Servo Motor
Servo outerAnkleServo;
float OuterAnklePA[3] = {0.0, 0.0, 0.0};
bool OuterAnklePAWR = false;




void setup() 
{
  // Global Variables

  //Setting Up Tasks 1-4 
  xTaskCreatePinnedToCore(
                    Communication,   /* Task function. */
                    "TASK1",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &TASK1,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */ 

  xTaskCreatePinnedToCore(
                    Read_Write,   /* Task function. */
                    "TASK2",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &TASK2,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */

  xTaskCreatePinnedToCore(
                    Motor_Control,   /* Task function. */
                    "TASK3",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &TASK3,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 1 */  

  xTaskCreatePinnedToCore(
                    Sensors,   /* Task function. */
                    "TASK4",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &TASK4,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 1 */               
}
// =============================================================================================
void Communication(void * pvParameters)
{
  Serial.print("LEG Communication Running...Core: ");
  Serial.println(xPortGetCoreID());
  for(;;)
  {
    
  } 
}

void Read_Write(void * pvParameters)
{
  Serial.print("LEG Read/Write Running...Core: ");
  Serial.println(xPortGetCoreID());
  for(;;)
  {

  }
}

void Motor_Control(void * pvParameters)
{
  Serial.print("LEG Motor_Control Running...Core: ");
  Serial.println(xPortGetCoreID());
  for(;;)
  {
    if ((SwingPA[0] != SwingPA[1]) && SwingPAWR == false)    // Shoulder Pitch 
    {  
      SwingPAWR = true;                                      // Flag enable
      swingServo.write(SwingPA[0]);                                  //Write Angle
      delay(1);                                              // Minor Wait
      SwingPA[2] = SwingPA[1];
      SwingPA[1] = SwingPA[0];                              // Log
      SwingPAWR = false;                                     // Flag Disable
    } 
    else 
    {
      SwingPAWR = true;                                      // Flag enable
      swingServo.write(SwingPA[1]);
      delay(1);
      SwingPAWR = false;                                     // Flag disable
    }

    if ((RaisePA[0] != RaisePA[1]) && RaisePAWR == false)    // Shoulder Pitch 
    {  
      RaisePAWR = true;                                      // Flag enable
      raiseServo.write(RaisePA[0]);                                  //Write Angle
      delay(1);                                              // Minor Wait
      RaisePA[2] = RaisePA[1];
      RaisePA[1] = RaisePA[0];                              // Log
      RaisePAWR = false;                                     // Flag Disable
    } 
    else 
    {
      RaisePAWR = true;                                      // Flag enable
      raiseServo.write(RaisePA[1]);
      delay(1);
      RaisePAWR = false;                                     // Flag disable
    }

    if ((KneePA[0] != KneePA[1]) && KneePAWR == false)    // Shoulder Pitch 
    {  
      KneePAWR = true;                                      // Flag enable
      kneeServo.write(KneePA[0]);                                  //Write Angle
      delay(1);                                              // Minor Wait
      KneePA[2] = KneePA[1];
      KneePA[1] = KneePA[0];                              // Log
      KneePAWR = false;                                     // Flag Disable
    } 
    else 
    {
      KneePAWR = true;                                      // Flag enable
      kneeServo.write(KneePA[1]);
      delay(1);
      KneePAWR = false;                                     // Flag disable
    }

    if ((InnerAnklePA[0] != InnerAnklePA[1]) && InnerAnklePAWR == false)    // Shoulder Pitch 
    {  
      InnerAnklePAWR = true;                                      // Flag enable
      innerAnkleServo.write(InnerAnklePA[0]);                                  //Write Angle
      delay(1);                                              // Minor Wait
      InnerAnklePA[2] = InnerAnklePA[1];
      InnerAnklePA[1] = InnerAnklePA[0];                              // Log
      InnerAnklePAWR = false;                                     // Flag Disable
    } 
    else 
    {
      InnerAnklePAWR = true;                                      // Flag enable
      innerAnkleServo.write(InnerAnklePA[1]);
      delay(1);
      InnerAnklePAWR = false;                                     // Flag disable
    }

    if ((OuterAnklePA[0] != OuterAnklePA[1]) && OuterAnklePAWR == false)    // Shoulder Pitch 
    {  
      OuterAnklePAWR = true;                                      // Flag enable
      outerAnkleServo.write(OuterAnklePA[0]);                                  //Write Angle
      delay(1);                                              // Minor Wait
      OuterAnklePA[2] = OuterAnklePA[1];
      OuterAnklePA[1] = OuterAnklePA[0];                              // Log
      OuterAnklePAWR = false;                                     // Flag Disable
    } 
    else 
    {
      OuterAnklePAWR = true;                                      // Flag enable
      outerAnkleServo.write(OuterAnklePA[1]);
      delay(1);
      OuterAnklePAWR = false;                                     // Flag disable
    }

    
  }
}

void Sensors(void * pvParameters)
{
  Serial.print("LEG Sensors...Core: ");
  Serial.println(xPortGetCoreID());
  for(;;)
  {

  }
}

































void loop() {

}