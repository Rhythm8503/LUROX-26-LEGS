// Brian Ordonez 

//   RTOS (Real Time Operating System) is being used to control
//   multiple servo motors at the same time and have them
//   synchronize to start and end at the same time

#include <ESP32Servo.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

TaskHandle_t Task1;
TaskHandle_t Task2;
TaskHandle_t Task3;
TaskHandle_t Task4;

Servo servo_1;
Servo servo_2;
Servo servo_3;
Servo servo_4;
Servo servo_5;


void Task1code(void *pvParameters);
void Task2code(void *pvParameters);
void Task3code(void *pvParameters);
void Task4code(void *pvParameters);


EventGroupHandle_t syncEventGroup;

const int TASK1_STARTED_BIT = (1 << 0);  // Bit 0 for Task1 start
const int TASK2_STARTED_BIT = (1 << 1);  // Bit 1 for Task2 start
const int TASK3_STARTED_BIT = (1 << 2);  // Bit 1 for Task2 start
const int TASK4_STARTED_BIT = (1 << 3);  // Bit 1 for Task2 start
const int TASKS_DONE_BIT    = (1 << 4);  // Bit 2 for tasks completion

const int TASK1_LOOP_DONE_BIT = (1 << 5);  // Bit 3 for Task1 loop done
const int TASK2_LOOP_DONE_BIT = (1 << 6);  // Bit 4 for Task2 loop done
const int TASK3_LOOP_DONE_BIT = (1 << 7);  // Bit 4 for Task2 loop done
const int TASK4_LOOP_DONE_BIT = (1 << 8);  // Bit 4 for Task2 loop done



void setup() 
{
  Serial.begin(115200);
  delay(1000);

  // This is new Setup!

  servo_1.attach(41);
  servo_2.attach(20);
  servo_3.attach(21);
  servo_4.attach(36);
  servo_5.attach(38);

  servo_1.write(90);
  servo_2.write(108);
  servo_3.write(90); 
  servo_4.write(95);
  servo_5.write(90);


  delay(1000); // DO NOT ERASE THIS! THIS IS SUPER MEGA IMPORTANT!

  syncEventGroup = xEventGroupCreate();

  //create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(
                    Task1code,   /* Task function. */
                    "Task1",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task1,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */                  
  // delay(500);    THESE WERE CAUSING SYNC ISSUES WITH THE OTHER SERVOS

  //create a task that will be executed in the Task2code() function, with priority 1 and executed on core 1
    xTaskCreatePinnedToCore(
                   Task2code,   /* Task function. */
                   "Task2",     /* name of task. */
                   10000,       /* Stack size of task */
                   NULL,        /* parameter of the task */
                   1,           /* priority of the task */
                   &Task2,      /* Task handle to keep track of created task */
                   0);          /* pin task to core 1 */
     // delay(500);  THESE WERE CAUSING SYNC ISSUES WITH THE OTHER SERVOS

     xTaskCreatePinnedToCore(
                    Task3code,   /* Task function. */
                    "Task3",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task3,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 0 */                  
     // delay(500);    THESE WERE CAUSING SYNC ISSUES WITH THE OTHER SERVOS

   xTaskCreatePinnedToCore(
                    Task4code,   /* Task function. */
                    "Task4",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task4,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 0 */                  
     // delay(500);    THESE WERE CAUSING SYNC ISSUES WITH THE OTHER SERVOS

  xEventGroupSetBits(syncEventGroup, TASK1_STARTED_BIT | TASK2_STARTED_BIT | TASK3_STARTED_BIT | TASK4_STARTED_BIT);

}

void Task1code( void * pvParameters )
{
  for(;;)
  {
      xEventGroupWaitBits(syncEventGroup, TASK1_STARTED_BIT | TASK2_STARTED_BIT | TASK3_STARTED_BIT | TASK4_STARTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);

      for(int i = 108; i <= 140; i++)
      {
        servo_2.write(i);
        vTaskDelay(pdMS_TO_TICKS(30));
      }

      xEventGroupSetBits(syncEventGroup, TASK1_LOOP_DONE_BIT);
      // Wait for Task 2 to complete its loop before continuing
      xEventGroupWaitBits(syncEventGroup, TASK2_LOOP_DONE_BIT | TASK3_LOOP_DONE_BIT | TASK4_LOOP_DONE_BIT, pdTRUE, pdFALSE, portMAX_DELAY);

      vTaskDelay(pdMS_TO_TICKS(10));

      for(int i = 140; i >= 60; i--)
      {
        servo_2.write(i);
        vTaskDelay(pdMS_TO_TICKS(30));
      }

      xEventGroupSetBits(syncEventGroup, TASK1_LOOP_DONE_BIT);

      xEventGroupWaitBits(syncEventGroup, TASK2_LOOP_DONE_BIT | TASK3_LOOP_DONE_BIT | TASK4_LOOP_DONE_BIT, pdTRUE, pdFALSE, portMAX_DELAY);


      vTaskDelay(pdMS_TO_TICKS(10));

      for(int i = 60; i <= 108; i++)
      {
        servo_2.write(i);
        vTaskDelay(pdMS_TO_TICKS(30));
      }

      xEventGroupSetBits(syncEventGroup, TASKS_DONE_BIT);
  } 
}

//Task2code: blinks an LED every 700 ms
void Task2code( void * pvParameters )
{
  for(;;)
  {
      xEventGroupWaitBits(syncEventGroup, TASK1_STARTED_BIT | TASK2_STARTED_BIT | TASK3_STARTED_BIT | TASK4_STARTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);

      for(int i = 90; i <= 100; i++)
      {
        servo_3.write(i);
        vTaskDelay(pdMS_TO_TICKS(30));
      }

      xEventGroupSetBits(syncEventGroup, TASK2_LOOP_DONE_BIT);

      // Wait for Task 1 to complete its loop before continuing
      xEventGroupWaitBits(syncEventGroup, TASK1_LOOP_DONE_BIT | TASK3_LOOP_DONE_BIT | TASK4_LOOP_DONE_BIT, pdTRUE, pdFALSE, portMAX_DELAY);

      vTaskDelay(pdMS_TO_TICKS(10));

      for(int i = 100; i <= 150; i++)
      {
        servo_3.write(i);
        vTaskDelay(pdMS_TO_TICKS(30));
      }

      xEventGroupSetBits(syncEventGroup, TASK2_LOOP_DONE_BIT);
      xEventGroupWaitBits(syncEventGroup, TASK1_LOOP_DONE_BIT | TASK3_LOOP_DONE_BIT | TASK4_LOOP_DONE_BIT, pdTRUE, pdFALSE, portMAX_DELAY);

      vTaskDelay(pdMS_TO_TICKS(10));

      for(int i = 150; i >= 90; i--)
      {
        servo_3.write(i);
        vTaskDelay(pdMS_TO_TICKS(30));
      }

      xEventGroupSetBits(syncEventGroup, TASKS_DONE_BIT);
  }
}

void Task3code( void * pvParameters )
{
  xEventGroupWaitBits(syncEventGroup, TASK1_STARTED_BIT | TASK2_STARTED_BIT | TASK3_STARTED_BIT | TASK4_STARTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);

  for(;;)
  {
    for(int i = 95; i <= 110; i++)
    {
      servo_4.write(i);
      vTaskDelay(pdMS_TO_TICKS(30));
    }

    xEventGroupSetBits(syncEventGroup, TASK3_LOOP_DONE_BIT);
    xEventGroupWaitBits(syncEventGroup, TASK1_LOOP_DONE_BIT | TASK2_LOOP_DONE_BIT | TASK4_LOOP_DONE_BIT, pdTRUE, pdFALSE, portMAX_DELAY);

    vTaskDelay(pdMS_TO_TICKS(10));

    for(int i = 110; i >= 80; i--)
    {
      servo_4.write(i);
      vTaskDelay(pdMS_TO_TICKS(30));
    }

    xEventGroupSetBits(syncEventGroup, TASK3_LOOP_DONE_BIT);
    xEventGroupWaitBits(syncEventGroup, TASK1_LOOP_DONE_BIT | TASK2_LOOP_DONE_BIT | TASK4_LOOP_DONE_BIT, pdTRUE, pdFALSE, portMAX_DELAY);

    vTaskDelay(pdMS_TO_TICKS(10));

    for(int i = 80; i <= 95; i++)
    {
      servo_4.write(i);
      vTaskDelay(pdMS_TO_TICKS(30));
    }

    xEventGroupSetBits(syncEventGroup, TASKS_DONE_BIT);
  }
}

void Task4code( void * pvParameters )
{
  xEventGroupWaitBits(syncEventGroup, TASK1_STARTED_BIT | TASK2_STARTED_BIT | TASK3_STARTED_BIT | TASK4_STARTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);

  for(;;)
  {
    for(int i = 90; i >= 75; i--)
    {
      servo_5.write(i);
      vTaskDelay(pdMS_TO_TICKS(30));
    }

    xEventGroupSetBits(syncEventGroup, TASK4_LOOP_DONE_BIT);
    xEventGroupWaitBits(syncEventGroup, TASK1_LOOP_DONE_BIT | TASK2_LOOP_DONE_BIT | TASK3_LOOP_DONE_BIT, pdTRUE, pdFALSE, portMAX_DELAY);

    vTaskDelay(pdMS_TO_TICKS(10));

    for(int i = 75; i <= 105; i++)
    {
      servo_5.write(i);
      vTaskDelay(pdMS_TO_TICKS(30));
    }

    xEventGroupSetBits(syncEventGroup, TASK4_LOOP_DONE_BIT);
    xEventGroupWaitBits(syncEventGroup, TASK1_LOOP_DONE_BIT | TASK2_LOOP_DONE_BIT | TASK3_LOOP_DONE_BIT, pdTRUE, pdFALSE, portMAX_DELAY);

    vTaskDelay(pdMS_TO_TICKS(10));

    for(int i = 105; i >= 90; i--)
    {
      servo_5.write(i);
      vTaskDelay(pdMS_TO_TICKS(30));
    }

    xEventGroupSetBits(syncEventGroup, TASKS_DONE_BIT);
  } 
}

void loop() 
{

}