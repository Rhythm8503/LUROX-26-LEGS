// LEG Framework - Brian Ordonez
// This message is coming from the NVIDIA Jetson!

#include <ESP32Servo.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include <ESP32SPISlave.h>
#include "semphr.h"

TaskHandle_t TASK1;    // SPI/UART Communication/ISR :    Core 0
TaskHandle_t TASK2;    // Variable Read/Write and Math:   Core 0
TaskHandle_t TASK3;    // Process MotorAngles:            Core 0
TaskHandle_t TASK4;    // SwingMotorControl:              Core 0
TaskHandle_t TASK5;    // RaiseMotorControl:              Core 1
TaskHandle_t TASK6;    // KneeMotorControl:               Core 1
TaskHandle_t TASK7;    // InnerAnkleMotorControl:         Core 1
TaskHandle_t TASK8;    // OuterAnkleMotorControl:         Core 1
TaskHandle_t TASK9;    // IMU, ASS600. VS56LOX:           Core 1

SemaphoreHandle_t sema;
QueueHandle_t queue;           // A Queue designed to contain data from Jetson and transfer it to other tasks 
                               // This prevents race conditions and is thread safe 

ESP32SPISlave slave;
static constexpr uint32_t BUFFER_SIZE {8};
uint8_t spi_slave_tx_buf[BUFFER_SIZE];
uint8_t spi_slave_rx_buf[BUFFER_SIZE];

uint8_t controllerID;               // Byte 0: Controller Address
typedef struct
{
  uint8_t M1;
  uint8_t M2;
  uint8_t M3;
  uint8_t M4;
  uint8_t M5;

}MotorAngleReading;

// Swing Servo Motor
Servo swingServo;
volatile float SwingPA[3] = {0.0, 0.0, 0.0};
bool SwingPAWR = false;

// Raise Servo Motor
Servo raiseServo;
volatile float RaisePA[3] = {0.0, 0.0, 0.0};
bool RaisePAWR = false;

// Knee Servo Motor
Servo kneeServo;
volatile float KneePA[3] = {0.0, 0.0, 0.0};
bool KneePAWR = false;

// InnerAnkle Servo Motor
Servo innerAnkleServo;
volatile float InnerAnklePA[3] = {0.0, 0.0, 0.0};
bool InnerAnklePAWR = false;

// OuterAnkle Servo Motor
Servo outerAnkleServo;
volatile float OuterAnklePA[3] = {0.0, 0.0, 0.0};
bool OuterAnklePAWR = false;


void setup() 
{
  Serial.begin(115200);

  // SPI Slave Initialization
  slave.setDataMode(SPI_MODE0);
  slave.begin();

  // Clear Buffers - SPI
  memset(spi_slave_tx_buf, 0, BUFFER_SIZE);
  memset(spi_slave_rx_buf, 0, BUFFER_SIZE);

  sema = xSemaphoreCreateMutex();

  // Create Queue for Data given from NVIDIA Jetsons
  queue = xQueueCreate(5, sizeof(MotorAngleReading));

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
                    Motor_Processing,   /* Task function. */
                    "TASK3",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &TASK3,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */

  xTaskCreatePinnedToCore(
                    SwingMotor_Control,   /* Task function. */
                    "TASK4",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &TASK4,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */

  xTaskCreatePinnedToCore(
                    RaiseMotor_Control,   /* Task function. */
                    "TASK5",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &TASK5,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 1 */

  xTaskCreatePinnedToCore(
                    KneeMotor_Control,   /* Task function. */
                    "TASK6",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &TASK6,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 1 */

  xTaskCreatePinnedToCore(
                    InnerAnkleMotor_Control,   /* Task function. */
                    "TASK7",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &TASK7,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 1 */

  xTaskCreatePinnedToCore(
                    OuterAnkleMotor_Control,   /* Task function. */
                    "TASK8",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &TASK8,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 1 */  

  xTaskCreatePinnedToCore(
                    Sensors,   /* Task function. */
                    "TASK9",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &TASK9,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 1 */        
       
}

void Communication(void * pvParameters)
{
  Serial.print("LEG Communication Running...Core: ");
  Serial.println(xPortGetCoreID());

  for (;;)
  {
    Serial.println("[Core 0]: Waiting for Handshake...");

    // Wait for handshake
    while (slave.transfer(spi_slave_tx_buf, spi_slave_rx_buf, BUFFER_SIZE) == 0);

    if (spi_slave_rx_buf[0] == 0xA1) // Check if addressed to this controller
    {
      Serial.println("Right Leg Controller Selected!");

      Serial.println("Receiving 8-byte command...");
      while (slave.transfer(spi_slave_tx_buf, spi_slave_rx_buf, BUFFER_SIZE) == 0);

      // Extract first 3 values
      controllerID = spi_slave_rx_buf[0];

      // Print raw received data
      Serial.print("Printing Received Data: ");
      for (int i = 0; i < 8; i++) 
      {
        Serial.print(spi_slave_rx_buf[i], HEX);
        Serial.print(" ");
      }
      Serial.println();

      // Print decoded info
      Serial.printf("Controller Address: 0x%02X\n", controllerID);

      // Notify task that data is ready
      xTaskNotifyGive(TASK2);
      vTaskDelay(pdMS_TO_TICKS(100));
    }
    else
    {
      Serial.println("[RIGHT LEG]: No message for me!");
    }
  }
}

void Read_Write(void * pvParameters)  // TASK2
{
  Serial.print("LEG Read/Write Running...Core: ");
  Serial.println(xPortGetCoreID());

  //ServoCommand cmd;
  MotorAngleReading mar;

  for(;;)
  {
    if(xSemaphoreTake(sema, portMAX_DELAY))
    {
      //cmd.servo_id = spi_slave_rx_buf[1];
      //cmd.angle = spi_slave_rx_buf[2];

      mar.M1 = spi_slave_rx_buf[1];
      mar.M2 = spi_slave_rx_buf[2];
      mar.M3 = spi_slave_rx_buf[3];
      mar.M4 = spi_slave_rx_buf[4];
      mar.M5 = spi_slave_rx_buf[5];

      xQueueSend(queue, &mar, portMAX_DELAY);
      xSemaphoreGive(sema);
    }

    xTaskNotifyGive(TASK3);

    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void Motor_Processing(void * pvParameters)  // TASK3
{
  Serial.print("LEG Motor_Processing Running...Core: ");
  Serial.println(xPortGetCoreID());

  MotorAngleReading mar;

  for(;;)
  {
    if(xQueueReceive(queue, &mar, portMAX_DELAY))
    {
      // Store the angles in global arrays (or use the local motorAngle structure)
      SwingPA[0] = (int)((mar.M1 / 255.0f) * 180);
      RaisePA[0] = (int)((mar.M2 / 255.0f) * 180);
      KneePA[0] = (int)((mar.M3 / 255.0f) * 180);
      InnerAnklePA[0] = (int)((mar.M4 / 255.0f) * 180);
      OuterAnklePA[0] = (int)((mar.M5 / 255.0f) * 180);

      // Notify all motor tasks to update simultaneously
      xTaskNotifyGive(Task4);
      xTaskNotifyGive(Task5);
      xTaskNotifyGive(Task6);
      xTaskNotifyGive(Task7);
      xTaskNotifyGive(Task8);
    }

    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void SwingMotor_Control(void * pvParameters)  // TASK4
{
  for(;;)
  {
    // Wait for notification from Task 2
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // Perform angle update
    if(SwingPA[0] != SwingPA[1] && !SwingPAWR)
    {
      SwingPAWR = true;
      swingServo.write(SwingPA[0]);  // Write to Swing motor
      vTaskDelay(pdMS_TO_TICKS(10)); // Minor delay to allow motor to move
      SwingPA[2] = SwingPA[1];
      SwingPA[1] = SwingPA[0];  // Log the new position
      SwingPAWR = false;  // Reset flag
    }

    // Optional: If no movement needed, update motor to last position
    else
    {
      SwingPAWR = true;
      swingServo.write(SwingPA[1]);
      vTaskDelay(pdMS_TO_TICKS(10));  // Minor wait
      SwingPAWR = false;
    }
  }
}

void RaiseMotor_Control(void * pvParameters)  // TASK5
{
  for(;;)
  {
    // Wait for notification from Task 2
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // Perform angle update
    if(RaisePA[0] != RaisePA[1] && !RaisePAWR)
    {
      RaisePAWR = true;
      raiseServo.write(RaisePA[0]);  // Write to Raise motor
      vTaskDelay(pdMS_TO_TICKS(10)); // Minor delay to allow motor to move
      RaisePA[2] = RaisePA[1];
      RaisePA[1] = RaisePA[0];  // Log the new position
      RaisePAWR = false;  // Reset flag
    }

    // Optional: If no movement needed, update motor to last position
    else
    {
      RaisePAWR = true;
      raiseServo.write(RaisePA[1]);
      vTaskDelay(pdMS_TO_TICKS(10));  // Minor wait
      RaisePAWR = false;
    }
  }
}

void KneeMotor_Control(void * pvParameters)   // TASK6
{
  for(;;)
  {
    // Wait for notification from Task 2
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // Perform angle update
    if(KneePA[0] != KneePA[1] && !KneePAWR)
    {
      KneePAWR = true;
      kneeServo.write(KneePA[0]);  // Write to Knee motor
      vTaskDelay(pdMS_TO_TICKS(10)); // Minor delay to allow motor to move
      KneePA[2] = KneePA[1];
      KneePA[1] = KneePA[0];  // Log the new position
      KneePAWR = false;  // Reset flag
    }

    // Optional: If no movement needed, update motor to last position
    else
    {
      KneePAWR = true;
      kneeServo.write(KneePA[1]);
      vTaskDelay(pdMS_TO_TICKS(10));  // Minor wait
      KneePAWR = false;
    }
  }
}

void InnerAnkleMotor_Control(void * pvParameters)   // TASK7
{
  for(;;)
  {
    // Wait for notification from Task 2
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // Perform angle update
    if(InnerAnklePA[0] != InnerAnklePA[1] && !InnerAnklePAWR)
    {
      InnerAnklePAWR = true;
      innerAnkleServo.write(InnerAnklePA[0]);  // Write to Inner Ankle motor
      vTaskDelay(pdMS_TO_TICKS(10)); // Minor delay to allow motor to move
      InnerAnklePA[2] = InnerAnklePA[1];
      InnerAnklePA[1] = InnerAnklePA[0];  // Log the new position
      InnerAnklePAWR = false;  // Reset flag
    }

    // Optional: If no movement needed, update motor to last position
    else
    {
      InnerAnklePAWR = true;
      innerAnkleServo.write(InnerAnklePA[1]);
      vTaskDelay(pdMS_TO_TICKS(10));  // Minor wait
      InnerAnklePAWR = false;
    }
  }
}

void OuterAnkleMotor_Control(void * pvParameters)   // TASK8
{
  for(;;)
  {
    // Wait for notification from Task 2
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // Perform angle update
    if(OuterAnklePA[0] != OuterAnklePA[1] && !OuterAnklePAWR)
    {
      OuterAnklePAWR = true;
      outerAnkleServo.write(OuterAnklePA[0]);  // Write to Outer Ankle motor
      vTaskDelay(pdMS_TO_TICKS(10)); // Minor delay to allow motor to move
      OuterAnklePA[2] = OuterAnklePA[1];
      OuterAnklePA[1] = OuterAnklePA[0];  // Log the new position
      OuterAnklePAWR = false;  // Reset flag
    }

    // Optional: If no movement needed, update motor to last position
    else
    {
      OuterAnklePAWR = true;
      outerAnkleServo.write(OuterAnklePA[1]);
      vTaskDelay(pdMS_TO_TICKS(10));  // Minor wait
      OuterAnklePAWR = false;
    }
  }
}


// void Motor_Control(void * pvParameters)
// {
//   Serial.print("LEG Motor_Control Running...Core: ");
//   Serial.println(xPortGetCoreID());

//   ServoCommand cmd;

//   for(;;)
//   {
//     ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
//     if(xQueueReceive(queue, &cmd, portMAX_DELAY))
//     {
//       uint8_t motorID = cmd.servo_id;
//       uint8_t angle = cmd.angle;

//       Serial.printf("Moving Servo 0x%02X to Angle %d\n", id, angle);

//       switch(motorID)
//       {
//         case 0x01:
//           swingServo.write(angle);break;
//         case 0x02:
//           raiseServo.write(angle);break;
//         case 0x03:
//           kneeServo.write(angle);break;
//         case 0x04:
//           innerAnkleServo.write(angle);break;
//         case 0x05:
//           outerAnkleServo.write(angle);break;
//         default:
//           Serial.printf("Unknown ServoID: 0x%02X\n", motorID);break;
//       }

//       vTaskDelay(pdMS_TO_TICKS(1));
//     }
//   }
  
  
//   // MotorAngle motorAngle;

//   // for(;;)
//   // {
//   //   ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

//   //   if(xSemaphoreTake(motorAngleMutex, portMAX_DELAY))
//   //   {
//   //     // Write to Motors
//   //     if(xQueueReceive(queue, &motorAngle, portMAX_DELAY))
//   //     {
//   //       SwingPA[0] = motorAngle.a;
//   //       RaisePA[0] = motorAngle.b;
//   //       KneePA[0] = motorAngle.c;
//   //       InnerAnklePA[0] = motorAngle.d;
//   //       OuterAnklePA[0] = motorAngle.e;
//   //     }
//   //     xSemaphoreGive(motorAngleMutex);
//   //   }

//   //   if ((SwingPA[0] != SwingPA[1]) && SwingPAWR == false)    // Shoulder Pitch 
//   //   {  
//   //     SwingPAWR = true;                                      // Flag enable
//   //     swingServo.write(SwingPA[0]);                                  //Write Angle
//   //     vTaskDelay(pdMS_TO_TICKS(1));                                              // Minor Wait
//   //     SwingPA[2] = SwingPA[1];
//   //     SwingPA[1] = SwingPA[0];                              // Log
//   //     SwingPAWR = false;                                     // Flag Disable
//   //   } 
//   //   else 
//   //   {
//   //     SwingPAWR = true;                                      // Flag enable
//   //     swingServo.write(SwingPA[1]);
//   //     vTaskDelay(pdMS_TO_TICKS(1));
//   //     SwingPAWR = false;                                     // Flag disable
//   //   }

//   //   if ((RaisePA[0] != RaisePA[1]) && RaisePAWR == false)    // Shoulder Pitch 
//   //   {  
//   //     RaisePAWR = true;                                      // Flag enable
//   //     raiseServo.write(RaisePA[0]);                                  //Write Angle
//   //     vTaskDelay(pdMS_TO_TICKS(1));                                              // Minor Wait
//   //     RaisePA[2] = RaisePA[1];
//   //     RaisePA[1] = RaisePA[0];                              // Log
//   //     RaisePAWR = false;                                     // Flag Disable
//   //   } 
//   //   else 
//   //   {
//   //     RaisePAWR = true;                                      // Flag enable
//   //     raiseServo.write(RaisePA[1]);
//   //     vTaskDelay(pdMS_TO_TICKS(1));
//   //     RaisePAWR = false;                                     // Flag disable
//   //   }

//   //   if ((KneePA[0] != KneePA[1]) && KneePAWR == false)    // Shoulder Pitch 
//   //   {  
//   //     KneePAWR = true;                                      // Flag enable
//   //     kneeServo.write(KneePA[0]);                                  //Write Angle
//   //     vTaskDelay(pdMS_TO_TICKS(1));                                              // Minor Wait
//   //     KneePA[2] = KneePA[1];
//   //     KneePA[1] = KneePA[0];                              // Log
//   //     KneePAWR = false;                                     // Flag Disable
//   //   } 
//   //   else 
//   //   {
//   //     KneePAWR = true;                                      // Flag enable
//   //     kneeServo.write(KneePA[1]);
//   //     vTaskDelay(pdMS_TO_TICKS(1));
//   //     KneePAWR = false;                                     // Flag disable
//   //   }

//   //   if ((InnerAnklePA[0] != InnerAnklePA[1]) && InnerAnklePAWR == false)    // Shoulder Pitch 
//   //   {  
//   //     InnerAnklePAWR = true;                                      // Flag enable
//   //     innerAnkleServo.write(InnerAnklePA[0]);                                  //Write Angle
//   //     vTaskDelay(pdMS_TO_TICKS(1));                                            // Minor Wait
//   //     InnerAnklePA[2] = InnerAnklePA[1];
//   //     InnerAnklePA[1] = InnerAnklePA[0];                              // Log
//   //     InnerAnklePAWR = false;                                     // Flag Disable
//   //   } 
//   //   else 
//   //   {
//   //     InnerAnklePAWR = true;                                      // Flag enable
//   //     innerAnkleServo.write(InnerAnklePA[1]);
//   //     vTaskDelay(pdMS_TO_TICKS(1));
//   //     InnerAnklePAWR = false;                                     // Flag disable
//   //   }

//   //   if ((OuterAnklePA[0] != OuterAnklePA[1]) && OuterAnklePAWR == false)    // Shoulder Pitch 
//   //   {  
//   //     OuterAnklePAWR = true;                                      // Flag enable
//   //     outerAnkleServo.write(OuterAnklePA[0]);                                  //Write Angle
//   //     vTaskDelay(pdMS_TO_TICKS(1));                                            // Minor Wait
//   //     OuterAnklePA[2] = OuterAnklePA[1];
//   //     OuterAnklePA[1] = OuterAnklePA[0];                              // Log
//   //     OuterAnklePAWR = false;                                     // Flag Disable
//   //   } 
//   //   else 
//   //   {
//   //     OuterAnklePAWR = true;                                      // Flag enable
//   //     outerAnkleServo.write(OuterAnklePA[1]);
//   //     vTaskDelay(pdMS_TO_TICKS(1));
//   //     OuterAnklePAWR = false;                                     // Flag disable
//   //   }
//   // }
// }

void Sensors(void * pvParameters)     // TASK9
{
  Serial.print("LEG Sensors...Core: ");
  Serial.println(xPortGetCoreID());
  for(;;)
  {

  }
}


void loop() {
  // Empty, RTOS is handling it!
}