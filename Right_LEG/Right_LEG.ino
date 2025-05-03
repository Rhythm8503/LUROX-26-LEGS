// // LEG Framework - Brian Ordonez
// // This message is coming from the NVIDIA Jetson!

// #include <ESP32Servo.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "freertos/event_groups.h"
// #include <ESP32SPISlave.h>
// #include "semphr.h"

// #define pin_Swing 41
// #define pin_Raise 20
// #define pin_Knee 21
// #define pin_InnerAnkle 36
// #define pin_OuterAnkle 38

// TaskHandle_t TASK1;    // SPI/UART Communication/ISR :    Core 0
// TaskHandle_t TASK2;    // Variable Read/Write and Math:   Core 0
// TaskHandle_t TASK3;    // Process MotorAngles:            Core 0
// TaskHandle_t TASK4;    // SwingMotorControl:              Core 0
// TaskHandle_t TASK5;    // RaiseMotorControl:              Core 1
// TaskHandle_t TASK6;    // KneeMotorControl:               Core 1
// TaskHandle_t TASK7;    // InnerAnkleMotorControl:         Core 1
// TaskHandle_t TASK8;    // OuterAnkleMotorControl:         Core 1
// //TaskHandle_t TASK9;    // IMU, ASS600. VS56LOX:           Core 1

// SemaphoreHandle_t sema;
// QueueHandle_t queue;           // A Queue designed to contain data from Jetson and transfer it to other tasks 
//                                // This prevents race conditions and is thread safe 

// ESP32SPISlave slave;
// static constexpr uint32_t BUFFER_SIZE {8};
// uint8_t spi_slave_tx_buf[BUFFER_SIZE];
// uint8_t spi_slave_rx_buf[BUFFER_SIZE];

// uint8_t controllerID;               // Byte 0: Controller Address
// typedef struct
// {
//   uint8_t M1;
//   uint8_t M2;
//   uint8_t M3;
//   uint8_t M4;
//   uint8_t M5;

// }MotorAngleReading;

// // Swing Servo Motor
// Servo swingServo;
// volatile float SwingPA[3] = {0.0, 0.0, 0.0};
// bool SwingPAWR = false;

// // Raise Servo Motor
// Servo raiseServo;
// volatile float RaisePA[3] = {0.0, 0.0, 0.0};
// bool RaisePAWR = false;

// // Knee Servo Motor
// Servo kneeServo;
// volatile float KneePA[3] = {0.0, 0.0, 0.0};
// bool KneePAWR = false;

// // InnerAnkle Servo Motor
// Servo innerAnkleServo;
// volatile float InnerAnklePA[3] = {0.0, 0.0, 0.0};
// bool InnerAnklePAWR = false;

// // OuterAnkle Servo Motor
// Servo outerAnkleServo;
// volatile float OuterAnklePA[3] = {0.0, 0.0, 0.0};
// bool OuterAnklePAWR = false;


// void setup() 
// {
//   Serial.begin(115200);

//   // SPI Slave Initialization
//   slave.setDataMode(SPI_MODE0);
//   slave.begin();

//   // Clear Buffers - SPI
//   memset(spi_slave_tx_buf, 0, BUFFER_SIZE);
//   memset(spi_slave_rx_buf, 0, BUFFER_SIZE);

//   sema = xSemaphoreCreateMutex();

//   // Create Queue for Data given from NVIDIA Jetsons
//   queue = xQueueCreate(5, sizeof(MotorAngleReading));

//   swingServo.attach(pin_Swing);
//   raiseServo.attach(pin_Raise);
//   kneeServo.attach(pin_Knee);
//   innerAnkleServo.attach(pin_InnerAnkle);
//   outerAnkleServo.attach(pin_OuterAnkle);

//   //Setting Up Tasks 1-4 
//   xTaskCreatePinnedToCore(
//                     Communication,   /* Task function. */
//                     "TASK1",     /* name of task. */
//                     10000,       /* Stack size of task */
//                     NULL,        /* parameter of the task */
//                     1,           /* priority of the task */
//                     &TASK1,      /* Task handle to keep track of created task */
//                     0);          /* pin task to core 0 */ 

//   xTaskCreatePinnedToCore(
//                     Read_Write,   /* Task function. */
//                     "TASK2",     /* name of task. */
//                     10000,       /* Stack size of task */
//                     NULL,        /* parameter of the task */
//                     1,           /* priority of the task */
//                     &TASK2,      /* Task handle to keep track of created task */
//                     0);          /* pin task to core 0 */

//   xTaskCreatePinnedToCore(
//                     Motor_Processing,   /* Task function. */
//                     "TASK3",     /* name of task. */
//                     10000,       /* Stack size of task */
//                     NULL,        /* parameter of the task */
//                     1,           /* priority of the task */
//                     &TASK3,      /* Task handle to keep track of created task */
//                     0);          /* pin task to core 0 */

//   xTaskCreatePinnedToCore(
//                     SwingMotor_Control,   /* Task function. */
//                     "TASK4",     /* name of task. */
//                     10000,       /* Stack size of task */
//                     NULL,        /* parameter of the task */
//                     1,           /* priority of the task */
//                     &TASK4,      /* Task handle to keep track of created task */
//                     0);          /* pin task to core 0 */

//   xTaskCreatePinnedToCore(
//                     RaiseMotor_Control,   /* Task function. */
//                     "TASK5",     /* name of task. */
//                     10000,       /* Stack size of task */
//                     NULL,        /* parameter of the task */
//                     1,           /* priority of the task */
//                     &TASK5,      /* Task handle to keep track of created task */
//                     1);          /* pin task to core 1 */

//   xTaskCreatePinnedToCore(
//                     KneeMotor_Control,   /* Task function. */
//                     "TASK6",     /* name of task. */
//                     10000,       /* Stack size of task */
//                     NULL,        /* parameter of the task */
//                     1,           /* priority of the task */
//                     &TASK6,      /* Task handle to keep track of created task */
//                     1);          /* pin task to core 1 */

//   xTaskCreatePinnedToCore(
//                     InnerAnkleMotor_Control,   /* Task function. */
//                     "TASK7",     /* name of task. */
//                     10000,       /* Stack size of task */
//                     NULL,        /* parameter of the task */
//                     1,           /* priority of the task */
//                     &TASK7,      /* Task handle to keep track of created task */
//                     1);          /* pin task to core 1 */

//   xTaskCreatePinnedToCore(
//                     OuterAnkleMotor_Control,   /* Task function. */
//                     "TASK8",     /* name of task. */
//                     10000,       /* Stack size of task */
//                     NULL,        /* parameter of the task */
//                     1,           /* priority of the task */
//                     &TASK8,      /* Task handle to keep track of created task */
//                     1);          /* pin task to core 1 */  

//   // xTaskCreatePinnedToCore(
//   //                   Sensors,   /* Task function. */
//   //                   "TASK9",     /* name of task. */
//   //                   10000,       /* Stack size of task */
//   //                   NULL,        /* parameter of the task */
//   //                   1,           /* priority of the task */
//   //                   &TASK9,      /* Task handle to keep track of created task */
//   //                   1);          /* pin task to core 1 */        
       
// }

// void Communication(void * pvParameters)
// {
//   Serial.print("LEG Communication Running...Core: ");
//   Serial.println(xPortGetCoreID());

//   for (;;)
//   {
//     Serial.println();
//     Serial.println("[Core 0]: Waiting for Handshake...");

//     // Wait for handshake
//     while (slave.transfer(spi_slave_tx_buf, spi_slave_rx_buf, BUFFER_SIZE) == 0);

//     if (spi_slave_rx_buf[0] == 0xA2) // Check if addressed to this controller
//     {
//       Serial.println("Right Leg Controller Selected!");

//       Serial.println("Receiving 8-byte command...");
//       while (slave.transfer(spi_slave_tx_buf, spi_slave_rx_buf, BUFFER_SIZE) == 0);

//       // Extract first 3 values
//       controllerID = spi_slave_rx_buf[0];

//       // Print raw received data
//       Serial.print("Printing Received Data: ");
//       for (int i = 0; i < 8; i++) 
//       {
//         Serial.print(spi_slave_rx_buf[i], HEX);
//         Serial.print(" ");
//       }
//       Serial.println();

//       // Print decoded info
//       Serial.printf("Controller Address: 0x%02X\n", controllerID);

//       // Notify task that data is ready
//       xTaskNotifyGive(TASK2);
//       vTaskDelay(pdMS_TO_TICKS(10));
//     }
//     else
//     {
//       Serial.println("[RIGHT LEG]: No message for me!");
//     }
//   }
// }

// void Read_Write(void * pvParameters)  // TASK2
// {
//   Serial.print("LEG Read/Write Running...Core: ");
//   Serial.println(xPortGetCoreID());

//   //ServoCommand cmd;
//   MotorAngleReading mar;

//   for(;;)
//   {
//     ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

//     if(xSemaphoreTake(sema, portMAX_DELAY))
//     {
//       //cmd.servo_id = spi_slave_rx_buf[1];
//       //cmd.angle = spi_slave_rx_buf[2];

//       mar.M1 = spi_slave_rx_buf[1];
//       mar.M2 = spi_slave_rx_buf[2];
//       mar.M3 = spi_slave_rx_buf[3];
//       mar.M4 = spi_slave_rx_buf[4];
//       mar.M5 = spi_slave_rx_buf[5];

//       xQueueSend(queue, &mar, portMAX_DELAY);
//       xSemaphoreGive(sema);
//     }

//     xTaskNotifyGive(TASK3);

//     vTaskDelay(pdMS_TO_TICKS(10));
//   }
// }

// void Motor_Processing(void * pvParameters)  // TASK3
// {
//   Serial.print("LEG Motor_Processing Running...Core: ");
//   Serial.println(xPortGetCoreID());

//   MotorAngleReading mar;

//   for(;;)
//   {
//     ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

//     if(xQueueReceive(queue, &mar, portMAX_DELAY))
//     {
//       // Store the angles in global arrays (or use the local motorAngle structure)
//       SwingPA[0] = (int)((mar.M1 / 255.0f) * 180);
//       RaisePA[0] = (int)((mar.M2 / 255.0f) * 180);
//       KneePA[0] = (int)((mar.M3 / 255.0f) * 180);
//       InnerAnklePA[0] = (int)((mar.M4 / 255.0f) * 180);
//       OuterAnklePA[0] = (int)((mar.M5 / 255.0f) * 180);

//       // Notify all motor tasks to update simultaneously
//       xTaskNotifyGive(TASK4);
//       xTaskNotifyGive(TASK5);
//       xTaskNotifyGive(TASK6);
//       xTaskNotifyGive(TASK7);
//       xTaskNotifyGive(TASK8);
//     }

//     vTaskDelay(pdMS_TO_TICKS(10));
//   }
// }

// void SwingMotor_Control(void * pvParameters)  // TASK4
// {
//   for(;;)
//   {
//     // Wait for notification from Task 2
//     ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

//     // Perform angle update
//     if(SwingPA[0] != SwingPA[1] && !SwingPAWR)
//     {
//       SwingPAWR = true;
//       swingServo.write(SwingPA[0]);  // Write to Swing motor
//       vTaskDelay(pdMS_TO_TICKS(10)); // Minor delay to allow motor to move
//       SwingPA[2] = SwingPA[1];
//       SwingPA[1] = SwingPA[0];  // Log the new position
//       SwingPAWR = false;  // Reset flag
//     }

//     // Optional: If no movement needed, update motor to last position
//     else
//     {
//       SwingPAWR = true;
//       swingServo.write(SwingPA[1]);
//       vTaskDelay(pdMS_TO_TICKS(10));  // Minor wait
//       SwingPAWR = false;
//     }
//   }
// }

// void RaiseMotor_Control(void * pvParameters)  // TASK5
// {
//   for(;;)
//   {
//     // Wait for notification from Task 2
//     ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

//     // Perform angle update
//     if(RaisePA[0] != RaisePA[1] && !RaisePAWR)
//     {
//       RaisePAWR = true;
//       raiseServo.write(RaisePA[0]);  // Write to Raise motor
//       vTaskDelay(pdMS_TO_TICKS(10)); // Minor delay to allow motor to move
//       RaisePA[2] = RaisePA[1];
//       RaisePA[1] = RaisePA[0];  // Log the new position
//       RaisePAWR = false;  // Reset flag
//     }

//     // Optional: If no movement needed, update motor to last position
//     else
//     {
//       RaisePAWR = true;
//       raiseServo.write(RaisePA[1]);
//       vTaskDelay(pdMS_TO_TICKS(10));  // Minor wait
//       RaisePAWR = false;
//     }
//   }
// }

// void KneeMotor_Control(void * pvParameters)   // TASK6
// {
//   for(;;)
//   {
//     // Wait for notification from Task 2
//     ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

//     // Perform angle update
//     if(KneePA[0] != KneePA[1] && !KneePAWR)
//     {
//       KneePAWR = true;
//       kneeServo.write(KneePA[0]);  // Write to Knee motor
//       vTaskDelay(pdMS_TO_TICKS(10)); // Minor delay to allow motor to move
//       KneePA[2] = KneePA[1];
//       KneePA[1] = KneePA[0];  // Log the new position
//       KneePAWR = false;  // Reset flag
//     }

//     // Optional: If no movement needed, update motor to last position
//     else
//     {
//       KneePAWR = true;
//       kneeServo.write(KneePA[1]);
//       vTaskDelay(pdMS_TO_TICKS(10));  // Minor wait
//       KneePAWR = false;
//     }
//   }
// }

// void InnerAnkleMotor_Control(void * pvParameters)   // TASK7
// {
//   for(;;)
//   {
//     // Wait for notification from Task 2
//     ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

//     // Perform angle update
//     if(InnerAnklePA[0] != InnerAnklePA[1] && !InnerAnklePAWR)
//     {
//       InnerAnklePAWR = true;
//       innerAnkleServo.write(InnerAnklePA[0]);  // Write to Inner Ankle motor
//       vTaskDelay(pdMS_TO_TICKS(10)); // Minor delay to allow motor to move
//       InnerAnklePA[2] = InnerAnklePA[1];
//       InnerAnklePA[1] = InnerAnklePA[0];  // Log the new position
//       InnerAnklePAWR = false;  // Reset flag
//     }

//     // Optional: If no movement needed, update motor to last position
//     else
//     {
//       InnerAnklePAWR = true;
//       innerAnkleServo.write(InnerAnklePA[1]);
//       vTaskDelay(pdMS_TO_TICKS(10));  // Minor wait
//       InnerAnklePAWR = false;
//     }
//   }
// }

// void OuterAnkleMotor_Control(void * pvParameters)   // TASK8
// {
//   for(;;)
//   {
//     // Wait for notification from Task 2
//     ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

//     // Perform angle update
//     if(OuterAnklePA[0] != OuterAnklePA[1] && !OuterAnklePAWR)
//     {
//       OuterAnklePAWR = true;
//       outerAnkleServo.write(OuterAnklePA[0]);  // Write to Outer Ankle motor
//       vTaskDelay(pdMS_TO_TICKS(10)); // Minor delay to allow motor to move
//       OuterAnklePA[2] = OuterAnklePA[1];
//       OuterAnklePA[1] = OuterAnklePA[0];  // Log the new position
//       OuterAnklePAWR = false;  // Reset flag
//     }

//     // Optional: If no movement needed, update motor to last position
//     else
//     {
//       OuterAnklePAWR = true;
//       outerAnkleServo.write(OuterAnklePA[1]);
//       vTaskDelay(pdMS_TO_TICKS(10));  // Minor wait
//       OuterAnklePAWR = false;
//     }
//   }
// }


// // void Motor_Control(void * pvParameters)
// // {
// //   Serial.print("LEG Motor_Control Running...Core: ");
// //   Serial.println(xPortGetCoreID());

// //   ServoCommand cmd;

// //   for(;;)
// //   {
// //     ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
// //     if(xQueueReceive(queue, &cmd, portMAX_DELAY))
// //     {
// //       uint8_t motorID = cmd.servo_id;
// //       uint8_t angle = cmd.angle;

// //       Serial.printf("Moving Servo 0x%02X to Angle %d\n", id, angle);

// //       switch(motorID)
// //       {
// //         case 0x01:
// //           swingServo.write(angle);break;
// //         case 0x02:
// //           raiseServo.write(angle);break;
// //         case 0x03:
// //           kneeServo.write(angle);break;
// //         case 0x04:
// //           innerAnkleServo.write(angle);break;
// //         case 0x05:
// //           outerAnkleServo.write(angle);break;
// //         default:
// //           Serial.printf("Unknown ServoID: 0x%02X\n", motorID);break;
// //       }

// //       vTaskDelay(pdMS_TO_TICKS(1));
// //     }
// //   }
  
  
// //   // MotorAngle motorAngle;

// //   // for(;;)
// //   // {
// //   //   ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

// //   //   if(xSemaphoreTake(motorAngleMutex, portMAX_DELAY))
// //   //   {
// //   //     // Write to Motors
// //   //     if(xQueueReceive(queue, &motorAngle, portMAX_DELAY))
// //   //     {
// //   //       SwingPA[0] = motorAngle.a;
// //   //       RaisePA[0] = motorAngle.b;
// //   //       KneePA[0] = motorAngle.c;
// //   //       InnerAnklePA[0] = motorAngle.d;
// //   //       OuterAnklePA[0] = motorAngle.e;
// //   //     }
// //   //     xSemaphoreGive(motorAngleMutex);
// //   //   }

// //   //   if ((SwingPA[0] != SwingPA[1]) && SwingPAWR == false)    // Shoulder Pitch 
// //   //   {  
// //   //     SwingPAWR = true;                                      // Flag enable
// //   //     swingServo.write(SwingPA[0]);                                  //Write Angle
// //   //     vTaskDelay(pdMS_TO_TICKS(1));                                              // Minor Wait
// //   //     SwingPA[2] = SwingPA[1];
// //   //     SwingPA[1] = SwingPA[0];                              // Log
// //   //     SwingPAWR = false;                                     // Flag Disable
// //   //   } 
// //   //   else 
// //   //   {
// //   //     SwingPAWR = true;                                      // Flag enable
// //   //     swingServo.write(SwingPA[1]);
// //   //     vTaskDelay(pdMS_TO_TICKS(1));
// //   //     SwingPAWR = false;                                     // Flag disable
// //   //   }

// //   //   if ((RaisePA[0] != RaisePA[1]) && RaisePAWR == false)    // Shoulder Pitch 
// //   //   {  
// //   //     RaisePAWR = true;                                      // Flag enable
// //   //     raiseServo.write(RaisePA[0]);                                  //Write Angle
// //   //     vTaskDelay(pdMS_TO_TICKS(1));                                              // Minor Wait
// //   //     RaisePA[2] = RaisePA[1];
// //   //     RaisePA[1] = RaisePA[0];                              // Log
// //   //     RaisePAWR = false;                                     // Flag Disable
// //   //   } 
// //   //   else 
// //   //   {
// //   //     RaisePAWR = true;                                      // Flag enable
// //   //     raiseServo.write(RaisePA[1]);
// //   //     vTaskDelay(pdMS_TO_TICKS(1));
// //   //     RaisePAWR = false;                                     // Flag disable
// //   //   }

// //   //   if ((KneePA[0] != KneePA[1]) && KneePAWR == false)    // Shoulder Pitch 
// //   //   {  
// //   //     KneePAWR = true;                                      // Flag enable
// //   //     kneeServo.write(KneePA[0]);                                  //Write Angle
// //   //     vTaskDelay(pdMS_TO_TICKS(1));                                              // Minor Wait
// //   //     KneePA[2] = KneePA[1];
// //   //     KneePA[1] = KneePA[0];                              // Log
// //   //     KneePAWR = false;                                     // Flag Disable
// //   //   } 
// //   //   else 
// //   //   {
// //   //     KneePAWR = true;                                      // Flag enable
// //   //     kneeServo.write(KneePA[1]);
// //   //     vTaskDelay(pdMS_TO_TICKS(1));
// //   //     KneePAWR = false;                                     // Flag disable
// //   //   }

// //   //   if ((InnerAnklePA[0] != InnerAnklePA[1]) && InnerAnklePAWR == false)    // Shoulder Pitch 
// //   //   {  
// //   //     InnerAnklePAWR = true;                                      // Flag enable
// //   //     innerAnkleServo.write(InnerAnklePA[0]);                                  //Write Angle
// //   //     vTaskDelay(pdMS_TO_TICKS(1));                                            // Minor Wait
// //   //     InnerAnklePA[2] = InnerAnklePA[1];
// //   //     InnerAnklePA[1] = InnerAnklePA[0];                              // Log
// //   //     InnerAnklePAWR = false;                                     // Flag Disable
// //   //   } 
// //   //   else 
// //   //   {
// //   //     InnerAnklePAWR = true;                                      // Flag enable
// //   //     innerAnkleServo.write(InnerAnklePA[1]);
// //   //     vTaskDelay(pdMS_TO_TICKS(1));
// //   //     InnerAnklePAWR = false;                                     // Flag disable
// //   //   }

// //   //   if ((OuterAnklePA[0] != OuterAnklePA[1]) && OuterAnklePAWR == false)    // Shoulder Pitch 
// //   //   {  
// //   //     OuterAnklePAWR = true;                                      // Flag enable
// //   //     outerAnkleServo.write(OuterAnklePA[0]);                                  //Write Angle
// //   //     vTaskDelay(pdMS_TO_TICKS(1));                                            // Minor Wait
// //   //     OuterAnklePA[2] = OuterAnklePA[1];
// //   //     OuterAnklePA[1] = OuterAnklePA[0];                              // Log
// //   //     OuterAnklePAWR = false;                                     // Flag Disable
// //   //   } 
// //   //   else 
// //   //   {
// //   //     OuterAnklePAWR = true;                                      // Flag enable
// //   //     outerAnkleServo.write(OuterAnklePA[1]);
// //   //     vTaskDelay(pdMS_TO_TICKS(1));
// //   //     OuterAnklePAWR = false;                                     // Flag disable
// //   //   }
// //   // }
// // }

// // void Sensors(void * pvParameters)     // TASK9
// // {
// //   Serial.print("LEG Sensors...Core: ");
// //   Serial.println(xPortGetCoreID());
// //   for(;;)
// //   {

// //   }
// // }


// void loop() {
//   // Empty, RTOS is handling it!
// }



// LEG Framework - Brian Ordonez
// This message is coming from the NVIDIA Jetson!

#include <ESP32Servo.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include <ESP32SPISlave.h>
#include "semphr.h"

#define pin_Swing 41
#define pin_Raise 20
#define pin_Knee 21
#define pin_InnerAnkle 36
#define pin_OuterAnkle 38

#define pin_Dir 1
#define pin_Step 2

const int stepDelayMicros = 4000;
long currentPosition = 0;

TaskHandle_t TASK1;    // SPI/UART Communication/ISR :    Core 0
TaskHandle_t TASK2;    // Variable Read/Write and Math:   Core 0
TaskHandle_t TASK3;    // Process MotorAngles:            Core 0
TaskHandle_t TASK4;    // SwingMotorControl:              Core 0
TaskHandle_t TASK5;    // RaiseMotorControl:              Core 1
TaskHandle_t TASK6;    // KneeMotorControl:               Core 1
TaskHandle_t TASK7;    // InnerAnkleMotorControl:         Core 1
TaskHandle_t TASK8;    // OuterAnkleMotorControl:         Core 1
TaskHandle_t TASK9;    // StepperMotorControl             Core 1

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
  uint8_t M6;

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

// Stepper Motor Variable Storage
volatile float StepperMotorPA[3] = {0.0, 0.0, 0.0};
bool StepperMotorPAWR = false;

EventGroupHandle_t syncEventGroup;

const int TASK4_START_BIT = (1 << 0);
const int TASK5_START_BIT = (1 << 1);
const int TASK6_START_BIT = (1 << 2);
const int TASK7_START_BIT = (1 << 3);
const int TASK8_START_BIT = (1 << 4);
const int TASK9_START_BIT = (1 << 5);

const int TASK4_END_BIT = (1 << 6);
const int TASK5_END_BIT = (1 << 7);
const int TASK6_END_BIT = (1 << 8);
const int TASK7_END_BIT = (1 << 9);
const int TASK8_END_BIT = (1 << 10);
const int TASK9_END_BIT = (1 << 11);


void setup() 
{
  Serial.begin(115200);

  // STEPPER Motor Initialization
  pinMode(pin_Dir, OUTPUT);
  pinMode(pin_Step, OUTPUT);

  // SPI Slave Initialization
  slave.setDataMode(SPI_MODE0);
  slave.begin();

  // Clear Buffers - SPI
  memset(spi_slave_tx_buf, 0, BUFFER_SIZE);
  memset(spi_slave_rx_buf, 0, BUFFER_SIZE);

  sema = xSemaphoreCreateMutex();

  // Create Queue for Data given from NVIDIA Jetsons
  queue = xQueueCreate(6, sizeof(MotorAngleReading));

  swingServo.attach(pin_Swing);
  raiseServo.attach(pin_Raise);
  kneeServo.attach(pin_Knee);
  innerAnkleServo.attach(pin_InnerAnkle);
  outerAnkleServo.attach(pin_OuterAnkle);

  swingServo.write(93);
  raiseServo.write(87);
  kneeServo.write(107);
  innerAnkleServo.write(82);
  outerAnkleServo.write(105);

  syncEventGroup = xEventGroupCreate();

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
                    StepperMotor_Control,   /* Task function. */
                    "TASK9",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &TASK9,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 1 */

  xEventGroupSetBits(syncEventGroup, TASK4_START_BIT | TASK5_START_BIT | TASK6_START_BIT | TASK7_START_BIT | TASK8_START_BIT | TASK9_START_BIT);

}

void Communication(void * pvParameters)
{
  Serial.print("RIGHT LEG Communication Running...Core: ");
  Serial.println(xPortGetCoreID());

  for (;;)
  {
    Serial.println();
    Serial.println("[Core 0]: Waiting for Handshake...");

    // Wait for handshake
    while (slave.transfer(spi_slave_tx_buf, spi_slave_rx_buf, BUFFER_SIZE) == 0);

    if (spi_slave_rx_buf[0] == 0xA3) //Right_LEG   Before 0xA4
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
      vTaskDelay(pdMS_TO_TICKS(10));
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
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    if(xSemaphoreTake(sema, portMAX_DELAY))
    {
      //cmd.servo_id = spi_slave_rx_buf[1];
      //cmd.angle = spi_slave_rx_buf[2];

      mar.M1 = spi_slave_rx_buf[1];
      mar.M2 = spi_slave_rx_buf[2];
      mar.M3 = spi_slave_rx_buf[3];
      mar.M4 = spi_slave_rx_buf[4];
      mar.M5 = spi_slave_rx_buf[5];
      mar.M6 = spi_slave_rx_buf[6];

      xQueueSend(queue, &mar, portMAX_DELAY);
      xSemaphoreGive(sema);
    }

    xTaskNotifyGive(TASK3);

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void Motor_Processing(void * pvParameters)  // TASK3
{
  Serial.print("LEG Motor_Processing Running...Core: ");
  Serial.println(xPortGetCoreID());

  MotorAngleReading mar;

  for(;;)
  {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    if(xQueueReceive(queue, &mar, portMAX_DELAY))
    {
      // Store the angles in global arrays (or use the local motorAngle structure)
      SwingPA[0] = (int)((mar.M1 / 255.0f) * 180);
      RaisePA[0] = (int)((mar.M2 / 255.0f) * 180);
      KneePA[0] = (int)((mar.M3 / 255.0f) * 180);
      InnerAnklePA[0] = (int)((mar.M4 / 255.0f) * 180);
      OuterAnklePA[0] = (int)((mar.M5 / 255.0f) * 180);
      StepperMotorPA[0] = (int)((mar.M6 - 128) * (50.0f / 127.0f));

      // Notify all motor tasks to update simultaneously
      xTaskNotifyGive(TASK4);
      xTaskNotifyGive(TASK5);
      xTaskNotifyGive(TASK6);
      xTaskNotifyGive(TASK7);
      xTaskNotifyGive(TASK8);
      xTaskNotifyGive(TASK9);
    }

    vTaskDelay(pdMS_TO_TICKS(10));
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
      xEventGroupWaitBits(syncEventGroup, TASK4_START_BIT | TASK5_START_BIT | TASK6_START_BIT | TASK7_START_BIT | TASK8_START_BIT | TASK9_START_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
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
      xEventGroupWaitBits(syncEventGroup, TASK4_START_BIT | TASK5_START_BIT | TASK6_START_BIT | TASK7_START_BIT | TASK8_START_BIT | TASK9_START_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
      SwingPAWR = true;
      swingServo.write(SwingPA[1]);
      vTaskDelay(pdMS_TO_TICKS(10));  // Minor wait
      SwingPAWR = false;
    }

    xEventGroupSetBits(syncEventGroup, TASK4_END_BIT);
    xEventGroupWaitBits(syncEventGroup, TASK5_END_BIT | TASK6_END_BIT | TASK7_END_BIT | TASK8_END_BIT | TASK9_END_BIT, pdTRUE, pdFALSE, portMAX_DELAY);

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
      xEventGroupWaitBits(syncEventGroup, TASK4_START_BIT | TASK5_START_BIT | TASK6_START_BIT | TASK7_START_BIT | TASK8_START_BIT | TASK9_START_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
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
      xEventGroupWaitBits(syncEventGroup, TASK4_START_BIT | TASK5_START_BIT | TASK6_START_BIT | TASK7_START_BIT | TASK8_START_BIT | TASK9_START_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
      RaisePAWR = true;
      raiseServo.write(RaisePA[1]);
      vTaskDelay(pdMS_TO_TICKS(10));  // Minor wait
      RaisePAWR = false;
    }

    xEventGroupSetBits(syncEventGroup, TASK5_END_BIT);
    xEventGroupWaitBits(syncEventGroup, TASK4_END_BIT | TASK6_END_BIT | TASK7_END_BIT | TASK8_END_BIT | TASK9_END_BIT, pdTRUE, pdFALSE, portMAX_DELAY);
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
      xEventGroupWaitBits(syncEventGroup, TASK4_START_BIT | TASK5_START_BIT | TASK6_START_BIT | TASK7_START_BIT | TASK8_START_BIT | TASK9_START_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
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
      xEventGroupWaitBits(syncEventGroup, TASK4_START_BIT | TASK5_START_BIT | TASK6_START_BIT | TASK7_START_BIT | TASK8_START_BIT | TASK9_START_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
      KneePAWR = true;
      kneeServo.write(KneePA[1]);
      vTaskDelay(pdMS_TO_TICKS(10));  // Minor wait
      KneePAWR = false;
    }

    xEventGroupSetBits(syncEventGroup, TASK6_END_BIT);
    xEventGroupWaitBits(syncEventGroup, TASK4_END_BIT | TASK5_END_BIT | TASK7_END_BIT | TASK8_END_BIT | TASK9_END_BIT, pdTRUE, pdFALSE, portMAX_DELAY);
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
      xEventGroupWaitBits(syncEventGroup, TASK4_START_BIT | TASK5_START_BIT | TASK6_START_BIT | TASK7_START_BIT | TASK8_START_BIT | TASK9_START_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
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
      xEventGroupWaitBits(syncEventGroup, TASK4_START_BIT | TASK5_START_BIT | TASK6_START_BIT | TASK7_START_BIT | TASK8_START_BIT | TASK9_START_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
      InnerAnklePAWR = true;
      innerAnkleServo.write(InnerAnklePA[1]);
      vTaskDelay(pdMS_TO_TICKS(10));  // Minor wait
      InnerAnklePAWR = false;
    }

    xEventGroupSetBits(syncEventGroup, TASK7_END_BIT);
    xEventGroupWaitBits(syncEventGroup, TASK4_END_BIT | TASK5_END_BIT | TASK6_END_BIT | TASK8_END_BIT | TASK9_END_BIT, pdTRUE, pdFALSE, portMAX_DELAY);
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
      xEventGroupWaitBits(syncEventGroup, TASK4_START_BIT | TASK5_START_BIT | TASK6_START_BIT | TASK7_START_BIT | TASK8_START_BIT | TASK9_START_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
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
      xEventGroupWaitBits(syncEventGroup, TASK4_START_BIT | TASK5_START_BIT | TASK6_START_BIT | TASK7_START_BIT | TASK8_START_BIT | TASK9_START_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
      OuterAnklePAWR = true;
      outerAnkleServo.write(OuterAnklePA[1]);
      vTaskDelay(pdMS_TO_TICKS(10));  // Minor wait
      OuterAnklePAWR = false;
    }

    xEventGroupSetBits(syncEventGroup, TASK8_END_BIT);
    xEventGroupWaitBits(syncEventGroup, TASK4_END_BIT | TASK5_END_BIT | TASK6_END_BIT | TASK7_END_BIT | TASK9_END_BIT, pdTRUE, pdFALSE, portMAX_DELAY);
  }
}

void StepperMotor_Control(void * pvParameters)    // TASK9
{
  for(;;)
  {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    if(StepperMotorPA[0] != StepperMotorPA[1] && !StepperMotorPAWR)
    {
      xEventGroupWaitBits(syncEventGroup, TASK4_START_BIT | TASK5_START_BIT | TASK6_START_BIT | TASK7_START_BIT | TASK8_START_BIT | TASK9_START_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
      StepperMotorPAWR = true;
      long stepsToMove = (StepperMotorPA[0]) - currentPosition;

      digitalWrite(pin_Dir, stepsToMove > 0 ? HIGH : LOW);
      stepsToMove = abs(stepsToMove);

      for(long i = 0; i < stepsToMove; i++)
      {
        digitalWrite(pin_Step, HIGH);
        delayMicroseconds(stepDelayMicros);
        digitalWrite(pin_Step, LOW);
        delayMicroseconds(stepDelayMicros);
      } 

      currentPosition =  StepperMotorPA[0];

      vTaskDelay(pdMS_TO_TICKS(10));
      StepperMotorPA[2] = StepperMotorPA[1];
      StepperMotorPA[1] = StepperMotorPA[0];
      StepperMotorPAWR = false;
    }

    else
    {
      xEventGroupWaitBits(syncEventGroup, TASK4_START_BIT | TASK5_START_BIT | TASK6_START_BIT | TASK7_START_BIT | TASK8_START_BIT | TASK9_START_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
      StepperMotorPAWR = true;
      long stepsToMove = (StepperMotorPA[1]) - currentPosition;

      digitalWrite(pin_Dir, stepsToMove > 0 ? HIGH : LOW);
      stepsToMove = abs(stepsToMove);

      for(long i = 0; i < stepsToMove; i++)
      {
        digitalWrite(pin_Step, HIGH);
        delayMicroseconds(stepDelayMicros);
        digitalWrite(pin_Step, LOW);
        delayMicroseconds(stepDelayMicros);
      } 

      currentPosition =  StepperMotorPA[1];
      vTaskDelay(pdMS_TO_TICKS(10));
      StepperMotorPAWR = false;
    }

    xEventGroupSetBits(syncEventGroup, TASK9_END_BIT);
    xEventGroupWaitBits(syncEventGroup, TASK4_END_BIT | TASK5_END_BIT | TASK6_END_BIT | TASK7_END_BIT | TASK8_END_BIT, pdTRUE, pdFALSE, portMAX_DELAY);
  }
}


void loop() {
  // Empty, RTOS is handling it!
}
