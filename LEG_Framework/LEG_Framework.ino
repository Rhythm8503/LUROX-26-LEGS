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
TaskHandle_t TASK3;    // Servo & Stepper Motor Control:  Core 1
TaskHandle_t TASK4;    // IMU, ASS600. VS56LOX:           Core 1

SemaphoreHandle_t motorAngleMutex;

ESP32SPISlave slave;
static constexpr uint32_t BUFFER_SIZE {8};
uint8_t spi_slave_tx_buf[BUFFER_SIZE];
uint8_t spi_slave_rx_buf[BUFFER_SIZE];

uint8_t controllerID;               // Byte 0: Controller Address
uint8_t servoID;                    // Byte 1: Servo ID
uint8_t angleValue;                 // Byte 2: Target Angle (0–180 degrees)
                                    // Byte 3–7: [Reserved for future use]

QueueHandle_t queue;           // A Queue designed to contain data from Jetson and transfer it to other tasks 
                               // This prevents race conditions and is thread safe 

typedef struct
{
  float a, b, c, d, e;

}MotorAngle;

typedef struct
{
  float a, b, c, d, e;

}AngleMeasurements;

typedef struct
{
  uint8_t servo_id;
  uint8_t angle;

}ServoCommand;

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

  motorAngleMutex = xSemaphoreCreateMutex();

  // Create Queue for Data given from NVIDIA Jetsons
  queue = xQueueCreate(5, sizeof(AngleMeasurements));

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
      Serial.println("Left Leg Controller Selected!");

      Serial.println("Receiving 8-byte command...");
      while (slave.transfer(spi_slave_tx_buf, spi_slave_rx_buf, BUFFER_SIZE) == 0);

      // Extract first 3 values
      controllerID = spi_slave_rx_buf[0];
      servoID = spi_slave_rx_buf[1];
      angleValue = spi_slave_rx_buf[2];

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
      Serial.printf("Servo ID: 0x%02X\n", servoID);
      Serial.printf("Target Angle: %d\n", angleValue);

      // Notify task that data is ready
      xTaskNotifyGive(TASK2);
      vTaskDelay(pdMS_TO_TICKS(100));
    }
    else
    {
      Serial.println("[LEFT LEG]: No message for me!");
    }
  }
}

void Read_Write(void * pvParameters)
{
  Serial.print("LEG Read/Write Running...Core: ");
  Serial.println(xPortGetCoreID());

  ServoCommand cmd;

  for(;;)
  {
    if(xSemaphoreTake(motorAngleMutex, portMAX_DELAY))
    {
      cmd.servo_id = spi_slave_rx_buf[1];
      cmd.angle = spi_slave_rx_buf[2];

      xQueueSend(queue, &cmd, portMAX_DELAY);
      xSemaphoreGive(motorAngleMutex);
    }

    xTaskNotifyGive(TASK3);

    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    //FROM HERE, you need to send sensor data from TASK4 back to the Jetson using SPI

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void Motor_Control(void * pvParameters)
{
  Serial.print("LEG Motor_Control Running...Core: ");
  Serial.println(xPortGetCoreID());

  ServoCommand cmd;

  for(;;)
  {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if(xQueueReceive(queue, &cmd, portMAX_DELAY))
    {
      uint8_t motorID = cmd.servo_id;
      uint8_t angle = cmd.angle;

      Serial.printf("Moving Servo 0x%02X to Angle %d\n", id, angle);

      switch(motorID)
      {
        case 0x01:
          swingServo.write(angle);break;
        case 0x02:
          raiseServo.write(angle);break;
        case 0x03:
          kneeServo.write(angle);break;
        case 0x04:
          innerAnkleServo.write(angle);break;
        case 0x05:
          outerAnkleServo.write(angle);break;
        default:
          Serial.printf("Unknown ServoID: 0x%02X\n", motorID);break;
      }

      vTaskDelay(pdMS_TO_TICKS(1));
    }
  }
  
  
  // MotorAngle motorAngle;

  // for(;;)
  // {
  //   ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

  //   if(xSemaphoreTake(motorAngleMutex, portMAX_DELAY))
  //   {
  //     // Write to Motors
  //     if(xQueueReceive(queue, &motorAngle, portMAX_DELAY))
  //     {
  //       SwingPA[0] = motorAngle.a;
  //       RaisePA[0] = motorAngle.b;
  //       KneePA[0] = motorAngle.c;
  //       InnerAnklePA[0] = motorAngle.d;
  //       OuterAnklePA[0] = motorAngle.e;
  //     }
  //     xSemaphoreGive(motorAngleMutex);
  //   }

  //   if ((SwingPA[0] != SwingPA[1]) && SwingPAWR == false)    // Shoulder Pitch 
  //   {  
  //     SwingPAWR = true;                                      // Flag enable
  //     swingServo.write(SwingPA[0]);                                  //Write Angle
  //     vTaskDelay(pdMS_TO_TICKS(1));                                              // Minor Wait
  //     SwingPA[2] = SwingPA[1];
  //     SwingPA[1] = SwingPA[0];                              // Log
  //     SwingPAWR = false;                                     // Flag Disable
  //   } 
  //   else 
  //   {
  //     SwingPAWR = true;                                      // Flag enable
  //     swingServo.write(SwingPA[1]);
  //     vTaskDelay(pdMS_TO_TICKS(1));
  //     SwingPAWR = false;                                     // Flag disable
  //   }

  //   if ((RaisePA[0] != RaisePA[1]) && RaisePAWR == false)    // Shoulder Pitch 
  //   {  
  //     RaisePAWR = true;                                      // Flag enable
  //     raiseServo.write(RaisePA[0]);                                  //Write Angle
  //     vTaskDelay(pdMS_TO_TICKS(1));                                              // Minor Wait
  //     RaisePA[2] = RaisePA[1];
  //     RaisePA[1] = RaisePA[0];                              // Log
  //     RaisePAWR = false;                                     // Flag Disable
  //   } 
  //   else 
  //   {
  //     RaisePAWR = true;                                      // Flag enable
  //     raiseServo.write(RaisePA[1]);
  //     vTaskDelay(pdMS_TO_TICKS(1));
  //     RaisePAWR = false;                                     // Flag disable
  //   }

  //   if ((KneePA[0] != KneePA[1]) && KneePAWR == false)    // Shoulder Pitch 
  //   {  
  //     KneePAWR = true;                                      // Flag enable
  //     kneeServo.write(KneePA[0]);                                  //Write Angle
  //     vTaskDelay(pdMS_TO_TICKS(1));                                              // Minor Wait
  //     KneePA[2] = KneePA[1];
  //     KneePA[1] = KneePA[0];                              // Log
  //     KneePAWR = false;                                     // Flag Disable
  //   } 
  //   else 
  //   {
  //     KneePAWR = true;                                      // Flag enable
  //     kneeServo.write(KneePA[1]);
  //     vTaskDelay(pdMS_TO_TICKS(1));
  //     KneePAWR = false;                                     // Flag disable
  //   }

  //   if ((InnerAnklePA[0] != InnerAnklePA[1]) && InnerAnklePAWR == false)    // Shoulder Pitch 
  //   {  
  //     InnerAnklePAWR = true;                                      // Flag enable
  //     innerAnkleServo.write(InnerAnklePA[0]);                                  //Write Angle
  //     vTaskDelay(pdMS_TO_TICKS(1));                                            // Minor Wait
  //     InnerAnklePA[2] = InnerAnklePA[1];
  //     InnerAnklePA[1] = InnerAnklePA[0];                              // Log
  //     InnerAnklePAWR = false;                                     // Flag Disable
  //   } 
  //   else 
  //   {
  //     InnerAnklePAWR = true;                                      // Flag enable
  //     innerAnkleServo.write(InnerAnklePA[1]);
  //     vTaskDelay(pdMS_TO_TICKS(1));
  //     InnerAnklePAWR = false;                                     // Flag disable
  //   }

  //   if ((OuterAnklePA[0] != OuterAnklePA[1]) && OuterAnklePAWR == false)    // Shoulder Pitch 
  //   {  
  //     OuterAnklePAWR = true;                                      // Flag enable
  //     outerAnkleServo.write(OuterAnklePA[0]);                                  //Write Angle
  //     vTaskDelay(pdMS_TO_TICKS(1));                                            // Minor Wait
  //     OuterAnklePA[2] = OuterAnklePA[1];
  //     OuterAnklePA[1] = OuterAnklePA[0];                              // Log
  //     OuterAnklePAWR = false;                                     // Flag Disable
  //   } 
  //   else 
  //   {
  //     OuterAnklePAWR = true;                                      // Flag enable
  //     outerAnkleServo.write(OuterAnklePA[1]);
  //     vTaskDelay(pdMS_TO_TICKS(1));
  //     OuterAnklePAWR = false;                                     // Flag disable
  //   }
  // }
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
  // Empty, RTOS is handling it!
}