// LEG Framework - Brian Ordonez

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
static constexpr uint32_t BUFFER_SIZE {5};
uint8_t spi_slave_tx_buf[BUFFER_SIZE];
uint8_t spi_slave_rx_buf[BUFFER_SIZE];

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

  // Position pos;
  // unsigned char data;

  for(;;)
  {
    Serial.println("[Core 0]: Waiting for Handshake...");

    while(slave.transfer(spi_slave_tx_buf, spi_slave_rx_buf, BUFFER_SIZE) == 0);

    if(spi_slave_rx_buf[0] == 0xA1)
    {
      Serial.println("[Core 0]: Handshake received, sending ACK...");
      spi_slave_tx_buf[0] = 0xA5;
      slave.transfer(spi_slave_tx_buf, spi_slave_rx_buf, BUFFER_SIZE);
    }

    // xSemaphoreTake(motorAngleMutex, portMAX_DELAY);
    // // Reading Values 
    // xSemaphoreGive(motorAngleMutex);
    // Serial.println("[Core 0]: Received Motor Angles");
  }
}





    // delay(500);

    // const size_t received_bytes = slave.transfer(spi_slave_tx_buf, spi_slave_rx_buf, BUFFER_SIZE);
    //     // show received data
        
    //     data = spi_slave_rx_buf[0];
    //     pos.x = map(data, 0, 255, 0, 180);
    //     pos.y = map();
         

    // xQueueSend(queue, &pos, portMAX_DELAY);
    // vTaskDelay(pdMS_TO_TICKS(100));
//   } 
// }

void Read_Write(void * pvParameters)
{
  Serial.print("LEG Read/Write Running...Core: ");
  Serial.println(xPortGetCoreID());

  AngleMeasurements angleMeasure;

  for(;;)
  {
    if(xSemaphoreTake(motorAngleMutex, portMAX_DELAY))
    {
      angleMeasure.a = spi_slave_rx_buf[0];
      angleMeasure.b = spi_slave_rx_buf[1];
      angleMeasure.c = spi_slave_rx_buf[2];
      angleMeasure.d = spi_slave_rx_buf[3];
      angleMeasure.e = spi_slave_rx_buf[4]; 
      xQueueSend(queue, &angleMeasure, portMAX_DELAY);
      //memcpy(motorAngle, &spi_slave_rx_buf[1], sizeof(motorAngle));
      xSemaphoreGive(motorAngleMutex);
    }

    xTaskNotifyGive(TASK3);

    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    //FROM HERE, you need to send sensor data from TASK4 back to the Jetson using SPI

    vTaskDelay(pdMS_TO_TICKS(100));


    // if(xQueueReceive(queue, &resPosition, portMAX_DELAY))
    // {

    // }
    // vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void Motor_Control(void * pvParameters)
{
  Serial.print("LEG Motor_Control Running...Core: ");
  Serial.println(xPortGetCoreID());

  MotorAngle motorAngle;

  for(;;)
  {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    if(xSemaphoreTake(motorAngleMutex, portMAX_DELAY))
    {
      // Write to Motors
      if(xQueueReceive(queue, &motorAngle, portMAX_DELAY))
      {
        SwingPA[0] = motorAngle.a;
        RaisePA[0] = motorAngle.b;
        KneePA[0] = motorAngle.c;
        InnerAnklePA[0] = motorAngle.d;
        OuterAnklePA[0] = motorAngle.e;
      }
      xSemaphoreGive(motorAngleMutex);
    }

    if ((SwingPA[0] != SwingPA[1]) && SwingPAWR == false)    // Shoulder Pitch 
    {  
      SwingPAWR = true;                                      // Flag enable
      swingServo.write(SwingPA[0]);                                  //Write Angle
      vTaskDelay(pdMS_TO_TICKS(1));                                              // Minor Wait
      SwingPA[2] = SwingPA[1];
      SwingPA[1] = SwingPA[0];                              // Log
      SwingPAWR = false;                                     // Flag Disable
    } 
    else 
    {
      SwingPAWR = true;                                      // Flag enable
      swingServo.write(SwingPA[1]);
      vTaskDelay(pdMS_TO_TICKS(1));
      SwingPAWR = false;                                     // Flag disable
    }

    if ((RaisePA[0] != RaisePA[1]) && RaisePAWR == false)    // Shoulder Pitch 
    {  
      RaisePAWR = true;                                      // Flag enable
      raiseServo.write(RaisePA[0]);                                  //Write Angle
      vTaskDelay(pdMS_TO_TICKS(1));                                              // Minor Wait
      RaisePA[2] = RaisePA[1];
      RaisePA[1] = RaisePA[0];                              // Log
      RaisePAWR = false;                                     // Flag Disable
    } 
    else 
    {
      RaisePAWR = true;                                      // Flag enable
      raiseServo.write(RaisePA[1]);
      vTaskDelay(pdMS_TO_TICKS(1));
      RaisePAWR = false;                                     // Flag disable
    }

    if ((KneePA[0] != KneePA[1]) && KneePAWR == false)    // Shoulder Pitch 
    {  
      KneePAWR = true;                                      // Flag enable
      kneeServo.write(KneePA[0]);                                  //Write Angle
      vTaskDelay(pdMS_TO_TICKS(1));                                              // Minor Wait
      KneePA[2] = KneePA[1];
      KneePA[1] = KneePA[0];                              // Log
      KneePAWR = false;                                     // Flag Disable
    } 
    else 
    {
      KneePAWR = true;                                      // Flag enable
      kneeServo.write(KneePA[1]);
      vTaskDelay(pdMS_TO_TICKS(1));
      KneePAWR = false;                                     // Flag disable
    }

    if ((InnerAnklePA[0] != InnerAnklePA[1]) && InnerAnklePAWR == false)    // Shoulder Pitch 
    {  
      InnerAnklePAWR = true;                                      // Flag enable
      innerAnkleServo.write(InnerAnklePA[0]);                                  //Write Angle
      vTaskDelay(pdMS_TO_TICKS(1));                                            // Minor Wait
      InnerAnklePA[2] = InnerAnklePA[1];
      InnerAnklePA[1] = InnerAnklePA[0];                              // Log
      InnerAnklePAWR = false;                                     // Flag Disable
    } 
    else 
    {
      InnerAnklePAWR = true;                                      // Flag enable
      innerAnkleServo.write(InnerAnklePA[1]);
      vTaskDelay(pdMS_TO_TICKS(1));
      InnerAnklePAWR = false;                                     // Flag disable
    }

    if ((OuterAnklePA[0] != OuterAnklePA[1]) && OuterAnklePAWR == false)    // Shoulder Pitch 
    {  
      OuterAnklePAWR = true;                                      // Flag enable
      outerAnkleServo.write(OuterAnklePA[0]);                                  //Write Angle
      vTaskDelay(pdMS_TO_TICKS(1));                                            // Minor Wait
      OuterAnklePA[2] = OuterAnklePA[1];
      OuterAnklePA[1] = OuterAnklePA[0];                              // Log
      OuterAnklePAWR = false;                                     // Flag Disable
    } 
    else 
    {
      OuterAnklePAWR = true;                                      // Flag enable
      outerAnkleServo.write(OuterAnklePA[1]);
      vTaskDelay(pdMS_TO_TICKS(1));
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
  // Empty, RTOS is handling it!
}