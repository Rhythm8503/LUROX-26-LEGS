#include <ESP32SPISlave.h>
#include <ESP32Servo.h>

ESP32SPISlave slave;

static constexpr uint32_t BUFFER_SIZE {8};
uint8_t spi_slave_tx_buf[BUFFER_SIZE];
uint8_t spi_slave_rx_buf[BUFFER_SIZE];

#define LED 39
#define dirPin 1
#define stepPin 2
#define servoPin 7

Servo testServo;

const int stepDelayMicros = 1000; // Microseconds per step
long currentPosition = 0;         // Current motor position in steps (0–200)

void setup() {
    Serial.begin(115200);
    Serial.println("Serial Enabled");
    delay(2000);

    pinMode(LED, OUTPUT);
    pinMode(dirPin, OUTPUT);
    pinMode(stepPin, OUTPUT);

    // SPI setup (HSPI by default)
    slave.setDataMode(SPI_MODE0);
    slave.begin();
    Serial.println("SPI Began");

    memset(spi_slave_tx_buf, 0, BUFFER_SIZE);
    memset(spi_slave_rx_buf, 0, BUFFER_SIZE);

    testServo.attach(servoPin);


    Serial.println("Waiting for SPI commands...");
}

void loop() {
    const size_t received_bytes = slave.transfer(spi_slave_tx_buf, spi_slave_rx_buf, BUFFER_SIZE);

    // Read the first byte
    uint8_t inputValue = spi_slave_rx_buf[0];
    uint8_t servoInput = spi_slave_rx_buf[1];

    // Map from 0x00–0xFF (0–255) to 0–200 step position
    long targetPosition = map(inputValue, 0, 255, 0, 200);
    long servoPosition = map(servoInput, 0, 255, 0, 180);

    // Serial.print("Input Byte: 0x");
    // if (inputValue < 0x10) Serial.print("0");
    // Serial.print(inputValue, HEX);
    // Serial.print(" | Mapped Target Position: ");
    // Serial.println(targetPosition);

    Serial.print("Input Byte 0: 0x");
    if (inputValue < 0x10) Serial.print("0");
    Serial.print(inputValue, HEX);

    Serial.print(" | Input Byte 1: 0x");
    if (servoInput < 0x10) Serial.print("0");
    Serial.print(servoInput, HEX);

    Serial.print(" | Mapped Step Target: ");
    Serial.print(targetPosition);
    Serial.print(" | Mapped Servo Angle: ");
    Serial.println(servoPosition);

    moveToPosition(targetPosition);
    moveToAngle(servoPosition);

    delay(1000); // Optional delay
}

void moveToPosition(long targetPosition) {
    long stepsToMove = targetPosition - currentPosition;

    if (stepsToMove == 0) {
        Serial.println("Already at target position.");
        return;
    }

    // Set direction
    digitalWrite(dirPin, stepsToMove > 0 ? HIGH : LOW);
    stepsToMove = abs(stepsToMove);

    // Step motor
    for (long i = 0; i < stepsToMove; i++) {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(stepDelayMicros);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(stepDelayMicros);
    }

    currentPosition = targetPosition;
    Serial.print("Moved to position: ");
    Serial.println(currentPosition);
}

void moveToAngle(long angle)
{
    testServo.write(angle);
}


// #include <ESP32SPISlave.h>

// ESP32SPISlave slave;

// static constexpr uint32_t BUFFER_SIZE {8};
// uint8_t spi_slave_tx_buf[BUFFER_SIZE];
// uint8_t spi_slave_rx_buf[BUFFER_SIZE];

// #define LED 39

// #define dirPin 1
// #define stepPin 2

// const int stepDelayMicros = 1000; // Step speed
// long currentPosition = 0;         // Track current position

// void setup() {
//     Serial.begin(115200);
//     Serial.println("Serial Enabled");
//     delay(2000);
//     pinMode(LED, OUTPUT);

//     // SPI setup (HSPI by default)
//     slave.setDataMode(SPI_MODE0);
//     slave.begin();

//     Serial.println("SPI Began");

//     // Clear buffers
//     memset(spi_slave_tx_buf, 0, BUFFER_SIZE);
//     memset(spi_slave_rx_buf, 0, BUFFER_SIZE);

//     pinMode(dirPin, OUTPUT);
//     pinMode(stepPin, OUTPUT);
//     //Serial.begin(115200);
//     Serial.println("Enter target position in steps (e.g., 200, -400, etc):");
// }

// void loop() {
//     delay(1000);

//     const size_t received_bytes = slave.transfer(spi_slave_tx_buf, spi_slave_rx_buf, BUFFER_SIZE);

//     // Print first two bytes in hex
//     Serial.print("Command Received: 0x");
//     if (spi_slave_rx_buf[0] < 0x10) Serial.print("0"); // zero pad
//     Serial.print(spi_slave_rx_buf[0], HEX);

//     Serial.print(", 0x");
//     if (spi_slave_rx_buf[1] < 0x10) Serial.print("0");
//     Serial.print(spi_slave_rx_buf[1], HEX);

//     Serial.print(", 0x");
//     if (spi_slave_rx_buf[1] < 0x10) Serial.print("0");
//     Serial.print(spi_slave_rx_buf[2], HEX);

//     Serial.print(", 0x");
//     if (spi_slave_rx_buf[1] < 0x10) Serial.print("0");
//     Serial.print(spi_slave_rx_buf[3], HEX);

//     Serial.print(", 0x");
//     if (spi_slave_rx_buf[1] < 0x10) Serial.print("0");
//     Serial.print(spi_slave_rx_buf[4], HEX);

//     Serial.print(", 0x");
//     if (spi_slave_rx_buf[1] < 0x10) Serial.print("0");
//     Serial.print(spi_slave_rx_buf[5], HEX);

//     Serial.print(", 0x");
//     if (spi_slave_rx_buf[1] < 0x10) Serial.print("0");
//     Serial.print(spi_slave_rx_buf[6], HEX);

//     Serial.print(", 0x");
//     if (spi_slave_rx_buf[1] < 0x10) Serial.print("0");
//     Serial.println(spi_slave_rx_buf[7], HEX);

//     // Use the first byte as command
//     uint8_t data = spi_slave_rx_buf[0];

//     if (data == 0xAB) {
//         Serial.println("Leg Controller has been Selected!");
//     } else {
//         Serial.println("No Connection");
//     }
// }




// #include <ESP32SPISlave.h>

// ESP32SPISlave slave;

// static constexpr uint32_t BUFFER_SIZE {8};
// uint8_t spi_slave_tx_buf[BUFFER_SIZE];
// uint8_t spi_slave_rx_buf[BUFFER_SIZE];

// #define LED 39
// void setup() {
//     Serial.begin(115200);
//     Serial.println("Serial Enabled");
//     delay(2000);
//     pinMode(LED, OUTPUT);
//     // begin() after setting
//     // HSPI = CS: 15, CLK: 14, MOSI: 13, MISO: 12 -> default
//     // VSPI = CS:  5, CLK: 18, MOSI: 23, MISO: 19
//     slave.setDataMode(SPI_MODE0);
//     slave.begin();

//     Serial.println("SPI Began");
//     // clear buffers
//     memset(spi_slave_tx_buf, 0, BUFFER_SIZE);
//     memset(spi_slave_rx_buf, 0, BUFFER_SIZE);
// }

// void loop() {
//     delay(1000);
//     char data;
//     const size_t received_bytes = slave.transfer(spi_slave_tx_buf, spi_slave_rx_buf, BUFFER_SIZE);
//         // show received data
//          Serial.print("Command Received: ");
//          Serial.print(spi_slave_rx_buf[0]); // Print first byte
//          Serial.print(", ");
//          Serial.println(spi_slave_rx_buf[1]); // Print second byte
//          data = spi_slave_rx_buf[0];

//          if(data == 0xFA)
//          {
//           Serial.println("Leg Controller has been Selected!");
//          }
//          else
//          {
//             Serial.println("No Connection");
//          }
// }

