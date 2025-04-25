#include <ESP32SPISlave.h>

ESP32SPISlave slave;

static constexpr uint32_t BUFFER_SIZE {8};
uint8_t spi_slave_tx_buf[BUFFER_SIZE];
uint8_t spi_slave_rx_buf[BUFFER_SIZE];

#define LED 39
void setup() {
    Serial.begin(115200);
    Serial.println("Serial Enabled");
    delay(2000);
    pinMode(LED, OUTPUT);
    // begin() after setting
    // HSPI = CS: 15, CLK: 14, MOSI: 13, MISO: 12 -> default
    // VSPI = CS:  5, CLK: 18, MOSI: 23, MISO: 19
    slave.setDataMode(SPI_MODE0);
    slave.begin();

    Serial.println("SPI Began");
    // clear buffers
    memset(spi_slave_tx_buf, 0, BUFFER_SIZE);
    memset(spi_slave_rx_buf, 0, BUFFER_SIZE);
}

void loop() {
    delay(1000);
    char data;
    const size_t received_bytes = slave.transfer(spi_slave_tx_buf, spi_slave_rx_buf, BUFFER_SIZE);
        // show received data
         Serial.print("Command Received: ");
         Serial.println(spi_slave_rx_buf[0]);
         data = spi_slave_rx_buf[0];
}

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

//     slave.setDataMode(SPI_MODE0);
//     slave.begin();
//     Serial.println("SPI Began");

//     memset(spi_slave_tx_buf, 0, BUFFER_SIZE);
//     memset(spi_slave_rx_buf, 0, BUFFER_SIZE);
// }

// void loop() {
//     const size_t received_bytes = slave.transfer(spi_slave_tx_buf, spi_slave_rx_buf, BUFFER_SIZE);
    
//     if (received_bytes > 0) {
//         Serial.print("Received: ");
//         for (size_t i = 0; i < received_bytes; ++i) {
//             char c = spi_slave_rx_buf[i];
//             Serial.printf("%c", isPrintable(c) ? c : '.');  // print char if printable, else dot
//         }
//         Serial.println();  // newline after each transfer
//     }

//     delay(10); // Small delay to avoid flooding Serial
// }

