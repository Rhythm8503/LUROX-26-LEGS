#include <SPI.h>
#include <ESP32SPISlave.h>

#define SS_PIN 10  // Slave Select Pin
#define CLK_PIN 12 // Clock Pin (SCK)
#define MOSI_PIN 11 // Master Out Slave In Pin (MOSI)
#define MISO_PIN 13 // Master In Slave Out Pin (MISO)

ESP32SPISlave spiSlave;

void setup() {
  Serial.begin(115200);

  // Initialize SPI Slave with specific pin configuration
  if (!spiSlave.begin(HSPI, CLK_PIN, MISO_PIN, MOSI_PIN, SS_PIN)) {
    Serial.println("SPI Slave Initialization Failed!");
    while (1);  // Stop execution if initialization fails
  }

  // Set up SPI configuration
  spiSlave.setDataMode(SPI_MODE0);  // Set SPI Mode

  pinMode(SS_PIN, INPUT_PULLUP);  // Slave select pin as input
  Serial.println("Slave is ready...");
}

void loop() {
  // Prepare buffers for transaction
  uint8_t txBuffer[1] = {0};  // Transmit buffer
  uint8_t rxBuffer[1] = {0};  // Receive buffer

  // Check if there are queued transactions
  if (spiSlave.numTransactionsInFlight() > 0) {
    // Wait for the transaction to complete
    std::vector<size_t> results = spiSlave.wait();

    // Process received data (example: increment and send back)
    if (spiSlave.numBytesReceived() > 0) {
      // Get the received data
      byte data = rxBuffer[0];

      // Print the received data
      Serial.print("Received: ");
      Serial.println(data);

      // Modify the received data (e.g., increment by 1)
      txBuffer[0] = data + 1;  // Increment received value by 1
    }

    // Send the response to master (transmit modified data)
    spiSlave.transfer(txBuffer, rxBuffer, sizeof(txBuffer));  // Transfer response
  }
}
