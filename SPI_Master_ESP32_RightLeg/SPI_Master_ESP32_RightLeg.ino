#include <SPI.h>

#define SCK_PIN 12
#define MISO_PIN 13
#define MOSI_PIN 11
#define SS_PIN 10

// SPI Settings
SPIClass hspi(HSPI); // SPI2 = HSPI
SPISettings spiSettings(1000000, MSBFIRST, SPI_MODE0); // 1 MHz, MSB first, Mode 0

void setup() {
    Serial.begin(115200);

    // Initialize HSPI with default pins
    hspi.begin(SCK_PIN, MISO_PIN, MOSI_PIN, SS_PIN);
    pinMode(SS_PIN, OUTPUT); // SS as output
    digitalWrite(SS_PIN, HIGH); // Deselect slave
}

void loop() {
    digitalWrite(SS_PIN, LOW); // Select slave

    // SPI transaction
    hspi.beginTransaction(spiSettings);
    byte dataToSend = 0xAA; // Example data
    byte receivedData = hspi.transfer(dataToSend); // Send and receive
    hspi.endTransaction(); // End SPI transaction

    digitalWrite(SS_PIN, HIGH); // Deselect slave

    // Print data
    Serial.print("Sent: 0x");
    Serial.print(dataToSend, HEX);
    Serial.print(", Received: 0x");
    Serial.println(receivedData, HEX);

    delay(1000);
}
