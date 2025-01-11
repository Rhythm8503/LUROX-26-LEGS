#include <SPI.h>

#define SS_PIN 10  // Slave Select Pin
#define CLK_PIN 12 // Clock Pin (SCK)
#define MOSI_PIN 11 // Master Out Slave In Pin (MOSI)
#define MISO_PIN 13 // Master In Slave Out Pin (MISO)

void setup() {
  Serial.begin(115200);

  // Initialize SPI
  SPI.begin(CLK_PIN, MISO_PIN, MOSI_PIN, SS_PIN);
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));

  pinMode(SS_PIN, OUTPUT);
  digitalWrite(SS_PIN, HIGH); // Disable the slave initially
}

void loop() {
  // Select the slave device
  digitalWrite(SS_PIN, LOW);

  // Send data to the slave
  byte data = 37;  // Example data to send
  byte receivedData = SPI.transfer(data);

  Serial.print("[MASTER] Sent: ");
  Serial.print(data);
  Serial.print(" | Received: ");
  Serial.println(receivedData);

  // Deselect the slave device
  digitalWrite(SS_PIN, HIGH);

  delay(1000);  // Send data every 1 second
}
