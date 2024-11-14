#include <SPI.h>

const int SS_PIN = 10;

void setup() {
  Serial.begin(115200);
  SPI.begin(12, 13, 11, 10);                    // Initialize SPI as Master
  pinMode(SS_PIN, OUTPUT);        // Set SS as output
  digitalWrite(SS_PIN, HIGH);     // Set SS HIGH (inactive)

  delay(1000);
}

void loop() {
  digitalWrite(SS_PIN, LOW);      // Enable Slave
  SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));

  const char *message = "Hello from Master!";
  for (int i = 0; i < strlen(message); i++) {
    SPI.transfer(message[i]);     // Send message to Slave byte by byte
    delay(50);
  }
  
  SPI.endTransaction();
  digitalWrite(SS_PIN, HIGH);     // Disable Slave
  
  delay(2000);                    // Wait before sending again
}
