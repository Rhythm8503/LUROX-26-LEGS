#include <SPI.h>

volatile bool received = false;
char message[100];                // Buffer for incoming data
int msgIndex = 0;                 // Updated variable name
int numLine = 0;

void IRAM_ATTR onSPIReceive();    // Forward declaration of the interrupt function

void setup() {
  Serial.begin(115200);
  pinMode(SS, INPUT);             // SS pin as input
  SPI.begin(12, 13, 11, 10);                    // Initialize SPI as Slave

  // Set up an interrupt on the SS pin to detect when communication starts
  attachInterrupt(digitalPinToInterrupt(SS), onSPIReceive, FALLING);
}

void loop() {
  if (received) {
    numLine++;
    message[msgIndex] = '\0';       // Null-terminate the string
    Serial.printf("%d%s", numLine, "Received: ");
    Serial.println(message);        // Print received message

    msgIndex = 0;                   // Reset index for next message
    received = false;               // Reset flag
  }
}

// Interrupt routine
void IRAM_ATTR onSPIReceive() {
  if (msgIndex < sizeof(message) - 1) {  // Prevent buffer overflow
    message[msgIndex++] = SPI.transfer(0);  // Receive data
    received = true;                     // Set flag when data is received
  }
}
