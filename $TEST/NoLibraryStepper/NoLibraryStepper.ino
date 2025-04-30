// #define dirPin 1
// #define stepPin 2

// const int stepsPerRev = 200; // Adjust if using microstepping
// //const int microstepping = 16; // e.g., 1/16 microstepping
// //const int totalSteps = stepsPerRev * microstepping;
// const int stepDelayMicros = 1000; // delay between steps in microseconds (controls speed)

// void setup() {
//   pinMode(dirPin, OUTPUT);
//   pinMode(stepPin, OUTPUT);

//   digitalWrite(dirPin, HIGH); // Set direction

//   // for (int i = 0; i < stepsPerRev; i++) {
//   //   digitalWrite(stepPin, HIGH);
//   //   delayMicroseconds(stepDelayMicros);
//   //   digitalWrite(stepPin, LOW);
//   //   delayMicroseconds(stepDelayMicros);
//   // }
// }

// void loop() {

//     digitalWrite(stepPin, HIGH);
//     delayMicroseconds(stepDelayMicros);
//     digitalWrite(stepPin, LOW);
//     delayMicroseconds(stepDelayMicros);
//   // nothing
// }


// #define dirPin 1
// #define stepPin 2

// const int stepsPerRev = 600; // Change this if using microstepping
// const int stepDelayMicros = 1000; // delay between steps in microseconds (controls speed)

// void setup() {
//   pinMode(dirPin, OUTPUT);
//   pinMode(stepPin, OUTPUT);
// }

// void loop() {
//   // Rotate clockwise
//   digitalWrite(dirPin, HIGH);
//   for (int i = 0; i < stepsPerRev; i++) {
//     digitalWrite(stepPin, HIGH);
//     delayMicroseconds(stepDelayMicros);
//     digitalWrite(stepPin, LOW);
//     delayMicroseconds(stepDelayMicros);
//   }

//   delay(1000); // 1 second delay

//   // Rotate counter-clockwise
//   digitalWrite(dirPin, LOW);
//   for (int i = 0; i < stepsPerRev; i++) {
//     digitalWrite(stepPin, HIGH);
//     delayMicroseconds(stepDelayMicros);
//     digitalWrite(stepPin, LOW);
//     delayMicroseconds(stepDelayMicros);
//   }

//   delay(1000); // 1 second delay
// }

#define dirPin 1
#define stepPin 2

const int stepDelayMicros = 4000; // Step speed
long currentPosition = 0;         // Track current position

void setup() {
  pinMode(dirPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  Serial.begin(115200);
  Serial.println("Enter target position in steps (e.g., 200, -400, etc):");
  // For neck = +110 steps for rotation to the left, and -110 steps for rotation to the right
}

void loop() {
  // Check for user input
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.length() > 0) {
      long targetPosition = input.toInt();
      long stepsToMove = targetPosition - currentPosition;

      if (stepsToMove == 0) {
        Serial.println("Already at target position.");
        return;
      }

      // Set direction
      if (stepsToMove > 0) {
        digitalWrite(dirPin, HIGH);  // Clockwise
      } else {
        digitalWrite(dirPin, LOW);   // Counter-clockwise
        stepsToMove = -stepsToMove;  // Make positive
      }

      // Step the motor
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
  }
}



