#include <Stepper.h>
#include <Servo.h>

const int stepsPerRevolution = 2048;
Stepper stepper(stepsPerRevolution, 8, 10, 9, 11);

Servo inputCup;
const int inputCupPin = 6;

int currentAngle = 0;

const int irSensorPin = A0;
const int trigPin = 4;
const int echoPin = 5;

unsigned long lastIRCheck = 0;
const unsigned long irCheckInterval = 500;

unsigned long lastDistancePrint = 0;
const unsigned long distancePrintInterval = 500;

bool objectDetected = false;

const int filterWindowSize = 5;
float distanceBuffer[filterWindowSize] = {0};
int bufferIndex = 0;
bool bufferFilled = false;

const int tolerance = 5;  // Added tolerance ±5 for IR checks

void disableStepper() {
  digitalWrite(8, LOW);
  digitalWrite(10, LOW);
  digitalWrite(9, LOW);
  digitalWrite(11, LOW);
  delay(500);
  Serial.println("Stepper disabled");
}

void setup() {
  Serial.begin(9600);
  stepper.setSpeed(10);
  inputCup.attach(inputCupPin);
  inputCup.write(90);

  pinMode(8, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(11, OUTPUT);

  pinMode(irSensorPin, INPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  Serial.println("Setup complete");
}

void rotateTo(int targetAngle) {
  Serial.print("Rotating to: ");
  Serial.println(targetAngle);
  int stepTarget = map(targetAngle, 0, 360, 0, stepsPerRevolution);
  int stepCurrent = map(currentAngle, 0, 360, 0, stepsPerRevolution);
  int stepsToMove = stepTarget - stepCurrent;

  stepper.step(stepsToMove);
  currentAngle = targetAngle;
  disableStepper();
}

float measureDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  float distance = duration * 0.034 / 2;
  return distance;
}

float updateDistanceBuffer(float newReading) {
  distanceBuffer[bufferIndex] = newReading;
  bufferIndex = (bufferIndex + 1) % filterWindowSize;

  if (bufferIndex == 0) bufferFilled = true;

  int count = bufferFilled ? filterWindowSize : bufferIndex;
  float sum = 0;
  for (int i = 0; i < count; i++) {
    sum += distanceBuffer[i];
  }
  return sum / count;
}

// This function does a 1-second average measurement to get a stable distance reading
float measureAverageDistance() {
  const unsigned long measurementDuration = 1000;
  unsigned long startTime = millis();
  float sum = 0;
  int count = 0;

  while (millis() - startTime < measurementDuration) {
    float distance = measureDistance();
    sum += distance;
    count++;
    delay(50);
  }

  float average = (count > 0) ? (sum / count) : 0;
  Serial.print("Average Distance: ");
  Serial.print(average);
  Serial.println(" cm");
  return average;
}

// Bin fullness check uses ONLY the 1-second average distance here
void processSorting(String type) {
  int targetAngle = (type == "plastic") ? 0 : 180;
  rotateTo(targetAngle);

  delay(1000); // Wait after rotation

  float averageDistance = measureAverageDistance();

  // Fullness threshold check using 1-second average distance
  if (averageDistance >= 0 && averageDistance <= 22) {
    Serial.println("Bin is full. No action taken.");
    return;
  }

  Serial.println("Bin is not full. Activating inputCup servo.");
  inputCup.write(0); // Open inputCup

  // Wait until object is no longer detected (by IR sensor) using tolerance
  while (true) {
    int irVal = analogRead(irSensorPin);
    if (irVal < (300 - tolerance) || irVal > (740 + tolerance)) {
      break;
    }
    delay(200);
  }

  delay(2000); // Wait a bit more before resetting
  inputCup.write(90); // Return to original position
}

void handleIRSensor() {
  int irValue = analogRead(irSensorPin);

  if (!objectDetected) {
    if (irValue >= (300 - tolerance) && irValue <= (740 + tolerance)) {
      Serial.println("IR detected PLASTIC");
      objectDetected = true;
      processSorting("plastic");
    } else if (irValue >= (1 - tolerance) && irValue < (299 + tolerance)) {
      Serial.println("IR detected NON-PLASTIC");
      objectDetected = true;
      processSorting("nonplastic");
    }
  } else {
    if (irValue < 1 || irValue > 741 ) {
      objectDetected = false;
      inputCup.write(90);
    }
  }
}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readString();
    command.trim();

    if (command == "plastic") {
      Serial.println("Manual command: PLASTIC");
      processSorting("plastic");
    } else if (command == "nonplastic") {
      Serial.println("Manual command: NON-PLASTIC");
      processSorting("nonplastic");
    } else if (command == "open") {
      Serial.println("Manual command: OPEN INPUTCUP");
      inputCup.write(0);
    } else if (command == "close") {
      Serial.println("Manual command: CLOSE INPUTCUP");
      inputCup.write(90);
    } else {
      Serial.print("Unknown command: ");
      Serial.println(command);
    }
  }

  if (millis() - lastIRCheck >= irCheckInterval) {
    lastIRCheck = millis();
    handleIRSensor();
  }

  // Display raw and filtered distances continuously for monitoring only
  if (millis() - lastDistancePrint >= distancePrintInterval) {
    lastDistancePrint = millis();

    float rawDistance = measureDistance();
    float filteredDistance = updateDistanceBuffer(rawDistance);
    int currentIR = analogRead(irSensorPin);

    Serial.print("Raw: ");
    Serial.print(rawDistance, 1);
    Serial.print(" cm | Filtered: ");
    Serial.print(filteredDistance, 1);
    Serial.print(" cm | IR: ");
    Serial.println(currentIR);
  }
}
