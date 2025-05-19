#include <Servo.h>

Servo lidServo;

// Sensor pins
const int trigLid = 6, echoLid = 7;
const int trigBin = 2, echoBin = 3;
const int irSensorPin = A0;

// LEDs
const int redLedPin = 13;
const int greenLedPin = 12;

// Configuration constants
const int openAngle = 90;
const int closeAngle = 0;
const int fullThreshold = 10;  // cm
const int openThreshold = 15;  // cm
const int minPlasticIntensity = 600;
const int maxPlasticIntensity = 800;

// Moving Average Filter settings
const int numSamples = 5;
int irSamples[numSamples];
int irIndex = 0;

// Timing
unsigned long previousBlinkTime = 0;
unsigned long previousCloseCheck = 0;
unsigned long lastPrintTime = 0;
const unsigned long blinkInterval = 200;
const unsigned long closeDelay = 1000;
const unsigned long printInterval = 1000;

bool greenLedState = LOW;
bool lidIsOpen = false;

// Get distance from ultrasonic sensor
int getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW); delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH);
  return duration * 0.034 / 2;
}

// Apply offset to compensate for sensor error
int optimizedDistance(int rawDistance, int sensorType) {
  if (sensorType == 1 && rawDistance > 15 && rawDistance < 30) return rawDistance - 5;
  return rawDistance;
}

// Moving average filter for IR sensor
int getFilteredIR() {
  irSamples[irIndex] = analogRead(irSensorPin);
  irIndex = (irIndex + 1) % numSamples;

  long sum = 0;
  for (int i = 0; i < numSamples; i++) {
    sum += irSamples[i];
  }

  return sum / numSamples;
}

void setup() {
  Serial.begin(9600);
  pinMode(trigLid, OUTPUT); pinMode(echoLid, INPUT);
  pinMode(trigBin, OUTPUT); pinMode(echoBin, INPUT);
  pinMode(irSensorPin, INPUT);
  pinMode(redLedPin, OUTPUT); pinMode(greenLedPin, OUTPUT);

  lidServo.attach(9);
  lidServo.write(closeAngle);

  for (int i = 0; i < numSamples; i++) irSamples[i] = analogRead(irSensorPin); // initialize buffer
}

void loop() {
  int lidDistance = optimizedDistance(getDistance(trigLid, echoLid), 1);
  int binDistance = optimizedDistance(getDistance(trigBin, echoBin), 2);
  bool isFull = (binDistance < fullThreshold);

  int filteredIR = getFilteredIR();
  bool isPlasticDetected = (filteredIR >= minPlasticIntensity && filteredIR <= maxPlasticIntensity);
  bool isObjectDetected = (filteredIR > 50);

  String lidStatus = "";
  String binStatus = isFull ? "FULL" : "OK";

  unsigned long currentMillis = millis();

  // Open lid if plastic is detected
  if (isPlasticDetected && !isFull) {
    lidServo.write(openAngle);
    lidIsOpen = true;
    previousCloseCheck = currentMillis;

    // Blink green LED
    if (currentMillis - previousBlinkTime >= blinkInterval) {
      previousBlinkTime = currentMillis;
      greenLedState = !greenLedState;
      digitalWrite(greenLedPin, greenLedState);
    }

    digitalWrite(redLedPin, LOW);
    lidStatus = "PLASTIC DETECTED, OPEN";

  } else {
    if (lidIsOpen) {
      if ((currentMillis - previousCloseCheck) >= closeDelay) {
        lidServo.write(closeAngle);
        lidIsOpen = false;
        digitalWrite(greenLedPin, LOW);
        lidStatus = "CLOSING";
      } else {
        lidStatus = "OPEN";
      }
    } else {
      lidStatus = "CLOSED";
    }

    if (isFull) {
      digitalWrite(redLedPin, HIGH);
      digitalWrite(greenLedPin, LOW);
    } else {
      digitalWrite(redLedPin, LOW);
      digitalWrite(greenLedPin, LOW);
    }

    if (isObjectDetected && !isPlasticDetected) {
      lidStatus = "OBJ DETECTED (NOT PLASTIC)";
    }
  }

  if (currentMillis - lastPrintTime >= printInterval) {
    lastPrintTime = currentMillis;
    Serial.print("IR: "); Serial.print(filteredIR);
    Serial.print(" | LidDist: "); Serial.print(lidDistance); Serial.print(" cm");
    Serial.print(" | BinDist: "); Serial.print(binDistance); Serial.print(" cm");
    Serial.print(" | Lid: "); Serial.print(lidStatus);
    Serial.print(" | Bin: "); Serial.println(binStatus);
  }

  delay(50);
}
