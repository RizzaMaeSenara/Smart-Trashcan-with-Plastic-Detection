#include <Servo.h>

Servo lidServo;

// Sensor pins
const int trigLid = 6, echoLid = 7; //nearby object
const int trigBin = 2, echoBin = 3; //bin fullness
const int irSensorPin = A0;

// LEDs and buzzer
const int redLedPin = 13;
const int greenLedPin = 12;
const int buzzerPin = 10;
const int buttonPin = 11; // Pushbutton to silence buzzer

// Configuration constants
const int openAngle = 90;
const int closeAngle = 0;
const int fullThreshold = 10;   // cm
const int openThreshold = 15;   // cm
const int minPlasticIntensity = 600;
const int maxPlasticIntensity = 800;

// Moving Average Filter settings
const int numSamples = 10;
int irSamples[numSamples];
int irIndex = 0;

// Timing
unsigned long previousBlinkTime = 0;
unsigned long plasticLastDetectedTime = 0;
unsigned long lastPrintTime = 0;
const unsigned long blinkInterval = 200;
const unsigned long closeDelay = 1000;  // 1 second
const unsigned long printInterval = 500;

bool greenLedState = LOW;
bool lidIsOpen = false;
bool buzzerSilenced = false;

int getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW); delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH);
  return duration * 0.034 / 2;
}

int optimizedDistance(int rawDistance, int sensorType) {
  if (sensorType == 1 && rawDistance > 15 && rawDistance < 30) return rawDistance - 5;
  return rawDistance;
}

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
  pinMode(buzzerPin, OUTPUT); digitalWrite(buzzerPin, LOW);
  pinMode(buttonPin, INPUT_PULLUP);

  lidServo.attach(9);
  lidServo.write(closeAngle);

  for (int i = 0; i < numSamples; i++) {
    irSamples[i] = analogRead(irSensorPin);
  }
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

  // --- Button Logic ---
  int buttonState = digitalRead(buttonPin);
  if (buttonState == LOW) {
    buzzerSilenced = true;
    digitalWrite(buzzerPin, LOW);
  }

  // --- Lid Control Logic ---
  if (isPlasticDetected && !isFull) {
    plasticLastDetectedTime = currentMillis;

    if (!lidIsOpen) {
      lidServo.write(openAngle);
      lidIsOpen = true;
      lidStatus = "PLASTIC DETECTED, OPENING";
    } else {
      lidStatus = "PLASTIC STILL DETECTED, KEEP OPEN";
    }

    if (currentMillis - previousBlinkTime >= blinkInterval) {
      previousBlinkTime = currentMillis;
      greenLedState = !greenLedState;
      digitalWrite(greenLedPin, greenLedState);
    }

    digitalWrite(redLedPin, LOW);
  } else {
    if (lidIsOpen) {
      if (currentMillis - plasticLastDetectedTime >= closeDelay) {
        if (!isPlasticDetected) {
          lidServo.write(closeAngle);
          lidIsOpen = false;
          digitalWrite(greenLedPin, LOW);
          lidStatus = "NO PLASTIC, CLOSING";
        } else {
          lidStatus = "PLASTIC DETECTED, OPEN";
        }
      } else {
        lidStatus = "WAITING TO CLOSE";
      }
    } else {
      lidStatus = "CLOSED";
    }

    if (isObjectDetected && !isPlasticDetected) {
      lidStatus = "OBJ DETECTED (NOT PLASTIC)";
    }
  }

  // --- Bin Full Logic ---
  if (isFull) {
    digitalWrite(redLedPin, HIGH);
    if (!buzzerSilenced) {
      digitalWrite(buzzerPin, HIGH);
    }
  } else {
    digitalWrite(redLedPin, LOW);
    digitalWrite(buzzerPin, LOW);
    buzzerSilenced = false; // Reset silence when bin is cleared
  }

  // --- Serial Monitoring ---
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
