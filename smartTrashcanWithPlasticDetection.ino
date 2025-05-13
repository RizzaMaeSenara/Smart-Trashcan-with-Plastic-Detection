#include <AccelStepper.h>

// Motor control using 4 pins (IN1, IN3, IN2, IN4)
#define motorInterfaceType 4  
AccelStepper stepper(motorInterfaceType, 8, 10, 9, 11);

// Sensor pins
const int trigLid = 6, echoLid = 7;     // Trig and Echo pins for lid distance sensor
const int trigBin = 2, echoBin = 3;     // Trig and Echo pins for bin distance sensor
const int irSensorPin = A0;              // IR sensor pin for detecting plastic material

// LED indicators
const int redLedPin = 13;  // Red LED for bin full indication
const int greenLedPin = 12; // Green LED for lid open indication

// Configuration constants
const int openAngle = 135;        // Angle to open the lid (in degrees)
const int fullThreshold = 10;     // Threshold for bin fullness (in cm)
const int openThreshold = 15;     // Threshold to open the lid (distance in cm)
const int minPlasticIntensity = 600; // Minimum IR sensor value to detect plastic
const int maxPlasticIntensity = 800; // Maximum IR sensor value for plastic detection

// Function to get distance using ultrasonic sensor
int getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW); delayMicroseconds(2);  // Ensure trigPin is LOW before sending pulse
  digitalWrite(trigPin, HIGH); delayMicroseconds(10); // Send pulse to trigger the sensor
  digitalWrite(trigPin, LOW); // End pulse
  long duration = pulseIn(echoPin, HIGH); // Measure duration of the echo signal
  return duration * 0.034 / 2; // Calculate and return distance (in cm)
}

// Function to apply correction or offset to sensor readings
int optimizedDistance(int rawDistance, int sensorType) {
  if (sensorType == 1 && rawDistance > 15 && rawDistance < 30) return rawDistance - 5; // Adjust for lid distance
  return rawDistance; // Return unmodified for other sensors
}

// Function to calculate a root value based on distance and plastic detection
float calculateThresholdRoot(float distance, bool isPlastic) {
  float a = 0.5, b = 1.0, c = 1.5; // Constants for the formula
  return (c - (a * distance + b * isPlastic)) / a; // Calculate root value
}

// Decision-making logic for whether to open the lid or not
bool shouldOpenLid(int distanceLid, int distanceBin, bool isPlasticDetected) {
  float a1 = 1.0, b1 = 1.5, c1 = 2.5; // Constants for lid calculation
  float a2 = 0.8, b2 = 1.0, c2 = 1.0; // Constants for bin calculation
  float x = (c1 - (b1 * isPlasticDetected)) / a1; // Calculate condition for opening lid based on plastic detection
  float y = (c2 - (b2 * (distanceBin < fullThreshold))) / a2; // Calculate condition based on bin fullness
  
  // Lid can only open if plastic is detected and bin is not full
  return isPlasticDetected && x >= 0 && y >= 0;
}

void setup() {
  Serial.begin(9600); // Initialize serial communication for debugging
  pinMode(trigLid, OUTPUT); pinMode(echoLid, INPUT); // Set lid sensor pins
  pinMode(trigBin, OUTPUT); pinMode(echoBin, INPUT); // Set bin sensor pins
  pinMode(irSensorPin, INPUT); // Set IR sensor pin
  pinMode(redLedPin, OUTPUT); pinMode(greenLedPin, OUTPUT); // Set LED pins

  stepper.setMaxSpeed(500.0); // Set the maximum speed of the motor
  stepper.setAcceleration(100.0); // Set the acceleration of the motor
}

void loop() {
  // Measure distances from the lid and the bin
  int lidDistance = optimizedDistance(getDistance(trigLid, echoLid), 1); // Get optimized lid distance
  int binDistance = optimizedDistance(getDistance(trigBin, echoBin), 2); // Get optimized bin distance
  bool isFull = (binDistance < fullThreshold); // Check if bin is full
  
  // Read the IR sensor to detect plastic
  int irSensorValue = analogRead(irSensorPin); // Read value from IR sensor
  bool isPlasticDetected = (irSensorValue >= minPlasticIntensity && irSensorValue <= maxPlasticIntensity); // Check if plastic is detected
  
  // Calculate root value based on current conditions
  float rootValue = calculateThresholdRoot(lidDistance, isPlasticDetected);
  
  // Check if conditions allow opening the lid
  bool canOpenLid = shouldOpenLid(lidDistance, binDistance, isPlasticDetected);

  // Store lid status
  String lidStatus = "";
  String binStatus = isFull ? "FULL" : "OK"; // Set bin status based on fullness

  if (canOpenLid && rootValue >= 0 && !isFull) {
    lidStatus = "OPENING"; // Prepare to open the lid
    // Print sensor data and status to the Serial Monitor
    Serial.print("IR: "); Serial.print(irSensorValue);
    Serial.print(" | LidDistance: "); Serial.print(lidDistance); Serial.print(" cm");
    Serial.print(" | BinDistance: "); Serial.print(binDistance); Serial.print(" cm");
    Serial.print(" | Lid: "); Serial.print(lidStatus);
    Serial.print(" | Bin: "); Serial.println(binStatus);

    digitalWrite(greenLedPin, HIGH); // Turn on green LED (lid open)
    digitalWrite(redLedPin, LOW); // Turn off red LED (bin not full)

    // Move motor to open the lid
    stepper.moveTo(openAngle * (2048 / 360)); // Convert angle to motor steps
    while (stepper.distanceToGo() != 0) stepper.run(); // Keep motor running until lid is fully open

    delay(5000);  // Keep the lid open for 5 seconds

    // Close the lid after delay
    lidStatus = "CLOSING"; // Prepare to close the lid
    Serial.print("IR: "); Serial.print(irSensorValue);
    Serial.print(" | LidDistance: "); Serial.print(lidDistance); Serial.print(" cm");
    Serial.print(" | BinDistance: "); Serial.print(binDistance); Serial.print(" cm");
    Serial.print(" | Lid: "); Serial.print(lidStatus);
    Serial.print(" | Bin: "); Serial.println(binStatus);

    stepper.moveTo(0); // Move motor to close the lid
    while (stepper.distanceToGo() != 0) stepper.run(); // Keep motor running until lid is fully closed

    digitalWrite(greenLedPin, LOW); // Turn off green LED (lid closed)
  } else {
    // If lid cannot open, set status accordingly
    lidStatus = isFull ? "BLOCKED" : "CLOSED"; // Lid is blocked if bin is full, otherwise closed
    Serial.print("IR: "); Serial.print(irSensorValue);
    Serial.print(" | LidDistance: "); Serial.print(lidDistance); Serial.print(" cm");
    Serial.print(" | BinDistance: "); Serial.print(binDistance); Serial.print(" cm");
    Serial.print(" | Lid: "); Serial.print(lidStatus);
    Serial.print(" | Bin: "); Serial.println(binStatus);

    digitalWrite(redLedPin, isFull ? HIGH : LOW); // Turn on red LED if bin is full
    digitalWrite(greenLedPin, LOW); // Keep green LED off
  }

  delay(500); // Delay to stabilize readings and avoid overloading the system
}
