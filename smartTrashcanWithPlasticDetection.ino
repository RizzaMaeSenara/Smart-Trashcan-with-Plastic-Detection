#include <AccelStepper.h>

// Stepper Motor Setup (Using ULN2003 Driver, 4-Wire Control)
#define motorInterfaceType 4  
AccelStepper stepper(motorInterfaceType, 8, 10, 9, 11);  // (IN1, IN3, IN2, IN4)

// Pin Definitions
const int trigLid = 6, echoLid = 7;   // Ultrasonic sensor for object detection near lid
const int trigBin = 2, echoBin = 3;   // Ultrasonic sensor for detecting bin fullness
const int irSensorPin = A0;            // IR Proximity Sensor for plastic detection (Analog input)
const int redLedPin = 13;              // Red LED (Indicates bin is full)
const int greenLedPin = 12;           // Green LED (Indicates lid is opening)

// Adjustable Stepper Motor Opening Angle (Set in Degrees)
const int openAngle = 135;  // Adjust this to change the lid opening angle

// Distance Thresholds 
const int fullThreshold = 10;  // If bin is filled up to 10cm from the lid, lid won't open
const int openThreshold = 15;  // Distance near lid to detect an object (cm)

// Plastic Detection Intensity Range (Only open lid if plastic intensity is in this range)
const int minPlasticIntensity = 600;  // Minimum intensity for plastic to be considered detected
const int maxPlasticIntensity = 800;  // Maximum intensity for plastic to be considered detected

// Function to Measure Distance Using an Ultrasonic Sensor
int getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);  
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);  // Send ultrasonic pulse
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);  // Measure pulse return time
  int distance = duration * 0.034 / 2;  // Convert time to distance in cm
  return distance;
}

// Optimization / error correction function for sensor noise
int optimizedDistance(int rawDistance, int sensorType) {
  // Apply correction only to lid sensor
  if (sensorType == 1) {
    // If the reading is close to the threshold, assume possible sensor error
    if (rawDistance > 15 && rawDistance < 30) {
      return rawDistance - 5;  // Apply correction for minor overestimation
    }
  }
  return rawDistance;  // Return as-is if not in correction range
}

// Handling errors with roots of an equation (threshold-based decision)
float calculateThresholdRoot(float distance, bool isPlastic) {
  float a = 0.5, b = 1.0, c = 1.5;  // Constants for the equation
  // Equation: a * d + b * v = c, where v = 1 if plastic is detected
  float root = (c - (a * distance + b * isPlastic)) / a; 
  return root;
}

// Solve linear system (for two sensors)
bool shouldOpenLid(int distanceLid, int distanceBin, bool isPlasticDetected) {
  // Using a simple linear system of equations to determine lid opening
  float a1 = 1.0, b1 = 1.5, c1 = 2.5;
  float a2 = 0.8, b2 = 1.0, c2 = 1.0;

  float x = (c1 - (b1 * isPlasticDetected)) / a1;
  float y = (c2 - (b2 * (distanceBin < fullThreshold))) / a2;

  // If both x and y are non-negative, open the lid
  return x >= 0 && y >= 0;
}

void setup() {
  Serial.begin(9600);

  // Set pin modes for ultrasonic sensors
  pinMode(trigLid, OUTPUT); pinMode(echoLid, INPUT);
  pinMode(trigBin, OUTPUT); pinMode(echoBin, INPUT);
  
  // Set pin modes for IR sensor & LEDs
  pinMode(irSensorPin, INPUT);  // Read analog values for IR sensor
  pinMode(redLedPin, OUTPUT);
  pinMode(greenLedPin, OUTPUT);
  
  // Configure the stepper motor
  stepper.setMaxSpeed(500.0);  // Maximum speed of the motor
  stepper.setAcceleration(100.0);  // Acceleration for smoother movement
}

void loop() {
  // Measure distances using ultrasonic sensors
  int lidDistance = getDistance(trigLid, echoLid);  // Distance to object near lid
  int binDistance = getDistance(trigBin, echoBin);  // Distance to detect if bin is full
  
  // Apply optimization to sensor readings
  lidDistance = optimizedDistance(lidDistance, 1);  // Optimize lid sensor
  binDistance = optimizedDistance(binDistance, 2);  // Optimize bin sensor

  bool isFull = (binDistance < fullThreshold);  // Check if bin is full
  
  // Read analog value from IR sensor for plastic detection
  int irSensorValue = analogRead(irSensorPin);  // Read analog value (0-1023)
  
  // Check if plastic is detected within the specified intensity range
  bool isPlasticDetected = (irSensorValue >= minPlasticIntensity && irSensorValue <= maxPlasticIntensity);
  
  // Debugging Output
  Serial.print("IR Sensor Value: "); Serial.println(irSensorValue);  // Show IR sensor value for debugging
  Serial.print("Lid Distance: "); Serial.print(lidDistance); Serial.println(" cm");
  Serial.print("Bin Distance: "); Serial.print(binDistance); Serial.println(" cm");

  // Calculate the root value for decision-making
  float rootValue = calculateThresholdRoot(lidDistance, isPlasticDetected);  // Get the root value
  
  // Debugging the root value
  Serial.print("Root Value: "); Serial.println(rootValue);

  // Use linear system to check if lid should open
  bool canOpenLid = shouldOpenLid(lidDistance, binDistance, isPlasticDetected);  // Evaluate using linear system
  
  // If the linear system returns true and root value is valid, proceed with opening the lid
  if (canOpenLid && rootValue >= 0 && !isFull) {
    // If conditions are met, open the lid
    Serial.println("Opening Lid...");
    digitalWrite(greenLedPin, HIGH);  // Turn on green LED
    digitalWrite(redLedPin, LOW);  // Turn off red LED

    // Move stepper motor to open position (Convert angle to steps)
    stepper.moveTo(openAngle * (2048 / 360));  
    while (stepper.distanceToGo() != 0) {
      stepper.run();
    }

    delay(5000);  // Keep lid open for 5 seconds

    // Close the lid
    Serial.println("Closing Lid...");
    stepper.moveTo(0);  // Move back to closed position
    while (stepper.distanceToGo() != 0) {
      stepper.run();
    }
    digitalWrite(greenLedPin, LOW);  // Turn off green LED
  } else {
    // If conditions are not met, show red LED if bin is full
    if (isFull) {
      Serial.println("Bin is Full! Lid won't open.");
      digitalWrite(redLedPin, HIGH);  // Turn on red LED
      digitalWrite(greenLedPin, LOW);  // Ensure green LED is off
    } else {
      digitalWrite(redLedPin, LOW);  // Turn off red LED
      digitalWrite(greenLedPin, LOW);  // Ensure green LED is off
    }
  }

  delay(500);  // Short delay for stability
}
