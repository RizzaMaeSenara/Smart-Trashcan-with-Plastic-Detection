#include <Stepper.h>
#include <Servo.h>

const int stepsPerRevolution = 2048; // Steps for full 360° rotation
Stepper stepper(stepsPerRevolution, 8, 10, 9, 11); // IN1, IN3, IN2, IN4

Servo inputCup; // Create servo object for input cup
const int inputCupPin = 6; // Define pin connected to servo

int currentAngle = 0; // Track current angle of stepper motor

const int irSensorPin = A0;
const int trigPinPlastic = 2;
const int echoPinPlastic = 3;
const int trigPinNonPlastic = 4;
const int echoPinNonPlastic = 5;

const int greenLEDPin = 12;
const int redLEDPinPlastic = 13;
const int redLEDPinNonPlastic = 7;

const int buzzerPin = A1;  // Added buzzer pin

unsigned long lastIRCheck = 0; // Timestamp for last IR check
const unsigned long irCheckInterval = 500; // Interval to check IR (ms)

unsigned long lastDistancePrint = 0; // Timestamp for last distance print
const unsigned long distancePrintInterval = 500; // Interval to print distance (ms)

bool objectDetected = false; // Track if an object has already been detected

const int filterWindowSize = 5; // Size of window for Finite impulse response (FIR Filter)
float distanceBufferPlastic[filterWindowSize] = {0}; 
float distanceBufferNonPlastic[filterWindowSize] = {0};
int bufferIndexPlastic = 0;
int bufferIndexNonPlastic = 0;
bool bufferFilledPlastic = false;
bool bufferFilledNonPlastic = false;

const int tolerance = 5; // ±5 tolerance for IR value threshold
String materialStatus = "Empty";

bool isPlasticBinFull = false;
bool isNonPlasticBinFull = false;

// --- Disable Stepper ---
void disableStepper() {
  digitalWrite(8, LOW); digitalWrite(10, LOW);
  digitalWrite(9, LOW); digitalWrite(11, LOW);
  delay(500);
  Serial.println("Stepper disabled");
}

// --- Setup ---
void setup() {
  Serial.begin(9600);
  stepper.setSpeed(10);
  inputCup.attach(inputCupPin);
  inputCup.write(90);

  pinMode(8, OUTPUT); pinMode(10, OUTPUT);
  pinMode(9, OUTPUT); pinMode(11, OUTPUT);
  pinMode(irSensorPin, INPUT);

  pinMode(trigPinPlastic, OUTPUT);
  pinMode(echoPinPlastic, INPUT);
  pinMode(trigPinNonPlastic, OUTPUT);
  pinMode(echoPinNonPlastic, INPUT);

  pinMode(greenLEDPin, OUTPUT);
  pinMode(redLEDPinPlastic, OUTPUT);
  pinMode(redLEDPinNonPlastic, OUTPUT);

  pinMode(buzzerPin, OUTPUT); // Initialize buzzer pin

  digitalWrite(greenLEDPin, HIGH);
  digitalWrite(redLEDPinPlastic, LOW);
  digitalWrite(redLEDPinNonPlastic, LOW);
  digitalWrite(buzzerPin, LOW); // Ensure buzzer off initially

  Serial.println("Setup complete");
}

// --- Stepper Rotation ---
void rotateTo(int targetAngle) {
  Serial.print("Rotating to: "); Serial.println(targetAngle);
  int stepTarget = map(targetAngle, 0, 360, 0, stepsPerRevolution);
  int stepCurrent = map(currentAngle, 0, 360, 0, stepsPerRevolution);
  int stepsToMove = stepTarget - stepCurrent;

  stepper.step(stepsToMove);
  currentAngle = targetAngle;
  disableStepper();
}

// --- Distance Measurement ---
float measureDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW); delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH);
  return duration * 0.034 / 2;
}

// --- Apply filter to smooth out sensor data, (Finite impulse response, FIR Filter) ---
float updateDistanceBuffer(float newReading, float* buffer, int& index, bool& filled) {
  buffer[index] = newReading;
  index = (index + 1) % filterWindowSize;
  if (index == 0) filled = true;
  int count = filled ? filterWindowSize : index;
  float sum = 0;
  for (int i = 0; i < count; i++) sum += buffer[i];
  return sum / count;
}

// --- Get average distance over 1 second for stable value, Time-based Arithmetic Mean ---
float measureAverageDistanceForBin(int trigPin, int echoPin) {
  const unsigned long duration = 1000;
  unsigned long start = millis();
  float sum = 0;
  int count = 0;

  while (millis() - start < duration) {
    float d = measureDistance(trigPin, echoPin);
    sum += d; count++;
    delay(50);
  }

  return (count > 0) ? (sum / count) : 0;
}

// --- Blink LED with Buzzer ---
void blinkLED(int pin, int times = 3, int delayMs = 200) {
  bool isRedLED = (pin == redLEDPinPlastic) || (pin == redLEDPinNonPlastic);

  for (int i = 0; i < times; i++) {
    digitalWrite(pin, LOW);
    if (isRedLED) digitalWrite(buzzerPin, HIGH); // Turn buzzer ON during red LED ON
    delay(delayMs);
    digitalWrite(pin, HIGH);
    if (isRedLED) digitalWrite(buzzerPin, LOW);  // Turn buzzer OFF during red LED OFF
    delay(delayMs);
  }
}

// --- Sorting Logic ---
void processSorting(String type) {
  int targetAngle = (type == "plastic") ? 0 : 180;
  rotateTo(targetAngle);
  delay(1000);

  float averageDistance;
  if (type == "plastic") {
    averageDistance = measureAverageDistanceForBin(trigPinPlastic, echoPinPlastic);
    Serial.print("Plastic Bin Avg Distance: ");
  } else {
    averageDistance = measureAverageDistanceForBin(trigPinNonPlastic, echoPinNonPlastic);
    Serial.print("Non-Plastic Bin Avg Distance: ");
  }

  Serial.print(averageDistance); Serial.println(" cm");

  if (averageDistance >= 0 && averageDistance <= 13) {
    Serial.println("Bin is full.");
    if (type == "plastic") {
      blinkLED(redLEDPinPlastic, 5);
      digitalWrite(redLEDPinPlastic, HIGH);
      isPlasticBinFull = true;
    } else {
      blinkLED(redLEDPinNonPlastic, 5);
      digitalWrite(redLEDPinNonPlastic, HIGH);
      isNonPlasticBinFull = true;
    }
    return;
  }

  // Bin not full
  Serial.println("Bin not full. Opening inputCup.");
  blinkLED(greenLEDPin, 5);
  inputCup.write(0);

  while (true) {
    int irVal = analogRead(irSensorPin);
    if (irVal < 1 || irVal > (740 + tolerance)) break;
    delay(200);
  }

  delay(2000);
  inputCup.write(90);
}

// --- IR Sensor Handling with Empirical Optimization ---
void handleIRSensor() {
  int irValue = analogRead(irSensorPin);

  if (!objectDetected) {
    if (irValue >= (300 - tolerance) && irValue <= (740 + tolerance)) {
      Serial.println("IR detected PLASTIC");
      materialStatus = "Plastic";
      objectDetected = true;

      if (!isPlasticBinFull) {
        processSorting("plastic");
      } else {
        Serial.println("Plastic bin still full, waiting...");
      }
    } else if (irValue >= (1 - tolerance) && irValue < (299 + tolerance)) {
      Serial.println("IR detected NON-PLASTIC");
      materialStatus = "Non-Plastic";
      objectDetected = true;

      if (!isNonPlasticBinFull) {
        processSorting("nonplastic");
      } else {
        Serial.println("Non-plastic bin still full, waiting...");
      }
    } else {
      materialStatus = "Empty";
    }
  } else {
    // Recheck if bin is no longer full and object is still present
    if (irValue >= (300 - tolerance) && irValue <= (740 + tolerance) && !isPlasticBinFull && materialStatus == "Plastic") {
      Serial.println("Plastic bin is now available, continuing...");
      processSorting("plastic");
    } else if (irValue >= (1 - tolerance) && irValue < (299 + tolerance) && !isNonPlasticBinFull && materialStatus == "Non-Plastic") {
      Serial.println("Non-plastic bin is now available, continuing...");
      processSorting("nonplastic");
    }

    if (irValue < 1 || irValue > 741) {
      objectDetected = false;
      inputCup.write(90);
      materialStatus = "Empty";
    }
  }
}

// --- Main Loop ---
void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    Serial.print("Received: ");
    Serial.println(command);

    if (command == "plastic") {
      Serial.println("Manual command: PLASTIC");
      materialStatus = "Plastic";
      processSorting("plastic");
    } else if (command == "nonplastic") {
      Serial.println("Manual command: NON-PLASTIC");
      materialStatus = "Non-Plastic";
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

  if (millis() - lastDistancePrint >= distancePrintInterval) {
    lastDistancePrint = millis();

    float rawDistancePlastic = measureDistance(trigPinPlastic, echoPinPlastic);
    float filteredDistancePlastic = updateDistanceBuffer(rawDistancePlastic, distanceBufferPlastic, bufferIndexPlastic, bufferFilledPlastic);
    float rawDistanceNonPlastic = measureDistance(trigPinNonPlastic, echoPinNonPlastic);
    float filteredDistanceNonPlastic = updateDistanceBuffer(rawDistanceNonPlastic, distanceBufferNonPlastic, bufferIndexNonPlastic, bufferFilledNonPlastic);
    int currentIR = analogRead(irSensorPin);

    // Reset red LED if bin is no longer full
    if (filteredDistancePlastic > 13 && isPlasticBinFull) {
      digitalWrite(redLEDPinPlastic, LOW);
      isPlasticBinFull = false;
    }
    if (filteredDistanceNonPlastic > 13 && isNonPlasticBinFull) {
      digitalWrite(redLEDPinNonPlastic, LOW);
      isNonPlasticBinFull = false;
    }

    Serial.print("RAW_PLASTIC:");
    Serial.print(rawDistancePlastic, 1);
    Serial.print(",FILTERED_PLASTIC:");
    Serial.print(filteredDistancePlastic, 1);
    Serial.print(",RAW_NONPLASTIC:");
    Serial.print(rawDistanceNonPlastic, 1);
    Serial.print(",FILTERED_NONPLASTIC:");
    Serial.print(filteredDistanceNonPlastic, 1);
    Serial.print(",IR:");
    Serial.print(currentIR);
    Serial.print(",STATUS:");
    Serial.println(materialStatus);

  }
}
