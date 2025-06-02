#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <ArduinoJson.h>
#include <Stepper.h>
#include <Servo.h>

// Replace with your network credentials
const char* ssid = "Rizza's A25";
const char* password = "12345678";

// Stepper motor configuration
const int stepsPerRevolution = 2048;
Stepper stepper(stepsPerRevolution, 8, 10, 9, 11);

// Servo motor configuration
Servo secondServo;
const int secondServoPin = 6;

// Sensor pins
const int irSensorPin = A0; // IR sensor connected to analog pin A0
const int trigPin = 4;      // HC-SR04 trigger pin
const int echoPin = 5;      // HC-SR04 echo pin

// Global variables
int currentAngle = 0;
String mode = "Auto"; // Default mode
unsigned long lastIRCheck = 0;
const unsigned long irCheckInterval = 500; // check every 500ms

AsyncWebServer server(80);

// Function to disable the stepper motor
void disableStepper() {
  digitalWrite(8, LOW);
  digitalWrite(10, LOW);
  digitalWrite(9, LOW);
  digitalWrite(11, LOW);
  delay(500); 
  Serial.println("Stepper disabled");
}

// Function to rotate the stepper motor to a specific angle
void rotateTo(int targetAngle) {
  Serial.print("Rotating to: ");
  Serial.println(targetAngle);

  int stepTarget = map(targetAngle, 0, 360, 0, stepsPerRevolution);
  int stepCurrent = map(currentAngle, 0, 360, 0, stepsPerRevolution);
  int stepsToMove = stepTarget - stepCurrent;

  stepper.step(stepsToMove);
  currentAngle = targetAngle;

  disableStepper(); // Turn off coils
}

// Function to measure distance using the ultrasonic sensor
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

// Function to measure average distance over a period
float measureAverageDistance() {
  const unsigned long measurementDuration = 1000; // 1 second
  unsigned long startTime = millis();
  float sum = 0;
  int count = 0;

  while (millis() - startTime < measurementDuration) {
    float distance = measureDistance();
    sum += distance;
    count++;
    delay(50); // Short delay between measurements
  }

  float average = (count > 0) ? (sum / count) : 0;
  Serial.print("Average Distance: ");
  Serial.print(average);
  Serial.println(" cm");
  return average;
}

// Function to process sorting based on type
void processSorting(String type) {
  if (type == "plastic") {
    rotateTo(180);
  } else if (type == "nonplastic") {
    rotateTo(0);
  }

  delay(1000); // Wait 1 second after rotating

  float averageDistance = measureAverageDistance();
  if (averageDistance >= 0 && averageDistance <= 5) {
    Serial.println("Bin is full. No action taken.");
    return;
  }

  // Bin is not full, activate second servo
  Serial.println("Bin is not full. Activating second servo.");
  secondServo.write(90);

  // Wait until object is no longer detected
  while (analogRead(irSensorPin) >= 1 && analogRead(irSensorPin) <= 25) {
    delay(200);
  }

  delay(2000); // Wait 2 seconds after object removed
  secondServo.write(0);
}

// Function to handle IR sensor readings in Auto mode
void handleIRSensor() {
  int irValue = analogRead(irSensorPin);
  Serial.print("IR value: ");
  Serial.println(irValue);

  if (irValue >= 300 && irValue <= 740) {
    Serial.println("IR detected PLASTIC");
    processSorting("plastic");
  } 
  else if (irValue >= 0 && irValue < 299) {
    Serial.println("IR detected NON-PLASTIC");
    processSorting("nonplastic");
  }
  else {
    Serial.println("No object detected.");
  }
}

// Function to get sensor data in JSON format
String getSensorData() {
  int irValue = analogRead(irSensorPin);
  float distance = measureDistance();

  DynamicJsonDocument json(1024);
  json["irValue"] = irValue;
  json["distance"] = distance;
  json["mode"] = mode;

  String response;
  serializeJson(json, response);
  return response;
}

void setup() {
  Serial.begin(115200);

  // Initialize stepper motor
  stepper.setSpeed(10);
  pinMode(8, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(11, OUTPUT);

  // Initialize servo motor
  secondServo.attach(secondServoPin);
  secondServo.write(0); // Start at 0 degrees

  // Initialize sensors
  pinMode(irSensorPin, INPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Connected! IP address: ");
  Serial.println(WiFi.localIP());

  // Serve the main page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", R"rawliteral(
      <!DOCTYPE html>
      <html>
      <head>
        <title>ESP32 Sorting System</title>
        <script>
          function fetchData() {
            fetch('/getSensorData')
              .then(response => response.json())
              .then(data => {
                document.getElementById('irValue').innerText = data.irValue;
                document.getElementById('distance').innerText = data.distance.toFixed(2);
                document.getElementById('mode').innerText = data.mode;
              });
          }

          function setMode(newMode) {
            fetch('/setMode?mode=' + newMode);
          }

          function manualSort(type) {
            fetch('/manualSort?type=' + type);
          }

          setInterval(fetchData, 1000);
        </script>
      </head>
      <body>
        <h1>ESP32 Sorting System</h1>
        <p>IR Sensor Value: <span id="irValue">0</span></p>
        <p>Ultrasonic Distance: <span id="distance">0.00</span> cm</p>
        <p>Current Mode: <span id="mode">Auto</span></p>

        <h2>Mode Selection</h2>
        <button onclick="setMode('Auto')">Auto Mode</button>
        <button onclick="setMode('Manual')">Manual Mode</button>

        <h2>Manual Sorting</h2>
        <button onclick="manualSort('plastic')">Plastic</button>
        <button onclick="manualSort('nonplastic')">Non-Plastic</button>
      </body>
      </html>
    )rawliteral");
  });

  // Endpoint to get sensor data
  server.on("/getSensorData", HTTP_GET, [](AsyncWebServerRequest *request){
    String data = getSensorData();
    request->send(200, "application/json", data);
  });

  // Endpoint to set mode
  server.on("/setMode", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("mode")) {
      mode = request->getParam("mode")->value();
      Serial.print("Mode set to: ");
      Serial.println(mode);
    }
    request->send(200, "text/plain", "Mode updated");
  });

  // Endpoint for manual sorting
  server.on("/manualSort", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("type")) {
      String type = request->getParam("type")->value();
      Serial.print("Manual sort: ");
      Serial.println(type);
      processSorting(type);
    }
    request->send(200, "text/plain", "Manual sort executed");
  });

  server.begin();
  Serial.println("Web server started");
}

void loop() {
  // Periodically check IR sensor in Auto mode
  if (mode == "Auto" && millis() - lastIRCheck >= irCheckInterval) {
    lastIRCheck = millis();
    handleIRSensor();
  }
}
