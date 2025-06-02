      #include <Stepper.h>
      #include <Servo.h>

      const int stepsPerRevolution = 2048;
      Stepper stepper(stepsPerRevolution, 8, 10, 9, 11);

      Servo secondServo;
      const int secondServoPin = 6;

      int currentAngle = 0;

      const int irSensorPin = A0; // IR sensor connected to analog pin A0
      const int trigPin = 4;      // HC-SR04 trigger pin
      const int echoPin = 5;      // HC-SR04 echo pin

      unsigned long lastIRCheck = 0;
      const unsigned long irCheckInterval = 500; // check every 500ms

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

        secondServo.attach(secondServoPin);
        secondServo.write(0); // Start at 0 degrees

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

        disableStepper(); // Turn off coils
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

      void loop() {
        // Check for serial input
        if (Serial.available() > 0) {
          String command = Serial.readString();
          command.trim(); // Remove any leading/trailing whitespace

          if (command == "plastic") {
            Serial.println("Manual command: PLASTIC");
            processSorting("plastic");
          } else if (command == "nonplastic") {
            Serial.println("Manual command: NON-PLASTIC");
            processSorting("nonplastic");
          } else {
            Serial.print("Unknown command: ");
            Serial.println(command);
          }
        }

        // Periodically check IR sensor
        if (millis() - lastIRCheck >= irCheckInterval) {
          lastIRCheck = millis();
          handleIRSensor();
        }
      }
