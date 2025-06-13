#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>

// WiFi AP credentials
const char* ssid = "Trashcan-ESP32";
const char* password = "trashcan123";

// Web server and WebSocket
WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);

// UART2 (pins 16/17 on ESP32)
#define RXD2 16
#define TXD2 17
HardwareSerial MySerial(2); // UART2

// Buffers
#define MAX_SENSOR_LINES 10
#define MAX_LOG_LINES 20
String sensorReadingsLines[MAX_SENSOR_LINES];
String logLines[MAX_LOG_LINES];
int sensorLineIndex = 0;
int logIndex = 0;

// Add to sensor buffer
void addSensorLine(const String& line) {
  sensorReadingsLines[sensorLineIndex] = line;
  sensorLineIndex = (sensorLineIndex + 1) % MAX_SENSOR_LINES;
}

// Add to log buffer
void addLogLine(const String& line) {
  logLines[logIndex] = line;
  logIndex = (logIndex + 1) % MAX_LOG_LINES;
}

// Process incoming line
void processSerialLine(String line) {
  line.trim();
  if (line.startsWith("RAW_")) {
    addSensorLine(line);
  } else if (line.length() > 0) {
    addLogLine(line);
  }
}

// Build sensor reading string
String buildSensorReadings() {
  String result = "";
  for (int i = 0; i < MAX_SENSOR_LINES; i++) {
    int idx = (sensorLineIndex + i) % MAX_SENSOR_LINES;
    if (sensorReadingsLines[idx].length() > 0) {
      result += sensorReadingsLines[idx] + "\n";
    }
  }
  return result;
}


// Build event log string
String buildEventLog() {
  String result = "";
  for (int i = 0; i < MAX_LOG_LINES; i++) {
    int idx = (logIndex + i) % MAX_LOG_LINES;
    if (logLines[idx].length() > 0) {
      result += logLines[idx] + "\n";
    }
  }
  return result;
}

// Send data to all WebSocket clients
void sendData() {
  String sensorData = buildSensorReadings();
  String eventLog = buildEventLog();

  sensorData.replace("\n", "\\n");
  eventLog.replace("\n", "\\n");

  String json = "{";
  json += "\"sensorReadings\":\"" + sensorData + "\",";
  json += "\"eventLog\":\"" + eventLog + "\"";
  json += "}";

  webSocket.broadcastTXT(json);
}

// Serve HTML dashboard
void handleRoot() {
  String html = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>Smart Trashcan Dashboard</title>
  <style>
    body { font-family: Arial, sans-serif; background: #f0f2f5; margin: 0; padding: 20px; }
    h1 { text-align: center; color: #333; }
    .container { max-width: 800px; margin: auto; }
    .box {
      background: white;
      border-radius: 8px;
      box-shadow: 0 2px 6px rgba(0,0,0,0.15);
      padding: 15px 20px;
      margin-bottom: 20px;
    }
    .box h2 {
      margin-top: 0;
      border-bottom: 2px solid #007acc;
      padding-bottom: 5px;
      color: #007acc;
    }
    pre {
      background: #222;
      color: #0f0;
      padding: 10px;
      border-radius: 5px;
      height: 160px;
      overflow-y: auto;
      font-family: monospace;
      white-space: pre-wrap;
    }
  </style>
</head>
<body>
  <div class="container">
    <h1>Smart Trashcan Dashboard</h1>
    <div class="box">
      <h2>Sensor Readings</h2>
      <pre id="sensorReadings">Waiting for data...</pre>
    </div>
    <div class="box">
      <h2>Event Log</h2>
      <pre id="eventLog">Waiting for events...</pre>
    </div>
  </div>

  <script>
    var sensorElem = document.getElementById('sensorReadings');
    var eventElem = document.getElementById('eventLog');

    var gateway = `ws://${window.location.hostname}:81/`;
    var websocket = new WebSocket(gateway);

    websocket.onopen = function(event) {
      console.log("WebSocket connected");
    };

    websocket.onclose = function(event) {
      console.log("WebSocket disconnected, reconnecting...");
      setTimeout(() => {
        websocket = new WebSocket(gateway);
      }, 2000);
    };

    websocket.onmessage = function(event) {
      try {
        let data = JSON.parse(event.data);
        sensorElem.innerText = data.sensorReadings || "No sensor data";
        eventElem.innerText = data.eventLog || "No events";
        eventElem.scrollTop = eventElem.scrollHeight;
      } catch(e) {
        console.error("Invalid JSON", e);
      }
    };
  </script>
</body>
</html>
  )rawliteral";

  server.send(200, "text/html", html);
}

void setup() {
  Serial.begin(115200);
  MySerial.begin(9600, SERIAL_8N1, RXD2, TXD2);

  WiFi.softAP(ssid, password);
  Serial.print("AP IP address: ");
  Serial.println(WiFi.softAPIP());

  server.on("/", handleRoot);
  server.begin();
  webSocket.begin();
  webSocket.onEvent([](uint8_t num, WStype_t type, uint8_t *payload, size_t length) {});
}

void loop() {
  server.handleClient();
  webSocket.loop();

  static String incomingLine = "";
  while (MySerial.available()) {
    char c = MySerial.read();
    if (c == '\n' || c == '\r') {
      if (incomingLine.length() > 0) {
        processSerialLine(incomingLine);
        incomingLine = "";
        sendData(); // Update UI
      }
    } else {
      incomingLine += c;
    }
  }
}
